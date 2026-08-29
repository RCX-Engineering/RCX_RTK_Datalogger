/*
 * webserver.cpp — ESPAsyncWebServer UI
 * =====================================
 * Pages (see webserver.h): / dashboard, /setup, /logs, plus the shared
 * /app.css and /app.js.
 *
 * Data routes (all dynamic responses carry Cache-Control: no-store):
 *   GET  /status       JSON — GPS, RTK, NTRIP, BLE, CAN live state
 *   GET  /sd           JSON — SD card capacity + file count
 *   GET  /files        JSON listing. Default: currently-open files with live
 *                       sizes. ?recent=1: newest 12. ?all=1[&offset=N&limit=M]:
 *                       the whole listing, one bounded page at a time, with
 *                       "total" for paging and "active":true on open files.
 *   GET  /log/<f>      Chunked download of a log file (no SD mutex — FatFS is
 *                       internally serialised; holding the log mutex across every
 *                       chunk would trigger premature EOF on a busy write cycle)
 *   DELETE /log/<f>    Delete a single .csv file
 *   POST /log/delete   Delete a batch (body: names=a.csv,b.csv,...), bounded to
 *                       DELETE_BATCH_MAX per request
 *   GET  /log_config   JSON — per-channel log enable flags + master pause state
 *   POST /log_config   Toggle flags (URL params: gps=0|1 imu=0|1 can=0|1 sat=0|1)
 *   GET  /log_pause    JSON — master pause state
 *   POST /log_pause    Set it (URL param: on=0|1). Suspends every channel and
 *                       releases the files they hold open, so those files can be
 *                       downloaded or deleted. RAM only — a reboot resumes.
 *   GET  /debug_log    JSON — SD debug-log (serial mirror) enable state
 *   POST /debug_log    Toggle enable (URL param: en=0|1). No-op if config.h's
 *                       DEBUG_SERIAL_TO_SD_FORCE_ON has forced it on. "supported":false
 *                       means this build wasn't compiled with DEBUG_SERIAL_TO_SD at all.
 *   GET  /lcd          JSON — current LCD/backlight enable state (RAM only,
 *                       always true after a reboot — see display.h)
 *   POST /lcd          Toggle it (URL param: on=0|1)
 *   GET  /imu/cal      JSON — IMU calibration state {state,calibrated}
 *   POST /imu/cal      Start or clear a calibration (URL param: action=start|clear)
 *   POST /sd/deleteold Delete all .csv files except the currently-open one
 */

#include "webserver.h"
#include "config.h"
#include "gnss.h"
#include "types.h"
#include "sd_log.h"
#include "ntrip.h"
#include "can_bus.h"     // CAN sniffer toggle + snapshot
#include "wifi_mgr.h"    // station-list management routes + device identity
#include "ntrip.h"       // caster-list management routes
#include "dbc_store.h"   // user-supplied CAN database storage
#include "dbc_parse.h"   // import audit report
#include "display.h"     // display_setEnabled()/display_isEnabled() — /lcd route
#include "imu.h"         // imu_requestCalibration() etc. — /imu/cal route
#include <memory>        // std::shared_ptr / std::make_shared
#include <string>        // std::string / std::to_string

// Forward-declare just the 3 functions the /debug_log route needs, instead of
// #include "debug_log.h": that header redirects Serial -> Debug when
// DEBUG_SERIAL_TO_SD is compiled in, and this file is NOT one of the tee-mirrored
// translation units (see debug_log.h) and has ~15 pre-existing Serial.print* call
// sites of its own. Undoing the redirect with #undef is exactly what caused a real
// compile break (Serial becomes fully unresolvable, not "reverted" — see debug_log.cpp
// for the full explanation) — so webserver.cpp simply never lets the macro apply here.
#if defined(DEBUG_SERIAL_TO_SD) && DEBUG_SERIAL_TO_SD
namespace DebugLog {
    bool isEnabled();
    bool isForced();
    void setEnabled(bool on);
}
#endif

#if defined(WEBSERVER_ENABLE) && WEBSERVER_ENABLE

#include <ESPAsyncWebServer.h>
#include <WiFi.h>
#include <SD_MMC.h>
#include <vector>
#include <esp_heap_caps.h>   // heap_caps_get_free_size for /status heap field
#include <esp_task_wdt.h>    // feed the async_tcp WDT during streamed SD reads

static AsyncWebServer webServer(WEBSERVER_PORT);
static bool webBegun = false;

// ── Shared-asset cache tag ────────────────────────────────────────────────────
// Identifies this build's /app.css and /app.js to the browser cache. Derived
// from the compiler's own build stamp, so it changes on every rebuild and a
// reflashed device can never serve a page against a stale cached script.
#define ASSET_ETAG "\"" __DATE__ __TIME__ "\""

// ── File-view paging bounds ───────────────────────────────────────────────────
// /files?all=1 answers one page at a time. The response is built in a std::string
// on the internal heap — the resource this board is tightest on, and the same one
// an NTRIP source-table scan needs ~30 KB of — so the entry count per response is
// capped here rather than growing with whatever the card happens to hold. MAX also
// sizes the static copy-out buffers in the handler, which is why it's kept equal
// to DEFAULT: the dashboard never asks for more than DEFAULT per page, so a larger
// MAX would only be unconditionally-reserved memory nothing ever uses.
#define FILES_PAGE_DEFAULT  50
#define FILES_PAGE_MAX      50

// Filenames accepted by one /log/delete request. Each request holds sdMutex for
// its whole run, so this bounds how long the logging writes (which wait only
// 50 ms for that mutex before dropping their row) can be starved by a bulk
// delete. The dashboard splits a larger selection into successive requests.
#define DELETE_BATCH_MAX    25

// In-flight file downloads (single-file + .tar). Used to block deletes while a
// download holds a file open: SD_MMC.remove() on a file that's open elsewhere
// corrupts FatFs state and resets the device. Touched only from the (serial)
// async_tcp task — DlState ctor/dtor — so a plain volatile int is sufficient.
static volatile int g_activeDownloads = 0;

// Download filename for the archive built by the last /export/build. Empty falls
// back to the generic names in the /export.tar handler. Written and read only on
// the (serial) async_tcp task, and only between a build request and its download.
//
// The value reaches a Content-Disposition header, so it is rebuilt character by
// character from a strict whitelist rather than trusted: a quote or a CRLF in a
// response header is a header-injection, and anything outside this set has no
// business in a filename anyway.
static char exportFileName[48] = "";

static void setExportFileName(const String& in) {
    size_t out = 0;
    for (size_t i = 0; i < in.length() && out < sizeof(exportFileName) - 1; i++) {
        const char c = in[i];
        const bool ok = (c >= 'A' && c <= 'Z') || (c >= 'a' && c <= 'z') ||
                        (c >= '0' && c <= '9') || c == '_' || c == '-' || c == '.';
        if (ok) exportFileName[out++] = c;
    }
    exportFileName[out] = '\0';
    // Must still be a plausible archive name after filtering, and must not have
    // been reduced to a relative path by it.
    const bool tar = out > 4 && strcmp(exportFileName + out - 4, ".tar") == 0;
    if (!tar || strstr(exportFileName, "..")) exportFileName[0] = '\0';
}

// Internal-RAM low-water marks, sampled in loop() (defined in the .ino). Surfaced
// on /status so the dashboard can show the worst dip over a long run.
extern volatile uint32_t g_minFreeInternal;
extern volatile uint32_t g_minLargestBlock;

// ── HTML dashboard ────────────────────────────────────────────────────────────
// Stored as a raw string literal in rodata (PROGMEM is a no-op on ESP32 — it
// just keeps the attribute in the linker script section; send_P reads it
// directly from flash-mapped memory with no copy to SRAM needed).
// ── Shared stylesheet ─────────────────────────────────────────────────────────
// Served once from /app.css instead of being repeated inside each of the three
// pages. Raw string literals live in rodata (PROGMEM is a no-op on ESP32 — it
// just keeps the attribute in the linker script section; send_P reads straight
// from flash-mapped memory with no copy into SRAM).
static const char WEB_CSS[] PROGMEM = R"css(
:root{
--bg:#eef1f5;--surface:#fff;--surface2:#f6f8fa;
--text:#14181d;--muted:#5a626d;--border:#c8cfd8;
--ok-bg:#0f7a37;--ok-fg:#fff;--warn-bg:#e6a100;--warn-fg:#1a1400;
--bad-bg:#c62330;--bad-fg:#fff;--neutral-fg:#1b2027;
--accent:#0b60c9;--accent-fg:#fff}
*{box-sizing:border-box;margin:0;padding:0}
body{background:var(--bg);color:var(--text);
font-family:-apple-system,BlinkMacSystemFont,"Segoe UI",Roboto,Helvetica,Arial,sans-serif;
font-variant-numeric:tabular-nums;-webkit-text-size-adjust:100%;
padding:1rem;font-size:15px;line-height:1.35}
h1{font-size:1.3rem;font-weight:800;letter-spacing:-.01em;margin-bottom:.6rem;display:flex;align-items:center;gap:.4rem}
h2{font-size:.95rem;font-weight:700;margin:1.1rem 0 .5rem;display:flex;align-items:center;gap:.4rem}
nav{display:flex;gap:.35rem;margin-bottom:.9rem;flex-wrap:wrap}
nav a{background:var(--surface);border:1px solid var(--border);color:var(--text);
padding:.4rem .85rem;border-radius:8px;text-decoration:none;font-size:.85rem;font-weight:700}
nav a.on{background:var(--accent);border-color:var(--accent);color:var(--accent-fg)}
.grid{display:grid;grid-template-columns:repeat(auto-fit,minmax(145px,1fr));gap:.5rem;margin-bottom:.5rem}
.card{background:var(--surface);border:1px solid var(--border);border-radius:8px;
padding:.5rem .7rem;box-shadow:0 1px 2px rgba(20,24,29,.06)}
.label{font-size:.66rem;font-weight:700;color:var(--muted);text-transform:uppercase;letter-spacing:.06em;margin-bottom:.22rem}
.val{font-size:1.05rem;font-weight:700;color:var(--text);word-break:break-word}
.val.ok,.val.warn,.val.bad{display:inline-block;padding:.1rem .5rem;border-radius:999px;font-size:.92rem;line-height:1.35}
.val.ok{background:var(--ok-bg);color:var(--ok-fg)}
.val.warn{background:var(--warn-bg);color:var(--warn-fg)}
.val.bad{background:var(--bad-bg);color:var(--bad-fg)}
.val.neutral{color:var(--neutral-fg)}
.empty{color:var(--muted);font-style:italic}
.files{display:flex;flex-direction:column;gap:.35rem}
.row{background:var(--surface);border:1px solid var(--border);border-radius:8px;padding:.45rem .7rem;
display:flex;justify-content:space-between;align-items:center;flex-wrap:wrap;gap:.35rem}
.fn{font-size:.85rem;font-weight:600;color:var(--text)}.sz{font-size:.74rem;color:var(--muted);margin-left:.4rem}
.rec{color:var(--ok-bg);font-weight:700}
.btns{display:flex;gap:.4rem;align-items:center}
a.dl{background:var(--accent);border:1px solid var(--accent);color:var(--accent-fg);padding:.3rem .65rem;
border-radius:6px;text-decoration:none;font-size:.78rem;font-weight:600;white-space:nowrap}
a.dl:active{filter:brightness(.9)}
button.del{background:var(--surface);border:1px solid var(--bad-bg);color:var(--bad-bg);padding:.3rem .65rem;
border-radius:6px;cursor:pointer;font:inherit;font-size:.78rem;font-weight:600;white-space:nowrap}
button.del:active{background:var(--bad-bg);color:#fff}
button.del:disabled{opacity:.45;cursor:not-allowed}
.trow{background:var(--surface);border:1px solid var(--border);border-radius:8px;padding:.55rem .75rem;
display:flex;justify-content:space-between;align-items:center;margin-bottom:.4rem;gap:.5rem}
.tlabel{font-size:.9rem;font-weight:700;color:var(--text)}.tdesc{font-size:.7rem;color:var(--muted);margin-top:.12rem}
.tin{width:76px;background:var(--surface);border:1px solid var(--border);color:var(--text);border-radius:6px;padding:.35rem .5rem;font:inherit;font-size:.95rem;font-weight:600;text-align:right}
.sw{position:relative;width:46px;height:26px;flex-shrink:0}
.sw input{opacity:0;width:0;height:0}
.sw .sl{position:absolute;cursor:pointer;inset:0;background:#cfd6de;border:1px solid #b3bcc7;border-radius:26px;transition:.2s}
.sw .sl::before{content:'';position:absolute;width:20px;height:20px;left:2px;bottom:2px;background:#fff;border-radius:50%;transition:.2s;box-shadow:0 1px 2px rgba(0,0,0,.3)}
.sw input:checked+.sl{background:var(--ok-bg);border-color:var(--ok-bg)}
.sw input:checked+.sl::before{transform:translateX(20px)}
.sw input:disabled+.sl{opacity:.55;cursor:not-allowed}
.sh{color:var(--muted);font-size:.72rem;font-weight:700;text-transform:uppercase;letter-spacing:.08em;
margin:.7rem 0 .3rem;padding-bottom:.2rem;border-bottom:1px solid var(--border)}
.grouphead{display:flex;justify-content:space-between;align-items:baseline;flex-wrap:wrap;gap:.3rem .6rem;
font-size:.9rem;font-weight:800;color:var(--text);margin:.9rem 0 .35rem;padding-bottom:.25rem;
border-bottom:2px solid var(--border)}
.grouphead .gmeta{font-size:.68rem;font-weight:600;color:var(--muted);white-space:nowrap}
.grouphead .gbtns{display:flex;gap:.35rem;flex:0 0 auto}
.grouphead .gbtns button{padding:.22rem .5rem;font-size:.72rem}
button.ref{background:var(--surface);border:1px solid var(--border);color:var(--text);padding:.3rem .6rem;
border-radius:6px;cursor:pointer;font:inherit;font-size:.78rem;font-weight:600}
button.ref:active{background:var(--surface2)}
button.ref:disabled{opacity:.45;cursor:not-allowed}
.sdbar{background:var(--surface);border:1px solid var(--border);border-radius:8px;padding:.55rem .75rem;
margin-bottom:.5rem;display:flex;align-items:center;gap:.75rem;flex-wrap:wrap}
.sdbar .label{margin-bottom:0;margin-right:.2rem}
progress{-webkit-appearance:none;appearance:none;width:130px;height:10px;border-radius:5px;overflow:hidden}
progress::-webkit-progress-bar{background:#dbe1e8;border-radius:5px}
progress::-webkit-progress-value{background:var(--ok-bg);border-radius:5px}
progress::-moz-progress-bar{background:var(--ok-bg)}
progress.warn::-webkit-progress-value{background:var(--warn-bg)}
progress.full::-webkit-progress-value{background:var(--bad-bg)}
.pausebar{background:var(--surface);border:1px solid var(--border);border-left-width:4px;border-radius:8px;
padding:.6rem .8rem;margin-bottom:.6rem;display:flex;align-items:center;gap:.75rem;flex-wrap:wrap}
.pausebar.rec{border-left-color:var(--ok-bg)}
.pausebar.paused{border-left-color:var(--warn-bg)}
button.big{padding:.5rem 1.1rem;border-radius:8px;cursor:pointer;font:inherit;font-size:.9rem;font-weight:700;
border:1px solid var(--accent);background:var(--accent);color:var(--accent-fg)}
button.big.warn{border-color:var(--warn-bg);background:var(--warn-bg);color:var(--warn-fg)}
button.big.plain{border-color:var(--border);background:var(--surface);color:var(--text)}
button.big:active{filter:brightness(.92)}
.frow{background:var(--surface);border:1px solid var(--border);border-radius:8px;padding:.45rem .7rem;
display:flex;justify-content:space-between;align-items:center;flex-wrap:wrap;gap:.35rem}
.tag{background:var(--ok-bg);color:var(--ok-fg);border-radius:999px;padding:.05rem .45rem;font-size:.7rem;font-weight:700}
.ck{width:18px;height:18px;flex-shrink:0;accent-color:var(--accent)}
.pager{display:flex;align-items:center;gap:.5rem;flex-wrap:wrap;margin:.5rem 0}
.pager .pos{font-size:.78rem;color:var(--muted);font-weight:600}
#toast{position:fixed;bottom:1rem;right:1rem;background:var(--surface);border:1px solid var(--ok-bg);
border-left-width:4px;color:var(--text);padding:.55rem 1rem;border-radius:6px;font-size:.82rem;font-weight:600;
box-shadow:0 3px 10px rgba(20,24,29,.18);opacity:0;transition:opacity .3s;pointer-events:none}
#toast.show{opacity:1}
)css";

// ── Shared page helpers ───────────────────────────────────────────────────────
// Served from /app.js and loaded by all three pages before their own script.
// Only genuinely cross-page helpers belong here; anything one page owns stays
// inline on that page so this file does not become a dumping ground.
static const char WEB_JS[] PROGMEM = R"js(
// Null-safe number formatter: returns '---' if v is not a finite number.
function fmt(v,dec){return(typeof v==='number'&&isFinite(v))?v.toFixed(dec):'---';}
// HTML-escape. SSIDs, hosts and filenames are operator-supplied and may contain
// & < > " ' — they are interpolated into markup, so never trusted verbatim.
function esc(s){return String(s==null?'':s).replace(/[&<>"']/g,function(c){
  return{'&':'&amp;','<':'&lt;','>':'&gt;','"':'&quot;',"'":'&#39;'}[c];});}
function fsize(b){return b>=1048576?(b/1048576).toFixed(2)+' MB':b>=1024?(b/1024).toFixed(1)+' KB':b+' B';}
// ── Request sequencing ────────────────────────────────────────────────────────
// Every request in flight costs internal SRAM — an AsyncWebServerRequest, lwIP
// pbufs, TCP control state — and that cost is roughly the same whether the body
// is 200 bytes or 2 KB. Internal SRAM is what this board runs closest to the
// edge on, so what matters is how many sockets are open at once, not how much
// data they carry. chain() runs a list of loaders strictly one after another, so
// the page never has more than one request outstanding.
//
// A failing loader must not stall the ones behind it, so each step swallows its
// own error.
function chain(fns){
  return fns.reduce(function(p,f){
    return p.then(function(){
      try{ return Promise.resolve(f()).catch(function(){}); }
      catch(e){ return null; }
    });
  }, Promise.resolve());
}
// Poll on completion rather than on a fixed interval: the next cycle is queued
// only once the previous has finished, so a slow link stretches the cadence
// instead of stacking up requests that were never going to keep pace. pick(tick)
// returns the loaders due on that cycle, which is how a page mixes fast and slow
// refresh rates without running two timers that can overlap.
function pollLoop(pick,ms){
  var tick=0;
  (function step(){
    chain(pick(tick++)).then(function(){ setTimeout(step,ms); });
  })();
}
function toast(msg,ok){
  var t=document.getElementById('toast');
  if(!t)return;
  t.textContent=msg; t.style.borderColor=ok?'var(--ok-bg)':'var(--bad-bg)'; t.style.color=ok?'var(--ok-bg)':'var(--bad-bg)';
  t.classList.add('show'); setTimeout(function(){t.classList.remove('show');},2500);}
function loadSD(){
  var el=document.getElementById('sdbar');
  if(!el)return;
  return fetch('/sd').then(function(r){return r.json();}).then(function(d){
    if(!d.ready){el.innerHTML='<span class="empty">SD not present or initialising&hellip;</span>';return;}
    var pct=d.total>0?d.used/d.total:0;
    var cls=pct>0.9?'full':pct>0.7?'warn':'';
    var freeStr=(((d.total-d.used)/1048576)>=1000)?
      ((d.total-d.used)/1073741824).toFixed(2)+' GB free':
      ((d.total-d.used)/1048576).toFixed(0)+' MB free';
    var totalStr=d.total>=1073741824?(d.total/1073741824).toFixed(1)+' GB':(d.total/1048576).toFixed(0)+' MB';
    el.innerHTML=
      '<span><span class="label">Free</span><span class="val '+(pct>0.9?'bad':pct>0.7?'warn':'ok')+'">'+freeStr+'</span></span>'+
      '<progress value="'+pct+'" max="1" class="'+cls+'"></progress>'+
      '<span><span class="label">Total</span><span class="val">'+totalStr+'</span></span>'+
      '<span><span class="label">Files</span><span class="val">'+d.files+'</span></span>';
  }).catch(function(){el.innerHTML='<span class="empty">SD unavailable</span>';});}
// Master pause bar — present on the dashboard and the log page, same markup and
// same endpoint, so the two never disagree about what is recording.
function renderPause(p){
  window._paused=!!p.paused;
  var bar=document.getElementById('pausebar');
  if(!bar)return;
  bar.className='pausebar '+(p.paused?'paused':'rec');
  bar.innerHTML='<span><span class="label">Logging</span><span class="val '+(p.paused?'warn':'ok')+'">'+
    (p.paused?'PAUSED':'RECORDING')+'</span></span>'+
    // Colour marks the state that wants attention: paused is the abnormal one to
    // get out of, so Resume is yellow. Pause is an ordinary action — plain white.
    '<button class="big '+(p.paused?'warn':'plain')+'" onclick="setPause('+(p.paused?'0':'1')+')">'+
    (p.paused?'\u25B6 Resume logging':'\u23F8 Pause logging')+'</button>'+
    '<span class="tdesc" style="flex:1;min-width:180px">'+
    (p.paused?'Files are closed. Downloads and deletes are available. Resuming starts new files.'
             :'Pause to download or delete files \u2014 a transfer competes with the recording writes for the SD card and will stall the device.')+
    '</span>';}
function loadPause(){
  return fetch('/log_pause').then(function(r){return r.json();}).then(renderPause).catch(function(){});}
function setPause(on){
  fetch('/log_pause?on='+on,{method:'POST'}).then(function(r){return r.json();}).then(function(d){
    renderPause(d);
    toast(d.paused?'Logging paused':'Logging resumed',!d.paused);
    // Files close/open on the logger task's next pass, so re-read shortly after.
    if(window.afterPauseChange)setTimeout(window.afterPauseChange,1200);
  }).catch(function(){toast('Pause failed',false);});}
)js";

// ── Page: dashboard ───────────────────────────────────────────────────────────
static const char PAGE_DASH[] PROGMEM = R"html(<!DOCTYPE html>
<html lang="en"><head>
<meta charset="UTF-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<meta name="color-scheme" content="light">
<title>RCX RTK Datalogger</title>
<link rel="stylesheet" href="/app.css">
</head><body>
<h1>&#x1F6F0; RCX RTK Datalogger</h1>
<nav><a class="on" href="/">Dashboard</a><a href="/setup">Setup</a><a href="/logs">Log Files</a></nav>
<h2>Status <button class="ref" onclick="loadStat()">&#x21BB;</button></h2>
<div id="stat"><span class="empty">Loading&hellip;</span></div>
<h2>SD Card</h2>
<div class="sdbar" id="sdbar"><span class="empty">Loading&hellip;</span></div>
<h2>Logging Channels</h2>
<div class="pausebar" id="pausebar"><span class="empty">Loading&hellip;</span></div>
<div id="logcfg">
  <div class="trow"><div><div class="tlabel">GPS</div><div class="tdesc">Position &middot; RTK &middot; NTRIP &middot; accuracy &mdash; 20 Hz</div></div>
    <label class="sw"><input type="checkbox" id="tog_gps" onchange="setLog('gps',this.checked)"><span class="sl"></span></label></div>
  <div class="trow"><div><div class="tlabel">IMU</div><div class="tdesc">Accel &middot; Gyro &mdash; 50 Hz independent file</div></div>
    <label class="sw"><input type="checkbox" id="tog_imu" onchange="setLog('imu',this.checked)"><span class="sl"></span></label></div>
  <div class="trow"><div><div class="tlabel">CAN</div><div class="tdesc">RPM &middot; speeds &middot; oil &middot; steering &middot; gear &mdash; 20 Hz</div></div>
    <label class="sw"><input type="checkbox" id="tog_can" onchange="setLog('can',this.checked)"><span class="sl"></span></label></div>
  <div class="trow"><div><div class="tlabel">SAT</div><div class="tdesc">Satellites &middot; PRN &middot; Elev &middot; Azimuth &middot; SNR &mdash; 1 Hz</div></div>
    <label class="sw"><input type="checkbox" id="tog_sat" onchange="setLog('sat',this.checked)"><span class="sl"></span></label></div>
  <div class="trow"><div><div class="tlabel">LCD</div><div class="tdesc">Screen + backlight &mdash; always starts ON after a reboot regardless of this setting.</div></div>
    <label class="sw"><input type="checkbox" id="tog_lcd" onchange="setLcd(this.checked)"><span class="sl"></span></label></div>
  <div class="trow" id="row_dbg"><div><div class="tlabel">Debug log</div><div class="tdesc" id="dbg_desc">Serial output mirrored to debug_*.txt on SD &mdash; for field debugging when a serial monitor isn't available.</div><div id="dbg_dl"></div></div>
    <label class="sw"><input type="checkbox" id="tog_dbg" onchange="setDebugLog(this.checked)"><span class="sl"></span></label></div>
</div>
<div class="tdesc" id="chanhint" style="margin-top:.3rem"></div>
<div class="sh">Recording now</div>
<div class="files" id="curfiles"><span class="empty">Loading&hellip;</span></div>
<div class="trow" style="margin-top:.5rem"><div><div class="tlabel">CAN sniffer</div>
  <div class="tdesc">Capture ALL CAN IDs (live) &middot; raw frames &rarr; canraw_*.csv. For finding channels the built-in profiles don&#39;t decode.</div></div>
  <label class="sw"><input type="checkbox" id="tog_sniff" onchange="setSniff(this.checked)"><span class="sl"></span></label></div>
<div id="sniffmeta" class="empty" style="margin:6px 0"></div>
<div class="files" id="snifftbl"></div>
<div id="toast"></div>
<script src="/app.js"></script>
<script>
function grp(t,c){return'<div class="sh">'+t+'</div><div class="grid">'+c+'</div>';}
// rate stoplight: green if >=g, amber if >=w, red below; non-numeric -> neutral
function rateState(v,g,w){if(typeof v!=='number'||!isFinite(v))return'neutral';return v>=g?'ok':v>=w?'warn':'bad';}
// Stoplight state. ok=green pill, warn=amber pill, bad=red pill, neutral=plain.
// A field with a defined 'ok' target that is neither ok nor bad is intermediate (warn).
function stateOf(val,ok,bad,warn){
  if(bad&&val===bad)return'bad';
  if(warn&&val===warn)return'warn';
  if(ok&&val===ok)return'ok';
  if(ok)return'warn';
  return'neutral';}
function statS(label,val,state){
  return'<div class="card"><div class="label">'+label+'</div><div class="val '+(state||'neutral')+'">'+val+'</div></div>';}
function stat(label,val,ok,bad,warn){return statS(label,val,stateOf(val,ok,bad,warn));}
// ESP32-S3 die-temp stoplight (°F): <185 ok, 185-212 warn, >=212 bad — matches the LCD.
function tempState(f){
  if(typeof f!=='number'||!isFinite(f))return'neutral';
  return f>=212?'bad':f>=185?'warn':'ok';}
function loadStat(){
  return fetch('/status').then(function(r){return r.json();}).then(function(d){
    var conn=
      stat('WiFi',d.wifi,'connected','---')+
      stat('SSID',d.ssid,'','---')+
      stat('NTRIP',d.ntrip,'connected','---')+
      stat('NTRIP Mount',d.mount,'','---')+
      stat('Base Dist',d.base_km>=0?d.base_km.toFixed(1)+' km':'—','','—')+
      statS('RTCM',d.rtcm_bytes>0?d.rtcm_bytes+' B':'0 — not flowing',d.rtcm_bytes>0?'ok':'bad')+
      stat('BLE',d.ble,'connected','---')+
      stat('RC App',d.rc,'connected','---')+
      stat('BLE Hz',d.ble_hz+' Hz','','');
    var gps=
      stat('RTK',d.rtk,'FIXED','NO FIX')+
      statS('Corr Age',(typeof d.diff_age==='number'&&d.diff_age>=0)?d.diff_age.toFixed(1)+' s':'—',
           (typeof d.diff_age!=='number'||d.diff_age<0)?'neutral':(d.diff_age<=5)?'ok':(d.rtk_hold_s>0&&d.diff_age>=d.rtk_hold_s*0.8)?'bad':'warn')+
      stat('Sats',d.sats,'','0')+
      stat('Acc',d.acc,'','')+
      statS('GPS Hz',d.gps_hz+' Hz',rateState(parseFloat(d.gps_hz),15,5))+
      stat('Lat',fmt(d.lat,9),'','')+
      stat('Lon',fmt(d.lon,9),'','');
    var can=
      stat('CAN',d.can,'connected','no data')+
      statS('CAN/s',d.can_hz,rateState(parseFloat(d.can_hz),20,1))+
      stat('RPM',d.rpm,'','')+
      stat('TPS',d.tps,'','')+
      stat('Oil Temp',d.oil_c,'','');
    var health=
      statS('CPU Temp',(d.esp_temp_f&&d.esp_temp_f!=='--')?d.esp_temp_f+' &deg;F':'--',(d.esp_temp_f&&d.esp_temp_f!=='--')?tempState(parseFloat(d.esp_temp_f)):'neutral')+
      statS('Free RAM',(typeof d.heap==='number')?((d.heap/1024).toFixed(0)+' KB'+(typeof d.heap_min==='number'?(' &#x2193;'+(d.heap_min/1024).toFixed(0)):'')):'---',
           (typeof d.heap!=='number')?'neutral':(d.heap<50000)?'bad':(d.heap<80000)?'warn':'ok')+
      stat('Min free block',(typeof d.heap_block_min==='number')?((d.heap_block_min/1024).toFixed(0)+' KB'):'---','','');
    var h=grp('Connectivity',conn)+grp('GPS',gps)+grp('CAN',can)+grp('Software Health',health);
    document.getElementById('stat').innerHTML=h;
  }).catch(function(){document.getElementById('stat').innerHTML='<span class="empty">Unavailable</span>';});}
function loadLogConfig(){
  // POST is used for toggle writes so browsers never cache the state change.
  // GET is used here just to READ current state — no side effects.
  return fetch('/log_config').then(function(r){return r.json();}).then(function(d){
    document.getElementById('tog_gps').checked=!!d.gps;
    document.getElementById('tog_imu').checked=!!d.imu;
    document.getElementById('tog_can').checked=!!d.can;
    document.getElementById('tog_sat').checked=!!d.sat;
    // These switches show the channel set that WILL run; while paused nothing is
    // recording, so say so rather than letting the switches imply otherwise.
    var hint=document.getElementById('chanhint');
    if(hint)hint.textContent=d.paused
      ?'Logging is paused \u2014 these are the channels that will start again on resume.'
      :'';
  }).catch(function(){});}
function setLog(key,val){
  // POST so the browser never serves a cached toggle response for the same key+value.
  // Params stay in the URL query string — ESPAsyncWebServer parses them regardless of method.
  fetch('/log_config?'+key+'='+(val?'1':'0'),{method:'POST'})
    .then(function(r){
      if(r.ok){toast(key.toUpperCase()+' log '+(val?'ON':'OFF'),val);loadLogConfig();}
      else toast('Toggle failed',false);
    }).catch(function(){toast('Network error',false);});}
function loadLcd(){
  return fetch('/lcd').then(function(r){return r.json();}).then(function(d){
    var t=document.getElementById('tog_lcd'); if(t) t.checked=!!d.on;
  }).catch(function(){});}
function setLcd(val){
  fetch('/lcd?on='+(val?'1':'0'),{method:'POST'}).then(function(r){return r.json();}).then(function(d){
    toast('LCD '+(d.on?'ON':'OFF'),d.on);
  }).catch(function(){toast('Network error',false);});}
function loadDebugLog(){
  // GET is always answered (route is registered unconditionally) — "supported":false
  // means this firmware build wasn't compiled with DEBUG_SERIAL_TO_SD, so hide the row
  // entirely rather than show a checkbox that can't do anything.
  return fetch('/debug_log').then(function(r){return r.json();}).then(function(d){
    var row=document.getElementById('row_dbg'),t=document.getElementById('tog_dbg');
    if(!d.supported){ if(row) row.style.display='none'; return; }
    if(row) row.style.display='';
    if(t){ t.checked=!!d.en; t.disabled=!!d.forced; }
    var desc=document.getElementById('dbg_desc');
    if(desc) desc.textContent = d.forced
      ? 'Serial output mirrored to debug_*.txt on SD. Forced ON by DEBUG_SERIAL_TO_SD_FORCE_ON in config.h \u2014 the web toggle can\u2019t turn this off.'
      : 'Serial output mirrored to debug_*.txt on SD \u2014 for field debugging when a serial monitor isn\u2019t available.';
    var dl=document.getElementById('dbg_dl');
    if(dl){
      // The file is still being written; content lags live serial by up to ~1s.
      dl.innerHTML = d.file
        ? '<a class="ref" style="display:inline-block;margin-top:.35rem" href="/log/'+d.file+'" download>&#x2B07; '+d.file+'</a>'
        : '<span class="empty">No debug file yet \u2014 starts once GPS sets the session timestamp.</span>';
    }
  }).catch(function(){var row=document.getElementById('row_dbg'); if(row) row.style.display='none';});}
function setDebugLog(val){
  fetch('/debug_log?en='+(val?'1':'0'),{method:'POST'}).then(function(r){return r.json();}).then(function(d){
    var t=document.getElementById('tog_dbg');
    if(d.forced){ if(t){t.checked=true;t.disabled=true;} toast('Forced ON by config.h \u2014 web toggle ignored',false); return; }
    toast('Debug log '+(d.en?'ON':'OFF'),d.en);
    loadDebugLog();
  }).catch(function(){toast('Network error',false);});}
// ── CAN sniffer ──────────────────────────────────────────────────────────────
// Lives with the log channels because that is what it is: another capture
// stream competing for the same SD write budget.
function setSniff(on){
  // When ENABLING, offer to keep the normal log channels running alongside the
  // sniffer (force=1). Default answer is safe (exclusive): Cancel = channels off.
  // OK = concurrent, accepting that raw-frame + typed logging together can exceed
  // the SD card's sustained write rate and drop frames under load.
  var force=0;
  if(on){
    force=confirm('Enable CAN sniffer WITH GPS/IMU/CAN logging still running?\n\n'+
      'OK  = log everything together (for finding vehicle-dynamics channels).\n'+
      '        Heavy bus traffic may exceed SD write speed and drop frames —\n'+
      '        watch the sniffer drops/overflow counters.\n\n'+
      'Cancel = sniff only, other log channels turned off (safer).')?1:0;
  }
  fetch('/can/sniff?on='+(on?1:0)+'&force='+force,{method:'POST'}).then(function(r){return r.json();}).then(function(d){
    window._sniffOn=d.sniffer; document.getElementById('tog_sniff').checked=d.sniffer;
    toast('CAN sniffer '+(d.sniffer?'ON':'OFF')+(d.sniffer&&force?' + logging':''),d.sniffer);
    // In exclusive mode the server forced the log channels off — resync those
    // switches so they show what is actually running.
    if(d.sniffer){loadSniff();loadLogConfig();}
    else{document.getElementById('snifftbl').innerHTML='';document.getElementById('sniffmeta').textContent='';}
  }).catch(function(){toast('Sniffer toggle failed',false);});}
function loadSniffState(){
  return fetch('/can/snapshot').then(function(r){return r.json();}).then(function(d){
    window._sniffOn=d.sniffer; document.getElementById('tog_sniff').checked=d.sniffer;
    if(d.sniffer)renderSniff(d);
  }).catch(function(){});}
function loadSniff(){
  return fetch('/can/snapshot').then(function(r){return r.json();}).then(renderSniff).catch(function(){});}
function hx2(v){return ('0'+v.toString(16).toUpperCase()).slice(-2);}
function renderSniff(d){
  var meta=document.getElementById('sniffmeta');
  var tbl=document.getElementById('snifftbl');
  if(!d.sniffer){meta.textContent='';tbl.innerHTML='';return;}
  meta.textContent=d.ids.length+' IDs'+(d.overflow?(' (+'+d.overflow+' over cap)'):'')+
    ' · raw drops: '+d.drops;
  if(!d.ids.length){tbl.innerHTML='<span class="empty">Listening&hellip; no frames yet</span>';return;}
  var rows=d.ids.map(function(x){
    var bytes=x.data.map(hx2).slice(0,x.dlc).join(' ');
    return '<div class="row"><div><span class="fn">0x'+('00'+x.id.toString(16).toUpperCase()).slice(-3)+'</span>'+
      '<span class="sz"> &mdash; '+bytes+'</span></div>'+
      '<div class="btns"><span class="sz">'+x.count+' · '+x.age+'ms</span></div></div>';}).join('');
  tbl.innerHTML=rows;}
// The files open for writing right now, with sizes that grow as they record —
// the direct confirmation that the channel switches above are actually producing
// data. Served from the default /files view, which is a pure-RAM read of the
// open-file table with no SD-bus access, so polling it is free.
function loadCurFiles(){
  var el=document.getElementById('curfiles');
  if(!el)return;
  return fetch('/files').then(function(r){return r.json();}).then(function(f){
    if(f.sd_ready===false){el.innerHTML='<span class="empty">SD not ready</span>';return;}
    if(!f.files||!f.files.length){
      el.innerHTML='<span class="empty">'+(window._paused?'Paused \u2014 nothing open'
                                                        :'No channel is writing')+'</span>';
      return;}
    el.innerHTML=f.files.map(function(x){
      return '<div class="row"><div><span class="fn">'+esc(x.name)+'</span>'+
        '<span class="sz"> &mdash; '+(x.size!=null?fsize(x.size):'')+
        '<span class="rec"> &#x25CF; recording</span></span></div></div>';}).join('');
  }).catch(function(){el.innerHTML='<span class="empty">unavailable</span>';});}
window.afterPauseChange=function(){chain([loadLogConfig,loadCurFiles]);};
// Startup runs one request at a time, then hands over to a single poll loop.
// Status and the open-file list refresh every cycle; SD capacity every fourth,
// since free space moves slowly; the sniffer table only while it is running.
chain([loadStat,loadSD,loadPause,loadLogConfig,loadCurFiles,loadDebugLog,loadLcd,loadSniffState])
  .then(function(){
    pollLoop(function(tick){
      var fns=[loadStat,loadCurFiles];
      if(tick%4===0)fns.push(loadSD);
      if(window._sniffOn)fns.push(loadSniff);
      return fns;
    },3000);
  });
</script></body></html>)html";

// ── Page: setup ───────────────────────────────────────────────────────────────
static const char PAGE_SETUP[] PROGMEM = R"html(<!DOCTYPE html>
<html lang="en"><head>
<meta charset="UTF-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<meta name="color-scheme" content="light">
<title>RCX RTK Datalogger &middot; Setup</title>
<link rel="stylesheet" href="/app.css">
</head><body>
<h1>&#x2699; Setup</h1>
<nav><a href="/">Dashboard</a><a class="on" href="/setup">Setup</a><a href="/logs">Log Files</a></nav>

<h2>Device Name</h2>
<div class="tdesc" style="margin-bottom:.4rem">Also the name of this unit&#39;s configuration WiFi network, which picks up the new name after a restart.</div>
<div style="display:flex;align-items:center;gap:.4rem;margin:.4rem 0;flex-wrap:wrap">
  <input id="dname" class="tin" type="text" placeholder="device name" maxlength="32" style="flex:1;min-width:160px">
  <button class="ref" onclick="devRename()">Rename</button>
</div>
<div id="devmsg" class="empty" style="margin-bottom:.3rem"></div>

<h2>WiFi Networks</h2>
<div class="tdesc" style="margin-bottom:.4rem">Tried top to bottom &mdash; the first one in range wins. Use &#x25B2;/&#x25BC; to set priority. Editing this list never drops the connection you&#39;re using right now; changes apply on the next reconnect.</div>
<div class="files" id="wifilist"><span class="empty">Loading&hellip;</span></div>
<div style="display:flex;align-items:center;gap:.4rem;margin:.4rem 0;flex-wrap:wrap">
  <input id="wssid" class="tin" type="text" placeholder="SSID" maxlength="32" style="flex:1;min-width:120px">
  <input id="wpass" class="tin" type="text" placeholder="password (blank = open)" maxlength="64" autocomplete="off" style="flex:1;min-width:120px">
  <button class="ref" onclick="wifiAdd()">Add</button>
</div>
<div id="wifimsg" class="empty" style="margin-bottom:.3rem"></div>

<h2>NTRIP Casters</h2>
<div class="tdesc" style="margin-bottom:.4rem">Sources of RTK corrections. Built-in entries can be switched off but not deleted. Changing anything here restarts caster selection, which briefly interrupts corrections.</div>
<div class="files" id="castlist"><span class="empty">Loading&hellip;</span></div>
<div style="display:flex;align-items:center;gap:.4rem;margin:.4rem 0;flex-wrap:wrap">
  <input id="chost" class="tin" type="text" placeholder="host" maxlength="47" style="flex:2;min-width:130px">
  <input id="cport" class="tin" type="text" placeholder="2101" maxlength="7" style="width:70px">
  <input id="cmount" class="tin" type="text" placeholder="mountpoint (blank = nearest)" maxlength="63" style="flex:2;min-width:130px">
</div>
<div style="display:flex;align-items:center;gap:.4rem;margin:.4rem 0;flex-wrap:wrap">
  <input id="cuser" class="tin" type="text" placeholder="username" maxlength="63" autocomplete="off" style="flex:1;min-width:120px">
  <input id="cpass" class="tin" type="text" placeholder="password" maxlength="39" autocomplete="off" style="flex:1;min-width:120px">
  <button class="ref" onclick="castAdd()">Add</button>
</div>
<div id="castmsg" class="empty" style="margin-bottom:.3rem"></div>
<div style="display:flex;align-items:center;gap:.5rem;margin:.2rem 0 .5rem;flex-wrap:wrap">
  <button class="ref" onclick="castReset()">Reset caster timeouts &amp; re-search</button>
  <span class="tdesc">Clears the per-caster search rate limits, cooldowns and retry backoff, then reselects immediately. Use after a late WiFi connection or joining the wrong network, instead of waiting the timers out.</span>
</div>

<h2>CAN Databases</h2>
<div class="tdesc" style="margin-bottom:.4rem">Upload a .dbc file to describe a vehicle the built-in profiles do not cover. Signal names become channel names, so name them as you want them logged &mdash; SoloStorm shows only the first 11 characters. Scale and offset must produce the logger&#39;s native units: km/h, &deg;C, bar, %, degrees. Built-in Porsche profiles are unaffected by anything stored here.</div>
<div class="files" id="dbclist"><span class="empty">Loading&hellip;</span></div>
<div style="display:flex;align-items:center;gap:.4rem;margin:.4rem 0;flex-wrap:wrap">
  <input id="dbcfile" class="tin" type="file" accept=".dbc" style="flex:2;min-width:150px">
  <button class="ref" onclick="dbcUpload()">Upload</button>
  <button class="ref" onclick="dbcSelect('')">Clear selection</button>
</div>
<div id="dbcmsg" class="empty" style="margin-bottom:.3rem"></div>
<pre id="dbcaudit" class="tdesc" style="display:none;white-space:pre-wrap;max-height:16rem;overflow:auto;margin:.3rem 0"></pre>

<h2>GNSS Tuning</h2>
<div id="gnsscfg">
  <div class="trow"><div><div class="tlabel">Elevation mask</div><div class="tdesc">Exclude sats below this angle &mdash; higher rejects low-elevation reflections/occlusions, but thins geometry. Degrees, 0&ndash;90.</div></div>
    <input id="ele" class="tin" type="number" min="0" max="90" step="1"></div>
  <div class="trow"><div><div class="tlabel">C/N0 mask</div><div class="tdesc">Exclude signals weaker than this &mdash; higher rejects reflections, but can drop weak-but-real sats. dB-Hz, 0&ndash;50.</div></div>
    <input id="cnr" class="tin" type="number" min="0" max="50" step="1"></div>
  <div class="trow"><div><div class="tlabel">PPP fallback</div><div class="tdesc">Used only when RTK is unavailable &mdash; RTK Fixed always wins. Needs ~10&ndash;20 min of clear sky to converge, and any signal loss restarts that. Decimetre-class at best.</div></div>
    <select id="ppp" class="tin">
      <option value="0">Off</option>
      <option value="1">B2b (BeiDou)</option>
      <option value="2">E6 HAS (Galileo)</option>
    </select></div>
  <div class="trow"><div class="tdesc">Applies immediately &amp; saves across reboots. Watch the Sats count &mdash; if it drops or fixes get flaky, ease the masks back. Changing PPP may briefly drop the fix &mdash; do it in the paddock, not on a run.</div>
    <button class="ref" onclick="applyGnss()">Apply</button></div>
</div>

<h2>IMU Calibration</h2>
<div class="tdesc" style="margin-bottom:.4rem">Corrects gyro/accel zero-offset drift. Set the unit still and resting flat, then calibrate &mdash; takes about 2 seconds.</div>
<div style="display:flex;align-items:center;gap:.4rem;margin:.4rem 0;flex-wrap:wrap">
  <button class="ref" id="btn_imucal" onclick="imuCal('start')">Calibrate</button>
  <button class="ref" onclick="imuCal('clear')">Clear</button>
</div>
<div id="imucal_msg" class="empty" style="margin-bottom:.3rem"></div>
<div id="toast"></div>
<script src="/app.js"></script>
<script>
// ── WiFi network list ────────────────────────────────────────────────────────
// Passwords are write-only: the device never sends a stored passphrase back, so
// there is nothing here to render for them.
function loadWifi(){
  return fetch('/wifi').then(r=>r.json()).then(d=>{
    var el=document.getElementById('wifilist');
    var ap=document.getElementById('wifimsg');
    ap.textContent='Config AP: '+(d.ap||'?')+'  \u00B7  http://'+(d.apip||'?');
    var nf=document.getElementById('dname');
    if(nf){
      // The placeholder always carries the live name, so the field still shows
      // what the device is called after it has been cleared.
      nf.placeholder=d.ap||'device name';
      // Seed the value ONCE, on first load. Re-seeding on every refresh would
      // undo both the clear that confirms a rename and anything being typed.
      if(!dnameSeeded&&nf!==document.activeElement){nf.value=d.ap||'';dnameSeeded=true;}
    }
    if(!d.nets||!d.nets.length){
      el.innerHTML='<span class="empty">None stored &mdash; the unit is running AP-only. Add a network above.</span>';
      return;
    }
    var h='';
    for(var i=0;i<d.nets.length;i++){
      var n=d.nets[i].ssid, on=d.nets[i].on;
      var live=(n===d.active)?' <span class="ok">&#x25CF; connected</span>':'';
      var off=on?'':' <span class="empty">(disabled)</span>';
      h+='<div class="trow"><div><div class="tlabel"'+(on?'':' style="opacity:.55"')+'>'
       + (i+1)+'. '+esc(n)+live+off+'</div></div><div>'
       + '<button class="ref" title="'+(on?'Disable':'Enable')+'" onclick="wifiEn('+i+','+(on?0:1)+')">'
       + (on?'&#x23FB;':'&#x25CB;')+'</button> '
       + '<button class="ref" title="Higher priority" onclick="wifiMove('+i+',-1)">&#x25B2;</button> '
       + '<button class="ref" title="Lower priority" onclick="wifiMove('+i+',1)">&#x25BC;</button> '
       + '<button class="ref" title="Remove" onclick="wifiDel('+i+',\''+esc(n).replace(/'/g,"\\'")+'\')">&#x1F5D1;</button>'
       + '</div></div>';
    }
    el.innerHTML=h;
  }).catch(()=>{document.getElementById('wifilist').innerHTML='<span class="empty">unavailable</span>';});
}
// Seeded once per page load; see loadWifi().
var dnameSeeded=false;
function devRename(){
  var f=document.getElementById('dname');
  var n=f.value.trim();
  if(!n){document.getElementById('devmsg').textContent='Enter a name.';return;}
  fetch('/device/name',{method:'POST',headers:{'Content-Type':'application/x-www-form-urlencoded'},
    body:'name='+encodeURIComponent(n)})
    .then(r=>r.text()).then(t=>{
      document.getElementById('devmsg').textContent=t;
      // Clear on success only. An empty box next to the new name in the status
      // line is the confirmation that the rename landed; leaving the old text
      // sitting there reads as though nothing happened. On a rejection the text
      // stays put so it can be corrected rather than retyped.
      if(t.indexOf('Renamed')===0)f.value='';
      loadWifi();});
}
function wifiAdd(){
  var s=document.getElementById('wssid').value.trim();
  if(!s){document.getElementById('wifimsg').textContent='Enter an SSID.';return;}
  var b='ssid='+encodeURIComponent(s)+'&pass='+encodeURIComponent(document.getElementById('wpass').value);
  fetch('/wifi/add',{method:'POST',headers:{'Content-Type':'application/x-www-form-urlencoded'},body:b})
    .then(r=>r.text()).then(t=>{
      document.getElementById('wssid').value='';document.getElementById('wpass').value='';
      document.getElementById('wifimsg').textContent=t;loadWifi();});
}
function wifiDel(i,name){
  if(!confirm('Remove "'+name+'"?'))return;
  fetch('/wifi/remove',{method:'POST',headers:{'Content-Type':'application/x-www-form-urlencoded'},body:'i='+i})
    .then(r=>r.text()).then(t=>{document.getElementById('wifimsg').textContent=t;loadWifi();});
}
function wifiEn(i,on){
  fetch('/wifi/enable',{method:'POST',headers:{'Content-Type':'application/x-www-form-urlencoded'},
    body:'i='+i+'&on='+on}).then(()=>loadWifi());
}
function wifiMove(i,d){
  fetch('/wifi/move',{method:'POST',headers:{'Content-Type':'application/x-www-form-urlencoded'},body:'i='+i+'&d='+d})
    .then(()=>loadWifi());
}
// ── NTRIP caster list ────────────────────────────────────────────────────────
function castReset(){
  const m=document.getElementById('castmsg');
  m.textContent='Resetting\u2026';
  fetch('/casters/reset',{method:'POST'})
    .then(r=>r.text()).then(t=>{m.textContent=t; setTimeout(loadCasters,1500);})
    .catch(()=>{m.textContent='Reset failed \u2014 device unreachable.';});
}
function loadCasters(){
  return fetch('/casters').then(r=>r.json()).then(d=>{
    var el=document.getElementById('castlist');
    if(!d.list||!d.list.length){el.innerHTML='<span class="empty">None configured.</span>';return;}
    var h='';
    for(var i=0;i<d.list.length;i++){
      var c=d.list[i];
      var where=esc(c.host)+':'+esc(c.port)+(c.mount?' /'+esc(c.mount):' <span class="empty">(nearest)</span>');
      var tags=(c.active?' <span class="ok">&#x25CF; in use</span>':'')
             + (c.def?' <span class="empty">built-in</span>':'')
             + (c.on?'':' <span class="empty">(disabled)</span>');
      h+='<div class="trow"><div><div class="tlabel"'+(c.on?'':' style="opacity:.55"')+'>'
       + where+tags+'</div></div><div>'
       + '<button class="ref" title="'+(c.on?'Disable':'Enable')+'" onclick="castEn('+i+','+(c.on?0:1)+')">'
       + (c.on?'&#x23FB;':'&#x25CB;')+'</button>'
       + (c.def?'':' <button class="ref" title="Remove" onclick="castDel('+i+',\''+esc(c.host).replace(/'/g,"\\'")+'\')">&#x1F5D1;</button>')
       + '</div></div>';
    }
    el.innerHTML=h;
  }).catch(()=>{document.getElementById('castlist').innerHTML='<span class="empty">unavailable</span>';});
}
function castEn(i,on){
  fetch('/casters/enable',{method:'POST',headers:{'Content-Type':'application/x-www-form-urlencoded'},
    body:'i='+i+'&on='+on}).then(()=>loadCasters());
}
function castDel(i,name){
  if(!confirm('Remove caster "'+name+'"?'))return;
  fetch('/casters/remove',{method:'POST',headers:{'Content-Type':'application/x-www-form-urlencoded'},
    body:'i='+i}).then(r=>r.text()).then(t=>{document.getElementById('castmsg').textContent=t;loadCasters();});
}
function castAdd(){
  var host=document.getElementById('chost').value.trim();
  if(!host){document.getElementById('castmsg').textContent='Enter a host.';return;}
  var b='host='+encodeURIComponent(host)
      + '&port='+encodeURIComponent(document.getElementById('cport').value.trim()||'2101')
      + '&mount='+encodeURIComponent(document.getElementById('cmount').value.trim())
      + '&user='+encodeURIComponent(document.getElementById('cuser').value)
      + '&pass='+encodeURIComponent(document.getElementById('cpass').value);
  fetch('/casters/add',{method:'POST',headers:{'Content-Type':'application/x-www-form-urlencoded'},body:b})
    .then(r=>r.text()).then(t=>{
      ['chost','cport','cmount','cuser','cpass'].forEach(function(k){document.getElementById(k).value='';});
      document.getElementById('castmsg').textContent=t;loadCasters();});
}
// ── CAN databases ────────────────────────────────────────────────────────────
function loadDbc(){
  return fetch('/dbc').then(r=>r.json()).then(d=>{
    var L=document.getElementById('dbclist');
    if(!d.files||!d.files.length){L.innerHTML='<span class="empty">No .dbc files on the card.</span>';}
    else{
      L.innerHTML=d.files.map(function(f){
        var sel=(f.name===d.active);
        return '<div class="frow"><div><b>'+f.name+'</b>'
          +(sel?' <span class="tag">selected</span>':'')
          +'<div class="tdesc">'+f.size+' bytes</div></div><div>'
          +'<button class="ref" onclick="dbcSelect(\''+f.name+'\')">'
          +(sel?'Reselect':'Select')+'</button> '
          +'<button class="ref" onclick="dbcDelete(\''+f.name+'\')">Delete</button>'
          +'</div></div>';}).join('');
    }
    if(d.status)document.getElementById('dbcmsg').textContent=d.status;
    var A=document.getElementById('dbcaudit');
    if(d.audit){A.textContent=d.audit;A.style.display='block';}
    else{A.style.display='none';}
    // The scan runs on its own task, so the listing is re-read once it lands.
    if(d.scanning)setTimeout(loadDbc,600);
  }).catch(function(){});
}
function dbcUpload(){
  var f=document.getElementById('dbcfile').files[0];
  if(!f){document.getElementById('dbcmsg').textContent='Choose a .dbc file first.';return;}
  var fd=new FormData();fd.append('file',f,f.name);
  document.getElementById('dbcmsg').textContent='Uploading '+f.name+'\u2026';
  fetch('/dbc/upload',{method:'POST',body:fd})
    .then(r=>r.text()).then(t=>{document.getElementById('dbcmsg').textContent=t;
      document.getElementById('dbcfile').value='';setTimeout(loadDbc,600);});
}
function dbcSelect(n){
  fetch('/dbc/select',{method:'POST',headers:{'Content-Type':'application/x-www-form-urlencoded'},
    body:'name='+encodeURIComponent(n)})
    .then(r=>r.text()).then(t=>{document.getElementById('dbcmsg').textContent=t;loadDbc();});
}
function dbcDelete(n){
  if(!confirm('Delete '+n+'?'))return;
  fetch('/dbc/delete',{method:'POST',headers:{'Content-Type':'application/x-www-form-urlencoded'},
    body:'name='+encodeURIComponent(n)})
    .then(r=>r.text()).then(t=>{document.getElementById('dbcmsg').textContent=t;setTimeout(loadDbc,600);});
}
// ── GNSS tuning ──────────────────────────────────────────────────────────────
function loadGnssCfg(){return fetch('/gnss_config').then(function(r){return r.json();}).then(function(d){
  var e=document.getElementById('ele'),c=document.getElementById('cnr'),p=document.getElementById('ppp');
  if(e)e.value=d.ele; if(c)c.value=d.cnr; if(p)p.value=d.ppp;}).catch(function(){});}
function applyGnss(){
  var e=document.getElementById('ele').value,c=document.getElementById('cnr').value,
      p=document.getElementById('ppp').value;
  fetch('/gnss_config?ele='+encodeURIComponent(e)+'&cnr='+encodeURIComponent(c)+'&ppp='+encodeURIComponent(p),{method:'POST'})
    .then(function(r){return r.json();}).then(function(d){
      var pn=['off','B2b','E6 HAS'][+p]||'off';
      toast(d.ok?('Applied: '+e+'\u00B0 / '+c+' dB-Hz \u00B7 PPP '+pn):'Value out of range',!!d.ok);})
    .catch(function(){toast('Apply failed',false);});}
// ── IMU calibration ──────────────────────────────────────────────────────────
function loadImuCal(){
  return fetch('/imu/cal').then(function(r){return r.json();}).then(renderImuCal).catch(function(){});}
function renderImuCal(d){
  var msg=document.getElementById('imucal_msg'), btn=document.getElementById('btn_imucal');
  if(!msg) return;
  if(d.state==='running'){
    msg.textContent='Calibrating \u2014 keep the unit still and level...';
    if(btn) btn.disabled=true;
    setTimeout(loadImuCal,500);
    return;
  }
  if(btn) btn.disabled=false;
  msg.textContent = d.state==='failed'
    ? 'Calibration failed \u2014 motion detected. Keep it still & flat, then try again.'
    : (d.calibrated ? 'Calibrated' : 'Not calibrated (using factory zero)');
}
function imuCal(action){
  fetch('/imu/cal?action='+action,{method:'POST'}).then(function(r){return r.json();}).then(function(d){
    toast(action==='start'?'IMU calibration started':'Calibration cleared',true);
    renderImuCal(d);
  }).catch(function(){toast('Network error',false);});}
chain([loadWifi,loadCasters,loadDbc,loadGnssCfg,loadImuCal]);
</script></body></html>)html";

// ── Page: log files ───────────────────────────────────────────────────────────
static const char PAGE_LOGS[] PROGMEM = R"html(<!DOCTYPE html>
<html lang="en"><head>
<meta charset="UTF-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<meta name="color-scheme" content="light">
<title>RCX RTK Datalogger &middot; Log Files</title>
<link rel="stylesheet" href="/app.css">
</head><body>
<h1>&#x1F4C1; Log Files</h1>
<nav><a href="/">Dashboard</a><a href="/setup">Setup</a><a class="on" href="/logs">Log Files</a></nav>
<div class="pausebar" id="pausebar"><span class="empty">Loading&hellip;</span></div>
<div class="sdbar" id="sdbar"><span class="empty">Loading&hellip;</span></div>
<div style="display:flex;align-items:center;gap:.4rem;margin-bottom:.5rem;flex-wrap:wrap">
  <button class="ref" id="btn_export_all" onclick="exportAll()">&#x1F4E6; Download all (.tar)</button>
  <button class="ref" onclick="deleteOld()">&#x1F5D1; Delete all old files</button>
  <span id="delstatus" style="font-size:.72rem;color:#5a8a5a"></span>
</div>
<h2>View</h2>
<div class="row" style="gap:.5rem">
  <div class="btns">
    <button class="ref" id="mode_active" onclick="setMode('active')">&#x25CF; Recording now</button>
    <button class="ref" id="mode_recent" onclick="setMode('recent')">&#x1F552; Last 12</button>
    <button class="ref" id="mode_all" onclick="setMode('all')">&#x1F4C2; All files</button>
  </div>
</div>
<div class="tdesc" id="modehint" style="margin:.3rem 0"></div>
<div class="pager" id="pager_top"></div>
<div class="files" id="files"><span class="empty">Loading&hellip;</span></div>
<div class="pager" id="pager_bottom"></div>
<h2>Selection</h2>
<div class="row" style="gap:.6rem">
  <label style="display:flex;align-items:center;gap:.45rem;font-size:.82rem;font-weight:700">
    <input type="checkbox" class="ck" id="ckall" onchange="selectAllOnPage(this.checked)"> Select all on this page</label>
  <div class="btns">
    <span class="sz" id="selcount">0 selected</span>
    <button class="ref" id="btn_dlsel" onclick="downloadSelected()" disabled>&#x2B07; Download selected</button>
    <button class="del" id="btn_delsel" onclick="deleteSelected()" disabled>&#x1F5D1; Delete selected</button>
  </div>
</div>
<div class="tdesc" style="margin:.3rem 0">Pause logging to download or delete. A transfer competes with the recording writes for the SD card, and a file open for writing can&#39;t be removed at all. Downloads build a single .tar on the device first &mdash; up to 64 files per archive. Each session heading below also carries its own archive and delete buttons, named for the local time of that run.</div>
<div id="toast"></div>
<script src="/app.js"></script>
<script>
// Page size is bounded on purpose: the server builds each listing response in
// internal heap, the scarcest RAM on this board, so the whole card is walked a
// page at a time rather than serialised into one huge response.
var PER=50;
var pgOffset=0, pgTotal=0, pgTrunc=false;
// The view opens on 'active' — the currently-recording files only. That path is
// a handful of entries straight out of RAM, so the page costs nothing to open
// or to poll. 'recent' and 'all' are deliberate, user-initiated expansions;
// 'all' additionally pages, so no single response ever carries the whole card.
var mode='active';
function setMode(m){
  mode=m; pgOffset=0;
  var hint=document.getElementById('modehint');
  hint.textContent = m==='active'
    ? 'Files being written right now. Sizes update live.'
    : m==='recent'
    ? 'The 12 newest files.'
    : 'Every file the device is tracking, '+PER+' at a time.';
  ['active','recent','all'].forEach(function(k){
    var b=document.getElementById('mode_'+k);
    if(b)b.style.borderColor=(k===m)?'var(--accent)':'var(--border)';
  });
  loadFiles();
}
// Selected filenames survive paging and mode changes — picking a set that spans
// pages is the normal case after a long day, so checkbox state cannot live in
// the DOM.
var selected={};
// Session key -> the non-recording filenames in that session. Populated by the
// listing render; the per-session buttons read it rather than the DOM.
var groupFiles={};
function selCount(){return Object.keys(selected).length;}
function updateSelUi(){
  var n=selCount();
  document.getElementById('selcount').textContent=n+' selected';
  document.getElementById('btn_delsel').disabled=(n===0);
  var dl=document.getElementById('btn_dlsel');
  dl.disabled=(n===0)||!window._paused||dlBusy;
  dl.title=window._paused?'':'Pause logging to download';
  var all=document.getElementById('btn_export_all');
  if(all){ all.disabled=!window._paused||dlBusy; all.title=window._paused?'':'Pause logging to download'; }
  // Session buttons are re-created by every listing render, so they are gated
  // here rather than at render time — this runs after each render and on every
  // pause change, which keeps them in step with the two toolbar buttons above.
  var gdl=document.querySelectorAll('button.gdl');
  for(var i=0;i<gdl.length;i++){
    gdl[i].disabled=!window._paused||dlBusy;
    gdl[i].title=window._paused?'':'Pause logging to download';
  }
  var gdel=document.querySelectorAll('button.gdel');
  for(var j=0;j<gdel.length;j++) gdel[j].disabled=dlBusy;
}
function toggleSel(name,on){
  if(on)selected[name]=1; else delete selected[name];
  updateSelUi();
}
function selectAllOnPage(on){
  var boxes=document.querySelectorAll('input.ck[data-fn]');
  for(var i=0;i<boxes.length;i++){
    if(boxes[i].disabled)continue;      // a recording file is never selectable
    boxes[i].checked=on;
    toggleSel(boxes[i].getAttribute('data-fn'),on);
  }
}
function renderPager(){
  if(mode!=='all'){
    document.getElementById('pager_top').innerHTML='';
    document.getElementById('pager_bottom').innerHTML='';
    return;
  }
  var from=pgTotal?(pgOffset+1):0;
  var to=Math.min(pgOffset+PER,pgTotal);
  var h='<button class="ref" onclick="gotoPage(0)"'+(pgOffset<=0?' disabled':'')+'>&#x00AB; Newest</button>'
      + '<button class="ref" onclick="gotoPage('+(pgOffset-PER)+')"'+(pgOffset<=0?' disabled':'')+'>&#x2039; Prev</button>'
      + '<span class="pos">'+from+'&ndash;'+to+' of '+pgTotal+(pgTrunc?'+':'')+'</span>'
      + '<button class="ref" onclick="gotoPage('+(pgOffset+PER)+')"'+((pgOffset+PER)>=pgTotal?' disabled':'')+'>Next &#x203A;</button>';
  document.getElementById('pager_top').innerHTML=h;
  document.getElementById('pager_bottom').innerHTML=h;
}
function gotoPage(off){
  if(off<0)off=0;
  if(off>=pgTotal)off=Math.max(0,pgTotal-PER);
  pgOffset=off; loadFiles();
}
// Filenames are <type>_YYYYMMDD_HHMMSS.csv, and that timestamp is always UTC —
// the device sets its clock from GPS, never local time. Every channel opened
// in one recording session shares the exact same stamp (they're all opened
// within the same instant), so grouping rows by it recovers "session" as a
// concept the server never has to compute, and converting the heading to the
// browser's own timezone is what actually answers "when was this" without
// doing UTC math by hand. The small UTC line stays alongside it because
// that's the form serial logs and the filenames themselves use — the thing
// you'd cross-reference against.
// The _UTC marker is optional here on purpose: files written before the logger
// started stamping it are still on cards and still belong to a session.
var SESSION_RE=/^[a-z]+_(\d{8})_(\d{6})(?:_UTC)?\.csv$/;
function sessionKey(name){ var m=SESSION_RE.exec(name); return m?(m[1]+'_'+m[2]):null; }
function sessionHeading(key){
  var y=+key.slice(0,4),mo=+key.slice(4,6),d=+key.slice(6,8),
      h=+key.slice(9,11),mi=+key.slice(11,13),s=+key.slice(13,15);
  var dt=new Date(Date.UTC(y,mo-1,d,h,mi,s));
  return {
    local: isNaN(dt) ? key : dt.toLocaleString(undefined,{dateStyle:'medium',timeStyle:'medium'}),
    utc: y+'-'+key.slice(4,6)+'-'+key.slice(6,8)+' '+key.slice(9,11)+':'+key.slice(11,13)+':'+key.slice(13,15)+' UTC'
  };
}
function loadFiles(_retry){
  var q = mode==='all'    ? '?all=1&offset='+pgOffset+'&limit='+PER
        : mode==='recent' ? '?recent=1'
                          : '';
  return fetch('/files'+q).then(function(r){return r.json();}).then(function(f){
    var el=document.getElementById('files');
    if(f.sd_ready===false){
      el.innerHTML='<span class="empty">SD not ready &mdash; waiting for card&hellip;</span>'; return;}
    pgTotal=f.total||0; pgTrunc=!!f.truncated;
    // The boot listing scan runs on the logger task; if it hasn't landed yet the
    // server says so. Re-fetch once after a short delay rather than showing an
    // empty card as though there were no files.
    if(f.refreshing && !_retry){ setTimeout(function(){loadFiles(true);},1100); }
    var html='';
    if(!f.files||!f.files.length){
      html='<span class="empty">'+(f.refreshing?'Scanning card&hellip;'
           :(mode==='active'?'Nothing recording right now':'No log files yet'))+'</span>';
    }else{
      function row(x){
        var nm=esc(x.name);
        var live=x.active?'<span class="rec"> &#x25CF; recording</span>':'';
        var sz=(x.size!=null?fsize(x.size):'')+live;
        // A file open for writing cannot be deleted (removing it corrupts FatFs
        // and resets the device), so it is offered for download but not select.
        var box=x.active
          ? '<input type="checkbox" class="ck" disabled title="Recording — pause logging first">'
          : '<input type="checkbox" class="ck" data-fn="'+nm+'"'+(selected[x.name]?' checked':'')+
            ' onchange="toggleSel(this.getAttribute(\'data-fn\'),this.checked)">';
        var dlBtn = window._paused
          ? '<a class="dl" href="/log/'+encodeURIComponent(x.name)+'">&#x2B07; Download</a>'
          : '<button class="ref" disabled title="Pause logging to download">&#x2B07; Download</button>';
        return '<div class="row"><div style="display:flex;align-items:center;gap:.55rem">'+box+
          '<div><span class="fn">'+nm+'</span><span class="sz"> &mdash; '+sz+'</span></div></div>'+
          '<div class="btns">'+dlBtn+
          '<button class="del" onclick="delFile(\''+nm.replace(/'/g,"\\'")+'\')">&#x1F5D1;</button>'+
          '</div></div>';
      }
      // Group consecutive same-session files. The server already returns rows
      // newest-first and every file in a session shares an identical stamp, so
      // a session's files are always adjacent — no re-sort needed here. A run
      // of files whose name doesn't parse (shouldn't happen, but harmless if it
      // does) just renders with no heading rather than being dropped.
      var groups=[];
      f.files.forEach(function(x){
        var key=sessionKey(x.name);
        var last=groups[groups.length-1];
        if(last && last.key===key) last.items.push(x);
        else groups.push({key:key, items:[x]});
      });
      // The session -> filenames map the per-session buttons act on. Rebuilt
      // from scratch on every listing render so it can never name a file that
      // has since been deleted or fallen off the page being shown.
      groupFiles={};
      html=groups.map(function(g){
        var rows=g.items.map(row).join('');
        if(!g.key) return rows;
        var h=sessionHeading(g.key);
        var total=g.items.reduce(function(a,x){return a+(x.size||0);},0);
        // A file still recording is excluded from both session actions: the
        // archive builder skips open files anyway, and deleting one corrupts
        // FatFs. Pausing closes them, at which point they rejoin the group.
        groupFiles[g.key]=g.items.filter(function(x){return !x.active;})
                                 .map(function(x){return x.name;});
        return '<div class="grouphead"><span>'+h.local+'</span>'+
          '<span class="gmeta">'+h.utc+' &middot; '+g.items.length+' file'+(g.items.length===1?'':'s')+
          (total?(' &middot; '+fsize(total)):'')+'</span>'+
          '<span class="gbtns">'+
          '<button class="ref gdl" onclick="downloadGroup(\''+g.key+'\')">&#x1F4E6; Session .tar</button>'+
          '<button class="ref gdel" onclick="deleteGroup(\''+g.key+'\')">&#x1F5D1; Delete session</button>'+
          '</span></div>'+rows;
      }).join('');
      if(pgTrunc){html+='<div class="empty" style="margin:.3rem 0">The card holds more files than the listing can track &mdash; the oldest are not shown. Delete some to see them.</div>';}
    }
    el.innerHTML=html;
    document.getElementById('ckall').checked=false;
    renderPager(); updateSelUi();
  }).catch(function(){document.getElementById('files').innerHTML='<span class="empty">SD unavailable</span>';});}
function delFile(name){
  if(!confirm('Delete '+name+'?')) return;
  fetch('/log/'+encodeURIComponent(name),{method:'DELETE'}).then(function(r){
    if(r.ok){toast('Deleted '+name,true);delete selected[name];loadFiles();loadSD();return;}
    // Surface the server's actual reason (recording / busy / not found) rather
    // than a generic failure.
    return r.text().then(function(t){
      toast(t||('Delete failed ('+r.status+')'),false);
      if(r.status===404){delete selected[name];loadFiles();loadSD();}
    });
  }).catch(function(){toast('Delete failed',false);});}
// Batch delete goes out in chunks: each request holds the SD mutex for its whole
// run, so a small bounded batch keeps that hold short enough not to starve the
// logging writes, and gives the browser progress to show on a large selection.
var CHUNK=25;
// Both "Download all" and "Download selected" build ONE combined tar on the
// device (in sdLogTask, off the async_tcp task — see buildExport() in
// sd_log.cpp) and then trigger a single download of that archive. This
// replaced a client-side sequential-download loop that fetched each file
// individually: that approach had no way to know when one file's transfer
// actually FINISHED (a plain download has no completion signal JS can await),
// so multiple multi-megabyte files ended up overlapping in practice, which
// drove the device's internal heap low enough to fail unrelated requests —
// including the dashboard — while a batch was running. Building server-side
// means exactly one file ever comes down the wire, regardless of selection
// size. Pausing is required for the same reason a single-file download is:
// the final GET reads from the FatFs volume the active writer also holds.
var dlBusy=false;
function runExport(names,fileName){
  if(dlBusy)return;
  if(!window._paused){ toast('Pause logging before downloading',false); return; }
  dlBusy=true; updateSelUi();
  var ds=document.getElementById('delstatus');
  ds.textContent='Building archive\u2026'; toast('Building archive\u2026',true);
  function done(){ dlBusy=false; updateSelUi(); }
  var body = names ? ('names='+encodeURIComponent(names.join(','))) : '';
  if(fileName) body += (body?'&':'')+'file='+encodeURIComponent(fileName);
  fetch('/export/build',{method:'POST',headers:{'Content-Type':'application/x-www-form-urlencoded'},body:body})
    .then(function(r){
    if(!r.ok) throw 0;
    var tries=0,stall=0,lastCount=-1,lastSize=-1;
    var iv=setInterval(function(){
      fetch('/export/status').then(function(r){return r.json();}).then(function(s){
        if(s.state==='ready'){
          clearInterval(iv); done();
          var mb=(s.size/1048576).toFixed(2);
          ds.textContent='Archive: '+s.count+' files, '+mb+' MB';
          toast('Archive ready ('+s.count+' files)',true);
          window.location='/export.tar';
        }else if(s.state==='error'){
          clearInterval(iv); done(); ds.textContent='Export failed'; toast('Export failed',false);
        }else{
          var mb=(s.size/1048576).toFixed(1);
          ds.textContent='Building archive\u2026 '+(s.count||0)+' files, '+mb+' MB';
        }
        // Cap on elapsed time, but reset it whenever progress advances so a slow
        // build of a big card doesn't get killed while it's still working. Only a
        // genuine stall (no new files/bytes for ~90 s) trips the timeout.
        if(s.count!==lastCount||s.size!==lastSize){lastCount=s.count;lastSize=s.size;stall=0;}
        else if(++stall>90){clearInterval(iv); done(); ds.textContent='Export stalled \u2014 see serial'; toast('Export stalled',false);}
        if(++tries>3600){clearInterval(iv); done(); ds.textContent='Export timed out (60 min)'; toast('Export timed out',false);}
      }).catch(function(){clearInterval(iv); done(); toast('Export status failed',false);});
    },1000);
  }).catch(function(){ds.textContent=''; done(); toast('Export failed to start',false);});
}
function exportAll(){ runExport(null,null); }
function downloadSelected(){
  var names=Object.keys(selected);
  if(!names.length)return;
  runExport(names,null);
}
// ── Per-session actions ───────────────────────────────────────────────────────
// A session is the set of channel files opened at the same instant, which is
// what one recording run actually produced — so acting on the whole group is
// the common case after a run, without ticking every box by hand. Both act on
// groupFiles[], which the listing render fills with that session's closed files.

// Archive name for a session. The stamp inside a log filename is UTC (the clock
// comes from GPS), so it is converted to whatever timezone this browser is in
// before being formatted back into the same YYYYMMDD_HHMMSS shape — the archive
// is then named for the local time of the run it holds, which is how the run is
// remembered, while the files inside keep their UTC-marked names. The zone tag
// is what makes the two readable side by side: rcx_logs_..._EDT.tar holding
// gps_..._UTC.csv states the offset rather than leaving it to be worked out.
function localZoneTag(dt){
  // Intl gives the zone in the browser's own locale, so it can come back as an
  // abbreviation (EDT), a GMT offset (GMT-4) or a localised string. Reduce it to
  // letters and digits — the same set the device accepts in a filename — and
  // fall back rather than emitting an empty tag.
  try{
    var parts=new Intl.DateTimeFormat(undefined,{timeZoneName:'short'}).formatToParts(dt);
    for(var i=0;i<parts.length;i++){
      if(parts[i].type==='timeZoneName'){
        var tag=parts[i].value.replace(/[^A-Za-z0-9]/g,'');
        if(tag)return tag;
      }
    }
  }catch(e){}
  return 'LOCAL';
}
function sessionArchiveName(key){
  var y=+key.slice(0,4),mo=+key.slice(4,6),d=+key.slice(6,8),
      h=+key.slice(9,11),mi=+key.slice(11,13),s=+key.slice(13,15);
  var dt=new Date(Date.UTC(y,mo-1,d,h,mi,s));
  if(isNaN(dt)) return 'rcx_logs_'+key+'_UTC.tar';
  function p(n){return (n<10?'0':'')+n;}
  return 'rcx_logs_'+dt.getFullYear()+p(dt.getMonth()+1)+p(dt.getDate())+'_'+
         p(dt.getHours())+p(dt.getMinutes())+p(dt.getSeconds())+'_'+localZoneTag(dt)+'.tar';
}
function downloadGroup(key){
  var names=groupFiles[key];
  if(!names||!names.length){ toast('Nothing to download in that session',false); return; }
  runExport(names,sessionArchiveName(key));
}
function deleteGroup(key){
  var names=(groupFiles[key]||[]).slice();
  if(!names.length){ toast('Nothing to delete in that session',false); return; }
  var h=sessionHeading(key);
  if(!confirm('Delete all '+names.length+' file'+(names.length===1?'':'s')+' from '+h.local+'?'+
              '\nThis cannot be undone.'))return;
  // One request: a session holds one file per enabled log channel, far below the
  // server's per-request cap, so this never needs the chunking the free-form
  // selection delete does.
  var ds=document.getElementById('delstatus');
  ds.textContent='Deleting session\u2026';
  fetch('/log/delete',{method:'POST',headers:{'Content-Type':'application/x-www-form-urlencoded'},
    body:'names='+encodeURIComponent(names.join(','))})
    .then(function(r){
      if(r.status===409)throw new Error('Busy: a download is in progress');
      if(!r.ok)throw new Error('Delete failed ('+r.status+')');
      return r.json();})
    .then(function(res){
      ds.textContent='';
      names.forEach(function(n){delete selected[n];});
      var kept=(res.failed||[]).length;
      toast('Deleted '+(res.deleted||0)+' file'+(res.deleted===1?'':'s')+
            (kept?(', '+kept+' kept'):''),kept===0);
      loadFiles(); loadSD();})
    .catch(function(e){
      ds.textContent='';
      toast(e.message||'Delete failed',false);
      loadFiles(); loadSD();});
}
function deleteSelected(){
  var names=Object.keys(selected);
  if(!names.length)return;
  if(!confirm('Delete '+names.length+' selected file'+(names.length===1?'':'s')+'?\nThis cannot be undone.'))return;
  var btn=document.getElementById('btn_delsel');
  btn.disabled=true;
  var done=0,failed=0;
  var ds=document.getElementById('delstatus');
  function next(){
    if(!names.length){
      ds.textContent='';
      toast('Deleted '+done+' file'+(done===1?'':'s')+(failed?(', '+failed+' failed'):''),failed===0);
      loadFiles(); loadSD(); return;
    }
    var batch=names.splice(0,CHUNK);
    ds.textContent='Deleting\u2026 '+done+' done';
    fetch('/log/delete',{method:'POST',headers:{'Content-Type':'application/x-www-form-urlencoded'},
      body:'names='+encodeURIComponent(batch.join(','))})
      .then(function(r){
        if(r.status===409){throw new Error('Busy: a download is in progress');}
        if(!r.ok){throw new Error('Delete failed ('+r.status+')');}
        return r.json();})
      .then(function(res){
        done+=res.deleted||0;
        failed+=(res.failed||[]).length;
        batch.forEach(function(n){delete selected[n];});
        next();})
      .catch(function(e){
        ds.textContent='';
        toast(e.message||'Delete failed',false);
        loadFiles(); loadSD();});
  }
  next();
}
function deleteOld(){
  // The endpoint scans the card and skips open files; trust its returned count
  // rather than pre-gating on /sd's per-session file counter.
  if(!confirm('Delete all old log files?\n(Any file recording right now is kept.)')) return;
  document.getElementById('delstatus').textContent='Deleting\u2026';
  fetch('/sd/deleteold',{method:'POST'}).then(function(r){
    if(r.status===409){toast('Can\u2019t delete while a download is running',false);return null;}
    if(!r.ok){toast('Delete failed',false);return null;}
    return r.json();
  }).then(function(res){
    document.getElementById('delstatus').textContent='';
    if(!res)return;
    toast(res.deleted>0?('Deleted '+res.deleted+' file'+(res.deleted===1?'':'s')):'No old files to delete',res.deleted>0);
    selected={}; pgOffset=0; loadFiles(); loadSD();
  }).catch(function(){toast('Delete failed',false);document.getElementById('delstatus').textContent='';});}
// Pausing closes the open files, which changes which rows are selectable.
window.afterPauseChange=function(){chain([function(){return loadFiles(true);},loadSD]);};
chain([loadSD,loadPause,function(){setMode('active');}])
  .then(function(){
    // Only the currently-recording view is polled: it is a pure-RAM read of a
    // handful of entries. The recent and all views are never auto-refreshed —
    // they are larger responses, and re-fetching one under the operator
    // mid-selection would fight with what they are doing. Downloads suspend
    // polling entirely so nothing competes with the transfer.
    pollLoop(function(tick){
      var fns=[];
      if(dlBusy)return fns;
      if(mode==='active')fns.push(function(){return loadFiles(true);});
      if(tick%5===0)fns.push(loadSD);
      return fns;
    },3000);
  });
</script></body></html>)html";

// ── Route registration ─────────────────────────────────────────────────────────
// Escape the characters that would otherwise terminate or corrupt a JSON string.
// Host names, mountpoints and SSIDs are all operator-supplied and reach the
// dashboard through these payloads, so none of them are emitted raw.
static String jsonEscape(const char* in) {
    String out;
    if (!in) return out;
    for (const char* p = in; *p; ++p) {
        unsigned char c = (unsigned char)*p;
        // A literal newline or control character inside a JSON string is invalid
        // and aborts the parse on the dashboard side, taking the whole payload
        // with it. Multi-line values such as the DBC audit depend on this.
        switch (c) {
            case '"':  out += "\\\""; break;
            case '\\': out += "\\\\"; break;
            case '\n': out += "\\n";  break;
            case '\r': out += "\\r";  break;
            case '\t': out += "\\t";  break;
            default:
                if (c < 0x20) { char u[7]; snprintf(u, sizeof(u), "\\u%04x", c); out += u; }
                else out += (char)c;
        }
    }
    return out;
}

// Page-load trace. The DMA-capable figures matter more than the general internal
// heap here: that narrower pool is where the response assembly buffer, the SD
// driver's transfer buffers and the WiFi stack all come from, so it is the one
// that runs out first and the one a corrupted page load would correlate with.
static void logPageRequest(const char* path) {
    Serial.printf("🌐 GET %s (DMA free %lu B, largest %lu B)\n", path,
                  (unsigned long)heap_caps_get_free_size(MALLOC_CAP_DMA),
                  (unsigned long)heap_caps_get_largest_free_block(MALLOC_CAP_DMA));
}

// ── Response sending ──────────────────────────────────────────────────────────
// Bodies go out through a filler callback rather than being handed to the server
// whole, because the two response classes underneath both discard body bytes when
// the socket write fails, and neither checks that it failed:
//
//   AsyncAbstractResponse::_ack   fills the buffer from the source, advances
//                                 _sentLength, calls client()->write(), then
//                                 free()s the buffer regardless of the result.
//   AsyncBasicResponse::_ack      does _content = _content.substring(space) and
//                                 _sentLength += space BEFORE the write call.
//
// AsyncClient::write() returns 0 whenever lwIP cannot allocate a pbuf to copy into
// (tcp_write → ERR_MEM). client()->space() does not predict that: it reports the
// TCP send window, which says nothing about whether a contiguous internal-DRAM
// block exists to build the segment in. So a failed write removes a span from the
// MIDDLE of the response and the transfer carries on as if nothing happened — the
// browser receives a document with a hole in it, which is why a page can appear to
// cut off and restart, and why a JSON endpoint can return unparseable text and
// leave its fields blank.
//
// The filler callback is the one place with a veto. AsyncChunkedResponse tests for
// RESPONSE_TRY_AGAIN before advancing _filledLength, so the same bytes are offered
// again on the next ack or poll rather than being lost. webSendChunk() uses that
// veto to size each chunk to what the heap can actually build a segment from.
//
// Declining is cheap for the BODY but not for the system: _ack() malloc's a buffer
// the size of the whole TCP send window before the filler is ever consulted, and
// repeats that allocate/free on every poll while the filler keeps saying no. An
// unbounded veto therefore does not wait quietly — it pins the pool the SDMMC
// driver needs, at poll cadence, forever. So the veto is bounded, and the chunk is
// never capped below what the server has already allocated for.

// Headroom left above the chunk itself, in the same pool. lwIP builds the segment
// with pbuf_alloc(PBUF_RAM), which needs the chunk plus its own header and TCP
// segment structures; leaving less than this wins the allocation and then starves
// the layer that has to carry it.
#define WEB_SEND_RESERVE     1024
// Smallest chunk worth sending. Below this the per-segment overhead dominates and
// a page needs so many acks that the server's own per-ack buffer churn becomes the
// problem instead.
#define WEB_SEND_MIN         256
// Consecutive declines before a chunk goes out anyway. A decline is not free: the
// server has ALREADY malloc'd a buffer the size of the whole TCP send window by
// the time the filler is consulted, and it repeats that allocate/free on every
// poll for as long as the filler keeps saying no. So an unbounded veto does not
// wait quietly — it holds the pool hostage at roughly 8 Hz and never finishes,
// which is worse for the SDMMC driver competing for the same memory than one
// small write that might not land. ~5 s of declines, then progress regardless.
#define WEB_SEND_MAX_DEFERS  40

// Chunk size the internal heap can currently support, or 0 to defer.
// MALLOC_CAP_INTERNAL|MALLOC_CAP_8BIT is the pool lwIP takes PBUF_RAM from — the
// one that governs whether a write succeeds. The general free-heap figure is not a
// substitute: it counts PSRAM, which no TCP segment can be built in.
//
// Note what is NOT done here: the chunk is not capped to some fixed size below
// maxLen. maxLen is derived from the send window the server has already allocated
// for, so returning less than it leaves that allocation partly unused and forces
// another ack — another full-window malloc — to move the same body. Filling what
// is offered, when memory allows, is what keeps a page down to a handful of
// allocation cycles instead of a dozen.
static size_t webSendChunk(size_t maxLen, uint16_t& defers) {
    const size_t largest =
        heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    size_t room = (largest > WEB_SEND_RESERVE) ? largest - WEB_SEND_RESERVE : 0;
    if (room > maxLen) room = maxLen;
    if (room >= maxLen || room >= WEB_SEND_MIN) { defers = 0; return room; }
    if (++defers < WEB_SEND_MAX_DEFERS) return 0;      // caller → RESPONSE_TRY_AGAIN
    defers = 0;
    // At most one line per WEB_SEND_MAX_DEFERS polls per response, so this cannot
    // itself become the load. It is the only way to tell a stalled send from an
    // ordinary slow one in a serial capture.
    Serial.printf("🌐 Send stalled — largest internal block %lu B, forcing %u B\n",
                  (unsigned long)largest, (unsigned)WEB_SEND_MIN);
    return maxLen < WEB_SEND_MIN ? maxLen : WEB_SEND_MIN;
}

// Send a body that lives in flash (the pages and the shared assets). Nothing is
// copied to RAM ahead of time; each chunk is read straight out of rodata as the
// socket accepts it, so a 17 KB page costs no heap beyond the chunk in flight.
static void webSendFlash(AsyncWebServerRequest* req, const char* contentType,
                         const char* body, const char* etag) {
    const size_t total = strlen_P(body);
    // Nagle would hold a segment back waiting to coalesce with the next while the
    // client waits to ACK. Same interaction, and same fix, as the download path.
    req->client()->setNoDelay(true);
    // Per-response decline counter, alive as long as the filler is.
    auto defers = std::make_shared<uint16_t>(0);
    AsyncWebServerResponse* r = req->beginChunkedResponse(contentType,
        [body, total, defers](uint8_t* buf, size_t maxLen, size_t index) -> size_t {
            if (index >= total) return 0;                 // 0 is end-of-body
            size_t want = total - index;
            if (want > maxLen) want = maxLen;
            const size_t n = webSendChunk(want, *defers);
            if (!n) return RESPONSE_TRY_AGAIN;            // offered again later
            memcpy_P(buf, body + index, n);
            return n;
        });
    if (etag) {
        r->addHeader("ETag", etag);
        r->addHeader("Cache-Control", "no-cache");
    } else {
        r->addHeader("Cache-Control", "no-store");
    }
    req->send(r);
}

// Send a body assembled at request time. The payload is held by shared_ptr for the
// life of the response — the filler outlives the handler that built it.
static void webSendBody(AsyncWebServerRequest* req, const char* contentType,
                        std::string body) {
    auto payload = std::make_shared<std::string>(std::move(body));
    auto defers  = std::make_shared<uint16_t>(0);
    req->client()->setNoDelay(true);
    AsyncWebServerResponse* r = req->beginChunkedResponse(contentType,
        [payload, defers](uint8_t* buf, size_t maxLen, size_t index) -> size_t {
            const size_t total = payload->size();
            if (index >= total) return 0;
            size_t want = total - index;
            if (want > maxLen) want = maxLen;
            const size_t n = webSendChunk(want, *defers);
            if (!n) return RESPONSE_TRY_AGAIN;
            memcpy(buf, payload->data() + index, n);
            return n;
        });
    r->addHeader("Cache-Control", "no-store");
    req->send(r);
}

void webserver_init() {
    // ── Pages ────────────────────────────────────────────────────────────────
    // Three documents rather than one: the dashboard is what gets opened in the
    // paddock and must load and refresh without dragging the whole setup surface
    // along with it. All three share /app.css and /app.js so the common styling
    // and helpers exist once in flash instead of three times.
    //
    // Each page log line carries the DMA-capable free heap and largest free block
    // as the response is handed to the server. That is the pool ESPAsyncWebServer
    // mallocs from on every ack to assemble the next segment, and the pool the SD
    // driver and WiFi also draw on. When a page arrives in the browser truncated
    // or restarted mid-document, this line is what says whether the send began
    // under memory pressure — a low figure here alongside a broken page is the
    // evidence, and a healthy one rules the theory out.
    webServer.on("/", HTTP_GET, [](AsyncWebServerRequest* req) {
        logPageRequest("/");
        webSendFlash(req, "text/html", PAGE_DASH, nullptr);
    });
    webServer.on("/setup", HTTP_GET, [](AsyncWebServerRequest* req) {
        logPageRequest("/setup");
        webSendFlash(req, "text/html", PAGE_SETUP, nullptr);
    });
    webServer.on("/logs", HTTP_GET, [](AsyncWebServerRequest* req) {
        logPageRequest("/logs");
        webSendFlash(req, "text/html", PAGE_LOGS, nullptr);
    });

    // Shared assets. These are the two largest responses on any page, and serving
    // them no-store meant re-fetching both on every page load and every
    // navigation — the worst thing to do on a board this tight on internal SRAM,
    // since each fetch is another concurrent request holding buffers.
    //
    // They carry an ETag derived from the build, so a browser revalidates with a
    // conditional request and gets a 304 with no body in the normal case, while a
    // firmware update still invalidates immediately. That keeps the staleness
    // safety of no-store without paying for the payload every time.
    webServer.on("/app.css", HTTP_GET, [](AsyncWebServerRequest* req) {
        if (req->header("If-None-Match") == ASSET_ETAG) { req->send(304); return; }
        webSendFlash(req, "text/css", WEB_CSS, ASSET_ETAG);
    });
    webServer.on("/app.js", HTTP_GET, [](AsyncWebServerRequest* req) {
        if (req->header("If-None-Match") == ASSET_ETAG) { req->send(304); return; }
        webSendFlash(req, "application/javascript", WEB_JS, ASSET_ETAG);
    });

    // Browsers request this on their own for every page, so without a route it
    // arrives as a third connection racing the page and its stylesheet, is served
    // by the 404 handler, and repeats on the next navigation because a 404 is not
    // cached. A cached 204 answers it once per browser session and removes one
    // concurrent response from every page load — which is the resource this board
    // actually runs out of.
    webServer.on("/favicon.ico", HTTP_GET, [](AsyncWebServerRequest* req) {
        AsyncWebServerResponse* r = req->beginResponse(204);
        r->addHeader("Cache-Control", "public, max-age=86400");
        req->send(r);
    });

    // ── /status — live telemetry snapshot ────────────────────────────────────
    webServer.on("/status", HTTP_GET, [](AsyncWebServerRequest* req) {
        GnssData g_s{}; SystemStatus s_s{}; CanData c_s{};
        if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
            g_s = gps; s_s = status; c_s = can;
            xSemaphoreGive(dataMutex);
        }
        // Fix-state string. PPP MUST be shown distinctly: it arrives as GGA quality 5, the
        // same code as RTK FLOAT, so without the pppActive test the dashboard would call a
        // base-less decimetre solution "FLOAT" and imply corrections are flowing when none are.
        const char* rtk = (g_s.rtkType==2)?"FIXED"
                        : g_s.pppActive   ?((g_s.diffStationId==9002)?"PPP E6":"PPP B2b")
                        : (g_s.rtkType==1)?"FLOAT"
                        : g_s.valid       ?"3D GPS":"NO FIX";
        char acc[24]; snprintf(acc, sizeof(acc), "%.3fm %s",
                               g_s.hAccM, g_s.epeValid ? "EPE" : "est");
        char rpmStr[12], tpsStr[12], oilStr[12];
        snprintf(rpmStr, sizeof(rpmStr), isnan(c_s.rpm)             ? "---" : "%.0f",  c_s.rpm);
        snprintf(tpsStr, sizeof(tpsStr), isnan(c_s.throttleActualPct)? "---" : "%.1f%%",c_s.throttleActualPct);
        snprintf(oilStr, sizeof(oilStr), isnan(c_s.oilTempC)        ? "---" : "%.0f\xc2\xb0""C", c_s.oilTempC);
        // Device temp → °F for display; "--" when not yet read / sensor fault.
        char espTStr[12];
        snprintf(espTStr, sizeof(espTStr), isnan(s_s.espTempC) ? "--" : "%.1f", cToF(s_s.espTempC));
        char json[1152];
        snprintf(json, sizeof(json),
            "{\"wifi\":\"%s\",\"ssid\":\"%s\",\"ntrip\":\"%s\",\"mount\":\"%s\",\"base_km\":%.1f,\"rtk\":\"%s\","
            "\"diff_age\":%.1f,\"rtk_hold_s\":%d,"
            "\"sats\":%d,\"acc\":\"%s\",\"ble\":\"%s\",\"ble_hz\":\"%.0f\","
            "\"rc\":\"%s\",\"can\":\"%s\","
            "\"gps_hz\":\"%.0f\",\"can_hz\":\"%.0f\","
            "\"rpm\":\"%s\",\"tps\":\"%s\",\"oil_c\":\"%s\","
            "\"lat\":%.9f,\"lon\":%.9f,"
            "\"esp_temp_f\":\"%s\",\"reset\":\"%s\","
            "\"rtcm_bytes\":%lu,\"heap\":%lu,\"heap_min\":%lu,\"heap_block_min\":%lu}",
            s_s.wifiConnected?"connected":s_s.wifiAttempting?"connecting":"---",
            s_s.wifiSSID,
            s_s.ntripConnected?"connected":"---",
            s_s.mountpoint,
            s_s.ntripDistanceKm,
            rtk,
            g_s.diffAgeS, (int)RTK_HOLD_TIMEOUT_S,
            g_s.numSV, acc,
            s_s.bleConnected?"connected":"---",
            s_s.blePacketHz,
            s_s.rcConnected?"connected":"---",
            s_s.canHz > 0.0f ? "connected" : "no data",
            s_s.gnssHz, s_s.canHz,
            rpmStr, tpsStr, oilStr,
            g_s.latitude, g_s.longitude,
            espTStr, g_resetReasonStr,
            (unsigned long)rtcmBytesTotal,
            (unsigned long)heap_caps_get_free_size(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT),
            (unsigned long)g_minFreeInternal, (unsigned long)g_minLargestBlock);
        webSendBody(req, "application/json", json);
    });

    // ── /sd — SD card capacity + file count ──────────────────────────────────
    webServer.on("/sd", HTTP_GET, [](AsyncWebServerRequest* req) {
        if (!sdlog_isReady()) {
            AsyncWebServerResponse* r = req->beginResponse(200, "application/json",
                "{\"ready\":false,\"total\":0,\"used\":0,\"free\":0,\"files\":0,\"active\":false}");
            r->addHeader("Cache-Control", "no-store");
            req->send(r); return;
        }
        // Capacity is cached by sdLogTask (see sdlog_getCapacity) — the web
        // handler does NO SD-bus access. Calling f_getfree here, in the async_tcp
        // callback, was a Task-WDT risk on a large/full card (same failure class
        // as the dir scan). total==0 only in the brief window before sdLogTask's
        // first refresh after mount.
        uint64_t total = 0, used = 0;
        sdlog_getCapacity(&total, &used);
        char json[192];
        snprintf(json, sizeof(json),
            "{\"ready\":true,\"total\":%llu,\"used\":%llu,\"free\":%llu,"
            "\"files\":%lu,\"active\":%s}",
            total, used, total > used ? total - used : 0ULL,
            (unsigned long)sdlog_getFileCount(),
            (sdlog_getActiveName() && sdlog_getActiveName()[0]) ? "true" : "false");
        AsyncWebServerResponse* r = req->beginResponse(200, "application/json", json);
        r->addHeader("Cache-Control", "no-store");
        req->send(r);
    });

    // ── /files — JSON listing of the .csv files on SD ─────────────────────────
    // Returns: {"sd_ready":bool,"mode":"active|recent|all","files":[{"name","size"
    //           [,"active":true]},...],"offset":N,"total":N,"refreshing":bool,
    //           "truncated":bool}
    // sd_ready lets the browser distinguish "no files yet" from "SD is still
    // initialising"; total/offset drive paging in the all view.
    webServer.on("/files", HTTP_GET, [](AsyncWebServerRequest* req) {
        if (!sdlog_isReady()) {
            AsyncWebServerResponse* r = req->beginResponse(200, "application/json",
                                                           "{\"sd_ready\":false,\"files\":[]}");
            r->addHeader("Cache-Control", "no-store");
            req->send(r); return;
        }

        // Three-stage view, all RAM only — no request here ever touches the SD
        // card, not even sdLogTask's own mutex. See sd_log.h for how: this
        // firmware creates every file it ever lists, so it maintains the
        // listing as files open and close instead of ever re-reading the card.
        //   (default)   active  — currently-open files, live sizes
        //   ?recent=1   last 12 — 12 most-recent files by timestamp
        //   ?all=1      paged   — the whole listing, &offset=N&limit=M at a time
        // Filenames are <type>_YYYYMMDD_HHMMSS.csv, so the timestamp substring
        // (name+4) sorts lexicographically == chronologically.
        //
        // ALL is paged because this response is assembled in a std::string on the
        // internal heap — the scarcest RAM on this board — so the number of entries
        // one request can carry has to be bounded no matter how many the card holds.
        // "total" tells the caller how far it can page.
        enum { MODE_ACTIVE, MODE_RECENT, MODE_ALL };
        int mode = req->hasParam("all")    ? MODE_ALL
                 : req->hasParam("recent") ? MODE_RECENT
                                           : MODE_ACTIVE;
        int offset = req->hasParam("offset") ? req->getParam("offset")->value().toInt() : 0;
        int limit  = req->hasParam("limit")  ? req->getParam("limit")->value().toInt()  : FILES_PAGE_DEFAULT;
        if (offset < 0) offset = 0;
        if (limit  < 1) limit  = FILES_PAGE_DEFAULT;
        if (limit  > FILES_PAGE_MAX) limit = FILES_PAGE_MAX;

        std::string json = "{\"sd_ready\":true,\"mode\":";
        json += (mode == MODE_ALL) ? "\"all\"," : (mode == MODE_RECENT) ? "\"recent\"," : "\"active\",";
        json += "\"files\":[";
        bool first = true;
        bool respRefreshing = false, respTrunc = false;

        // ── Currently-open files, from RAM, with LIVE sizes. In the active and
        //    recent views these lead the list and must not depend on the
        //    directory listing (which lags under heavy logging, making a
        //    recording channel look like it isn't recording). The paged view
        //    below sources its rows from the snapshot instead, and uses this
        //    same set only to mark which of them are open.
        char     anames[4][LOG_NAME_MAX];
        uint32_t asizes[4];
        int an = sdlog_getActiveFiles(anames, asizes, 4);
        int total = 0;
        if (mode != MODE_ALL) {
            for (int i = 0; i < an; i++) {
                if (!first) json += ','; first = false;
                json += "{\"name\":\""; json += anames[i];
                json += "\",\"size\":"; json += std::to_string(asizes[i]);
                json += ",\"active\":true}";
            }
        }

        if (mode == MODE_ALL) {
            // Paged straight from the snapshot with no separate active-file
            // prepend: open files are already IN the snapshot (they were added
            // the moment they opened), so prepending them here would both
            // duplicate rows and break the page arithmetic against "total".
            // Each entry is checked against the open set instead, which is what
            // supplies the live size and the not-deletable marking.
            static char     pageNames[FILES_PAGE_MAX][LOG_NAME_MAX];
            static uint32_t pageSizes[FILES_PAGE_MAX];
            int n = sdlog_getDirSnapshotPage(pageNames, pageSizes, offset, limit,
                                             &total, &respTrunc, nullptr);
            respRefreshing = sdlog_dirScanPending();
            for (int i = 0; i < n; i++) {
                bool     act  = false;
                uint32_t size = pageSizes[i];
                for (int a = 0; a < an; a++)
                    if (strcmp(pageNames[i], anames[a]) == 0) { act = true; size = asizes[a]; break; }
                if (!first) json += ','; first = false;
                json += "{\"name\":\""; json += pageNames[i];
                json += "\",\"size\":"; json += std::to_string(size);
                if (act) json += ",\"active\":true";
                json += "}";
            }
        } else if (mode != MODE_ACTIVE) {
            // ── RECENT: the newest 12 historical (closed) files from the RAM
            //    listing sdLogTask maintains. NO SD access / NO sdMutex here —
            //    enumerating the root in this async_tcp callback is what tripped
            //    the Task-WDT. Open files are already listed above, so skip any
            //    entry that matches one (dedup).
            const int want = 12;
            static char     snapNames[12][LOG_NAME_MAX];
            static uint32_t snapSizes[12];
            int n = sdlog_getDirSnapshot(snapNames, snapSizes, want, nullptr, nullptr);
            respRefreshing = sdlog_dirScanPending();
            respTrunc      = false;   // a deliberate newest-12 window is not truncation
            for (int i = 0; i < n; i++) {
                bool dup = false;                                // already shown as active?
                for (int a = 0; a < an; a++)
                    if (strcmp(snapNames[i], anames[a]) == 0) { dup = true; break; }
                if (dup) continue;
                if (!first) json += ','; first = false;
                json += "{\"name\":\""; json += snapNames[i];
                json += "\",\"size\":"; json += std::to_string(snapSizes[i]);
                json += "}";
            }
        }
        json += "],\"offset\":";
        json += std::to_string(mode == MODE_ALL ? offset : 0);
        json += ",\"total\":";
        json += std::to_string(total);
        json += ",\"refreshing\":";
        json += respRefreshing ? "true" : "false";
        json += ",\"truncated\":";
        json += respTrunc ? "true" : "false";
        json += "}";

        webSendBody(req, "application/json", std::move(json));
    });

    // ── /log/<file> GET — download a CSV file ─────────────────────────────────
    // REQUIRES logging to be paused. The chunk callback below reads the file on
    // the async_tcp task, and that read takes the FatFs volume lock — the same
    // lock sdLogTask holds for its writes. With logging live, async_tcp blocks
    // INSIDE file.read(), where the WDT feed just above it has already happened
    // and cannot help; async_tcp is the single task serving the whole web UI, so
    // the device goes unresponsive (confirmed in the field: paused downloads are
    // fast and clean, unpaused ones froze the unit). Refusing here rather than
    // only in the browser is deliberate — the failure belongs to the endpoint,
    // not to one page's buttons.
    webServer.on("/log/*", HTTP_GET, [](AsyncWebServerRequest* req) {
        if (!sdlog_isReady()) {
            req->send(503, "text/plain", "SD not ready"); return;
        }
        if (!sdlog_isPaused()) {
            req->send(409, "text/plain",
                "Pause logging before downloading — a download competes with the "
                "recording writes for the SD card and will stall the device.");
            return;
        }
        // One transfer at a time, enforced at the endpoint. Every response in
        // flight costs a per-ack malloc() out of the same internal/DMA-capable
        // pool the SD driver and WiFi already compete for, and a streamed file is
        // the largest and longest-lived of them. Two at once is what drives that
        // pool low enough for other responses — the pages above, in particular —
        // to come apart mid-send. A whole selection now arrives as one .tar via
        // /export.tar, so nothing in the UI needs concurrent transfers.
        if (g_activeDownloads > 0) {
            req->send(409, "text/plain",
                "Busy: another download is already in progress. "
                "Wait for it to finish, or use Download selected for one archive.");
            return;
        }
        // Extract filename: "/log/foo.csv" → substring(5) → "foo.csv" → "/foo.csv"
        String fn = req->url().substring(5);
        if (!fn.startsWith("/")) fn = "/" + fn;

        // Reject path traversal and anything but root .csv logs / the debug .txt.
        // The debug file is deliberately allowed while still OPEN: this route never
        // takes sdMutex (see above) and reads only to the size its own handle saw,
        // so serving it cannot disturb the writer on sdLogTask.
        bool allowed = fn.endsWith(".csv") ||
                       (fn.startsWith("/debug_") && fn.endsWith(".txt"));
        if (!allowed || fn.indexOf('/', 1) >= 0) {
            req->send(403, "text/plain", "Only root .csv or debug .txt files allowed"); return;
        }

        // DlState owns the File; its destructor closes it when the response ends
        // (either at EOF or on client disconnect — whichever comes first). The
        // ctor/dtor also bump g_activeDownloads so deletes can't pull a file out
        // from under an in-flight read (which resets the device), and separately
        // tell sdLogTask to defer its periodic capacity/directory maintenance
        // scans for the duration — those compete with this read for the SD bus
        // (see sd_log.h).
        //
        // No sdMutex is taken in the chunk callback. FatFS serialises file
        // operations internally, and taking the logging module's mutex per chunk
        // would also cause premature EOF: ESPAsyncWebServer reads a chunk-callback
        // return of 0 as end-of-file, so a mutex timeout would silently truncate
        // the download mid-stream. The pause requirement above is what keeps the
        // writer off the volume lock instead.
        struct DlState {
            File file;
            DlState()  { g_activeDownloads++; sdlog_downloadBegin(); }
            ~DlState() { if (file) file.close(); g_activeDownloads--; sdlog_downloadEnd(); }
        };
        auto state = std::make_shared<DlState>();
        state->file = SD_MMC.open(fn, FILE_READ);
        if (!state->file) {
            Serial.printf("🌐 /log%s — not found\n", fn.c_str());
            req->send(404, "text/plain", "Not found: " + fn); return;
        }

        Serial.printf("🌐 Download: %s (%lu B)\n", fn.c_str(),
                      (unsigned long)state->file.size());

        // Nagle's algorithm (on by default) holds a chunk back waiting either to
        // coalesce with more data or for the peer's ACK, and most HTTP clients run
        // delayed ACK — the two together cap throughput at roughly one segment per
        // ACK round trip, tens of KB/s over WiFi regardless of how fast the SD read
        // is. Same fix already proven on the RCX1 base's NTRIP casting socket.
        req->client()->setNoDelay(true);

        String bare = fn.startsWith("/") ? fn.substring(1) : fn;
        auto* resp = req->beginChunkedResponse("text/csv",
            [state](uint8_t* buf, size_t maxLen, size_t index) -> size_t {
                // Streamed SD reads run in the async_tcp task. Each chunk is a
                // single bounded read (fast), but on a busy/slow card a read can
                // stall behind a logger write on the FatFs volume lock; feed the
                // task WDT each chunk so a transient stall can't abort the device
                // (harmless no-op if async_tcp isn't WDT-subscribed).
                esp_task_wdt_reset();
                if (!state->file) return 0;
                // Seek if the byte position diverged (e.g. range request retry)
                if ((uint32_t)state->file.position() != (uint32_t)index)
                    state->file.seek((uint32_t)index);
                return state->file.read(buf, maxLen);
            });
        resp->addHeader("Content-Disposition", "attachment; filename=\"" + bare + "\"");
        resp->addHeader("Cache-Control", "no-store");
        req->send(resp);
    });

    // ── /log/<file> DELETE — delete a CSV file ────────────────────────────────
    // Failure modes here are now all LEGIBLE. The previous version returned a bare
    // 404 on three different conditions — including a silent mutex timeout that
    // logged nothing — which looked like a dead button. It also had no guard against
    // deleting a file open for WRITING (the canraw sniffer dump), where SD_MMC.remove()
    // on the live handle can corrupt FatFs and reset the unit.
    // ── /log/delete POST — delete a batch of named files ─────────────────────
    // Body: names=a.csv,b.csv,... (form-encoded). Registered BEFORE the /log/*
    // wildcard routes so this exact path is matched by this handler rather than
    // being read as a filename by them.
    //
    // Bounded to DELETE_BATCH_MAX names per request: sdMutex is held across the
    // whole run, and the recording writes wait only 50 ms for that mutex before
    // dropping their row, so an unbounded batch would silently cost log data.
    // The browser splits a larger selection across successive requests. The WDT
    // is fed between removes for the same reason /sd/deleteold does it — this
    // runs on the async_tcp task, which IS watchdog-subscribed.
    webServer.on("/log/delete", HTTP_POST, [](AsyncWebServerRequest* req) {
        if (!sdlog_isReady()) { req->send(503, "text/plain", "SD not ready"); return; }
        if (g_activeDownloads > 0) {   // see the single-file delete guard below
            req->send(409, "text/plain", "Busy: a download is in progress"); return;
        }
        if (!req->hasParam("names", true)) {
            req->send(400, "text/plain", "No files given"); return;
        }

        // Split the comma-separated list, validating each name before it can
        // reach SD_MMC: root .csv only, no path separators, never a file that is
        // open for writing.
        String list = req->getParam("names", true)->value();
        std::vector<String> want;
        int start = 0;
        while (start <= list.length() && (int)want.size() < DELETE_BATCH_MAX) {
            int comma = list.indexOf(',', start);
            String nm = (comma < 0) ? list.substring(start) : list.substring(start, comma);
            nm.trim();
            if (nm.length() && nm.endsWith(".csv") && nm.indexOf('/') < 0 && nm.indexOf('\\') < 0)
                want.push_back(nm);
            if (comma < 0) break;
            start = comma + 1;
        }
        if (want.empty()) { req->send(400, "text/plain", "No valid filenames"); return; }

        int deleted = 0;
        std::vector<String> removedOk, failed;
        auto sdm = sdlog_getMutex();
        if (!sdm) { req->send(503, "text/plain", "SD subsystem not ready"); return; }
        if (xSemaphoreTake(sdm, pdMS_TO_TICKS(2000)) != pdTRUE) {
            req->send(503, "text/plain", "SD busy — try again"); return;
        }
        for (size_t idx = 0; idx < want.size(); idx++) {
            const String& nm = want[idx];
            esp_task_wdt_reset();
            // Feeding the watchdog is not the same as yielding. This loop runs on
            // the async_tcp task, which is also the only task feeding bytes to
            // every other response currently in flight; esp_task_wdt_reset() keeps
            // the WDT quiet but never lets the scheduler run, so a full batch of
            // SD_MMC.remove() calls monopolises that task for its entire duration.
            // Any page or asset response mid-send during that window is starved,
            // and a starved AsyncAbstractResponse either stalls on a failed
            // malloc() or takes a short client()->write() whose unwritten bytes it
            // discards while still advancing its sent counter — a page truncated
            // against its own Content-Length. One tick every few files is enough
            // for the scheduler to service the other connections and costs the
            // delete almost nothing.
            if (idx % 4 == 3) vTaskDelay(1);
            if (sdlog_isActiveFile(nm.c_str())) {   // recording — refuse, don't corrupt FatFs
                Serial.printf("⛔ SD batch skip (recording): %s\n", nm.c_str());
                failed.push_back(nm); continue;
            }
            String path = "/" + nm;
            if (!SD_MMC.exists(path))        { failed.push_back(nm); continue; }
            if (SD_MMC.remove(path)) {
                deleted++; removedOk.push_back(nm);
                Serial.printf("🗑️  SD batch: %s\n", nm.c_str());
            } else {
                Serial.printf("⚠️  SD batch fail: %s\n", nm.c_str());
                failed.push_back(nm);
            }
        }
        xSemaphoreGive(sdm);
        // Snapshot updates only AFTER sdMutex is released — the two locks are
        // never nested (see the single-file delete and sd_log.h).
        for (auto& nm : removedOk) sdlog_noteFileDeleted(nm.c_str());

        std::string json = "{\"deleted\":" + std::to_string(deleted) + ",\"failed\":[";
        for (size_t i = 0; i < failed.size(); i++) {
            if (i) json += ',';
            json += "\""; json += jsonEscape(failed[i].c_str()).c_str(); json += "\"";
        }
        json += "]}";
        Serial.printf("🗑️  SD batch-delete: %d removed, %d failed\n", deleted, (int)failed.size());
        webSendBody(req, "application/json", std::move(json));
    });

    webServer.on("/log/*", HTTP_DELETE, [](AsyncWebServerRequest* req) {
        if (!sdlog_isReady()) {
            req->send(503, "text/plain", "SD not ready"); return;
        }
        // Refuse while any DOWNLOAD holds a file open (same corruption risk).
        if (g_activeDownloads > 0) {
            req->send(409, "text/plain", "Busy: a download is in progress"); return;
        }
        String fn = req->url().substring(5);
        if (!fn.startsWith("/")) fn = "/" + fn;
        if (!fn.endsWith(".csv") || fn.indexOf('/', 1) >= 0) {
            req->send(400, "text/plain", "Only root .csv files may be deleted"); return;
        }

        // Refuse to delete a file that's open for WRITING right now — the four typed
        // channels OR the canraw sniffer dump (which isn't a typed channel, so it
        // needs this explicit check). This is the guard that was missing: the canraw
        // file is held open by sdLogTask while the sniffer is on, and removing it
        // here corrupts FatFs + resets the device.
        String bare = fn.startsWith("/") ? fn.substring(1) : fn;
        if (sdlog_isActiveFile(bare.c_str())) {
            Serial.printf("⛔ SD DELETE refused (recording): %s\n", bare.c_str());
            req->send(409, "text/plain",
                "That file is recording now. Turn off its logging channel "
                "(or the CAN sniffer) first, then delete.");
            return;
        }

        auto sdm = sdlog_getMutex();
        if (!sdm) { req->send(503, "text/plain", "SD subsystem not ready"); return; }
        // 2000 ms (was 500): under active multi-channel logging the SD bus is held
        // in bursts and 500 ms lost the race, returning a SILENT 404. On timeout we
        // now return an honest, retryable 503 instead of a misleading 404.
        if (xSemaphoreTake(sdm, pdMS_TO_TICKS(2000)) != pdTRUE) {
            Serial.printf("⏳ SD DELETE busy (mutex timeout): %s\n", fn.c_str());
            req->send(503, "text/plain", "SD busy — try again"); return;
        }
        int status; const char* msg;
        bool didDelete = false;
        if (!SD_MMC.exists(fn)) {
            Serial.printf("⚠️  SD DELETE — not found: %s\n", fn.c_str());
            status = 404; msg = "Not found";
        } else if (SD_MMC.remove(fn)) {
            Serial.printf("🗑️  SD: %s deleted\n", fn.c_str());
            didDelete = true;
            status = 200; msg = "Deleted";
        } else {
            Serial.printf("❌ SD DELETE failed (in use?): %s\n", fn.c_str());
            status = 500; msg = "Delete failed (file may be in use)";
        }
        xSemaphoreGive(sdm);
        if (didDelete) sdlog_noteFileDeleted(bare.c_str());   // after sdMutex is released — see sd_log.h
        req->send(status, "text/plain", msg);
    });

    // ── /log_config GET — read current per-channel flags ─────────────────────
    // ── /log_config POST — write flags (params: gps=0|1 imu=0|1 can=0|1 sat=0|1)
    // Both methods use URL query-string params. POST is used for writes so the
    // browser never serves a stale cached toggle response. ESPAsyncWebServer parses
    // query-string params from the URL regardless of HTTP method, so the C++ handler
    // logic is the same for both — we distinguish only to set the correct HTTP method
    // mask on the route (HTTP_GET|HTTP_POST).
    webServer.on("/log_config", HTTP_GET | HTTP_POST, [](AsyncWebServerRequest* req) {
        bool changed = false;
        bool g  = sdlog_getLogGps();
        bool im = sdlog_getLogImu();
        bool c  = sdlog_getLogCan();
        bool s  = sdlog_getLogSat();

        if (req->hasParam("gps")) { g  = req->getParam("gps")->value() != "0"; changed = true; }
        if (req->hasParam("imu")) { im = req->getParam("imu")->value() != "0"; changed = true; }
        if (req->hasParam("can")) { c  = req->getParam("can")->value() != "0"; changed = true; }
        if (req->hasParam("sat")) { s  = req->getParam("sat")->value() != "0"; changed = true; }

        if (changed) {
            // NOTE: the sniffer and log channels MAY now run together (see the
            // force path in /can/sniff — concurrent capture for vehicle-dynamics
            // channel discovery). So enabling a log channel no longer force-drops
            // the sniffer; that would have silently ended a deliberate concurrent
            // run. The throughput trade-off is the user's to make: to go back to
            // sniffer-only, toggle the sniffer off explicitly. The default entry
            // into sniffing is still exclusive unless force=1 is passed.
            // sd_log is the sole NVS owner for "rcx_log" — sdlog_setConfig() sets
            // the live volatile flags and writes to NVS in one call.
            sdlog_setConfig(g, im, c, s);
            Serial.printf("🌐 Log config: GPS=%d IMU=%d CAN=%d SAT=%d\n", g, im, c, s);
            AsyncWebServerResponse* r = req->beginResponse(200, "application/json", "{\"ok\":true}");
            r->addHeader("Cache-Control", "no-store");
            req->send(r); return;
        }

        // Read-only response — current state. "paused" is included because the
        // four flags describe the channel set that WILL run, which during a pause
        // is not the same as what is recording; without it the switches would
        // imply logging is active when it is not.
        char json[128];
        snprintf(json, sizeof(json),
                 "{\"gps\":%s,\"imu\":%s,\"can\":%s,\"sat\":%s,\"paused\":%s}",
                 sdlog_getLogGps() ? "true" : "false",
                 sdlog_getLogImu() ? "true" : "false",
                 sdlog_getLogCan() ? "true" : "false",
                 sdlog_getLogSat() ? "true" : "false",
                 sdlog_isPaused()  ? "true" : "false");
        AsyncWebServerResponse* r = req->beginResponse(200, "application/json", json);
        r->addHeader("Cache-Control", "no-store");
        req->send(r);
    });

    // ── /log_pause GET — read master pause state ──────────────────────────────
    // ── /log_pause POST — set it (param: on=0|1) ─────────────────────────────
    // Pausing suspends every channel and lets the logger task close the files
    // they hold open, which is what makes the current session's files available
    // to download or delete — an open file can never be removed (that corrupts
    // FatFs and resets the device). The close happens on the logger task's next
    // pass, not inside this handler: web handlers touch the SD bus zero times.
    // RAM-only, so a reboot always comes back logging.
    webServer.on("/log_pause", HTTP_GET | HTTP_POST, [](AsyncWebServerRequest* req) {
        if (req->hasParam("on")) {
            bool on = req->getParam("on")->value() != "0";
            sdlog_setPaused(on);
            Serial.printf("🌐 Log pause: %s\n", on ? "paused" : "running");
        }
        char json[32];
        snprintf(json, sizeof(json), "{\"paused\":%s}", sdlog_isPaused() ? "true" : "false");
        AsyncWebServerResponse* r = req->beginResponse(200, "application/json", json);
        r->addHeader("Cache-Control", "no-store");
        req->send(r);
    });

    // ── /lcd GET — read current LCD/backlight enable state ────────────────────
    // ── /lcd POST — set it (param: on=0|1) ─────────────────────────────────────
    // RAM only, on purpose (display.h): the unit must never boot with a blank
    // screen because it happened to be off last time it was powered down, so
    // this is never written to NVS and always starts true. Independent of
    // thermal throttling (thermal.h), which can also blank the same hardware —
    // either wanting it off is enough; this only ever changes the user's own
    // side of that.
    webServer.on("/lcd", HTTP_GET | HTTP_POST, [](AsyncWebServerRequest* req) {
        if (req->hasParam("on")) {
            bool on = req->getParam("on")->value() != "0";
            display_setEnabled(on);
            Serial.printf("🌐 LCD: %s\n", on ? "on" : "off");
        }
        char json[24];
        snprintf(json, sizeof(json), "{\"on\":%s}", display_isEnabled() ? "true" : "false");
        AsyncWebServerResponse* r = req->beginResponse(200, "application/json", json);
        r->addHeader("Cache-Control", "no-store");
        req->send(r);
    });

    // ── /imu/cal GET — current calibration state ───────────────────────────────
    // ── /imu/cal POST — start or clear (param: action=start|clear) ─────────────
    // Bias capture itself is non-blocking (imu.h): start arms it, samples are
    // gathered inline in the normal 50 Hz imu_read() calls over ~2 s, then the
    // result lands here on the next poll. Unit must be stationary and resting
    // flat (one axis along gravity) for the ~2 s capture — moving it fails the
    // capture (state goes to "failed") rather than storing a bad bias.
    webServer.on("/imu/cal", HTTP_GET | HTTP_POST, [](AsyncWebServerRequest* req) {
        if (req->hasParam("action")) {
            String action = req->getParam("action")->value();
            if (action == "start") { imu_requestCalibration(); Serial.println("🌐 IMU cal: start requested"); }
            else if (action == "clear") { imu_requestClearCalibration(); Serial.println("🌐 IMU cal: clear requested"); }
        }
        const char* stateStr;
        switch (imu_calibrationState()) {
            case IMU_CAL_RUNNING: stateStr = "running"; break;
            case IMU_CAL_DONE:    stateStr = "done";    break;
            case IMU_CAL_FAILED:  stateStr = "failed";  break;
            default:               stateStr = "idle";    break;
        }
        char json[64];
        snprintf(json, sizeof(json), "{\"state\":\"%s\",\"calibrated\":%s}",
                 stateStr, imu_isCalibrated() ? "true" : "false");
        AsyncWebServerResponse* r = req->beginResponse(200, "application/json", json);
        r->addHeader("Cache-Control", "no-store");
        req->send(r);
    });

    // ── /debug_log GET|POST — SD debug-log (serial→SD mirror) enable state ───
    // Registered unconditionally (unlike the tee itself) so the dashboard always
    // gets a real JSON answer instead of a 404 — "supported":false is how the page
    // learns this firmware build wasn't compiled with DEBUG_SERIAL_TO_SD and hides
    // the row, rather than showing a checkbox that silently does nothing.
    webServer.on("/debug_log", HTTP_GET | HTTP_POST, [](AsyncWebServerRequest* req) {
#if defined(DEBUG_SERIAL_TO_SD) && DEBUG_SERIAL_TO_SD
        if (req->hasParam("en")) {
            DebugLog::setEnabled(req->getParam("en")->value() != "0");
        }
        char json[160];
        snprintf(json, sizeof(json), "{\"supported\":true,\"en\":%s,\"forced\":%s,\"file\":\"%s\"}",
                 DebugLog::isEnabled() ? "true" : "false",
                 DebugLog::isForced()  ? "true" : "false",
                 DebugLog::currentFileName());
        AsyncWebServerResponse* r = req->beginResponse(200, "application/json", json);
#else
        AsyncWebServerResponse* r = req->beginResponse(200, "application/json",
            "{\"supported\":false,\"en\":false,\"forced\":false,\"file\":\"\"}");
#endif
        r->addHeader("Cache-Control", "no-store");
        req->send(r);
    });

    // ── /gnss_config GET|POST — read / tune the elevation + C/N0 masks ────────
    // GET returns the current live values. POST with ele=/cnr= validates, then hands
    // off to the GNSS task, which writes the LG290P, PQTMSAVEPARs, and persists to NVS.
    // No module reset; masks take effect within a couple of epochs.
    webServer.on("/gnss_config", HTTP_GET | HTTP_POST, [](AsyncWebServerRequest* req) {
        bool  apply = false;
        float ele = gnss_getEleMask();
        float cnr = gnss_getCnrMask();
        int   ppp = (int)gnss_getPppMode();
        bool  pppChanged = false;
        if (req->hasParam("ele")) { ele = req->getParam("ele")->value().toFloat(); apply = true; }
        if (req->hasParam("cnr")) { cnr = req->getParam("cnr")->value().toFloat(); apply = true; }
        if (req->hasParam("ppp")) { ppp = req->getParam("ppp")->value().toInt();
                                    pppChanged = (ppp != (int)gnss_getPppMode()); apply = true; }
        if (apply) {
            bool ok = gnss_requestMasks(ele, cnr);
            if (ok) Serial.printf("🌐 GNSS masks requested: ele=%.1f° cnr=%.1f dB-Hz\n", ele, cnr);
            // Only touch PPP when the value actually CHANGED. Re-writing the same mode would
            // send a PQTMCFG write (and possibly bounce the nav engine) every time someone
            // taps Apply to nudge a mask — a needless fix drop. See gnss_requestPppMode().
            if (ok && pppChanged) {
                ok = gnss_requestPppMode((uint8_t)ppp);
                if (ok) Serial.printf("🌐 PPP mode requested: %d\n", ppp);
            }
            AsyncWebServerResponse* r = req->beginResponse(200, "application/json",
                ok ? "{\"ok\":true}" : "{\"ok\":false,\"err\":\"range\"}");
            r->addHeader("Cache-Control", "no-store");
            req->send(r); return;
        }
        char json[80];
        snprintf(json, sizeof(json), "{\"ele\":%.1f,\"cnr\":%.1f,\"ppp\":%u}",
                 gnss_getEleMask(), gnss_getCnrMask(), (unsigned)gnss_getPppMode());
        AsyncWebServerResponse* r = req->beginResponse(200, "application/json", json);
        r->addHeader("Cache-Control", "no-store");
        req->send(r);
    });

    // ── /sd/deleteold POST — bulk delete all non-active CSV files ────────────
    webServer.on("/sd/deleteold", HTTP_POST, [](AsyncWebServerRequest* req) {
        if (!sdlog_isReady()) {
            req->send(503, "text/plain", "SD not ready"); return;
        }
        if (g_activeDownloads > 0) {   // see single-delete guard above
            req->send(409, "text/plain", "Busy: a download is in progress"); return;
        }
        int deleted = 0;
        std::vector<String> removedOk;
        auto sdm = sdlog_getMutex();
        if (sdm && xSemaphoreTake(sdm, pdMS_TO_TICKS(1000)) == pdTRUE) {
            std::vector<String> toDelete;
            File dir = SD_MMC.open("/");
            if (dir) {
                int scanIdx = 0;
                while (true) {
                    File f = dir.openNextFile();
                    if (!f) break;
                    esp_task_wdt_reset();   // root scan runs in async_tcp — keep WDT fed
                    // Yield as well as feed: this scan can walk the whole card on
                    // the same task that has to keep other in-flight responses
                    // moving. See the /log/delete remove loop for the full reason.
                    if (scanIdx++ % 4 == 3) vTaskDelay(1);
                    String path = String(f.name()); f.close();
                    if (!path.endsWith(".csv")) continue;
                    if (!path.startsWith("/")) path = "/" + path;
                    // Skip ANY file open for writing — typed channels AND the canraw
                    // sniffer dump. Previously only the gps active name was skipped,
                    // so a bulk delete with the sniffer on would remove the open
                    // canraw file (FatFs corruption + reset).
                    String bare = path.substring(1);
                    if (sdlog_isActiveFile(bare.c_str())) continue;
                    toDelete.push_back(path);
                }
                dir.close();
            }
            for (size_t idx = 0; idx < toDelete.size(); idx++) {
                const String& p = toDelete[idx];
                esp_task_wdt_reset();       // and during the removes
                if (idx % 4 == 3) vTaskDelay(1);   // see the /log/delete loop
                if (SD_MMC.remove(p)) { deleted++; removedOk.push_back(p);
                                         Serial.printf("🗑️  SD bulk: %s\n", p.c_str()); }
                else                  { Serial.printf("⚠️  SD bulk fail: %s\n", p.c_str()); }
            }
            xSemaphoreGive(sdm);
        }
        // Snapshot updates AFTER sdMutex is released — see /log/<f> DELETE above
        // for why these two locks are never nested.
        for (auto& p : removedOk) sdlog_noteFileDeleted(p.c_str());
        char json[48]; snprintf(json, sizeof(json), "{\"deleted\":%d}", deleted);
        Serial.printf("🗑️  SD bulk-delete: %d file(s) removed\n", deleted);
        req->send(200, "application/json", json);
    });

    // ── /export/build POST — queue a tar build (runs in sdLogTask) ───────────
    // Body optionally carries names=a.csv,b.csv,... to build only that set;
    // omitted or empty builds every .csv on the card ("download all"). It may
    // also carry file=<name>.tar to name the download, which is how a session
    // archive gets a stamp in the browser's own timezone — the device clock is
    // GPS UTC and has no idea what timezone the operator is standing in.
    webServer.on("/export/build", HTTP_POST, [](AsyncWebServerRequest* req) {
        if (!sdlog_isReady()) { req->send(503, "text/plain", "SD not ready"); return; }
        String names;   // kept alive through the call below; sdlog_requestExport copies it
        if (req->hasParam("names", true)) names = req->getParam("names", true)->value();
        exportFileName[0] = '\0';
        if (req->hasParam("file", true)) setExportFileName(req->getParam("file", true)->value());
        sdlog_requestExport(names.length() ? names.c_str() : nullptr);
        AsyncWebServerResponse* r = req->beginResponse(200, "application/json", "{\"building\":true}");
        r->addHeader("Cache-Control", "no-store");
        req->send(r);
    });

    // ── /export/status GET — poll the tar build (0 idle/1 building/2 ready/3 err)
    webServer.on("/export/status", HTTP_GET, [](AsyncWebServerRequest* req) {
        int st = 0; uint64_t sz = 0; int cnt = 0;
        sdlog_getExportState(&st, &sz, &cnt);
        const char* s = (st == 2) ? "ready" : (st == 1) ? "building" : (st == 3) ? "error" : "idle";
        char json[96];
        snprintf(json, sizeof(json), "{\"state\":\"%s\",\"size\":%llu,\"count\":%d}",
                 s, (unsigned long long)sz, cnt);
        AsyncWebServerResponse* r = req->beginResponse(200, "application/json", json);
        r->addHeader("Cache-Control", "no-store");
        req->send(r);
    });

    // ── /export.tar GET — stream the prebuilt archive ────────────────────────
    // Streams ONE existing file via the same WDT-fed chunk reader as /log/* — the
    // multi-file scan/assembly already happened in sdLogTask, so nothing heavy
    // runs in this callback.
    webServer.on("/export.tar", HTTP_GET, [](AsyncWebServerRequest* req) {
        int st = 0; sdlog_getExportState(&st, nullptr, nullptr);
        if (st != 2) { req->send(409, "text/plain", "Archive not ready — build it first"); return; }
        if (!sdlog_isPaused()) {   // same FatFs lock contention as /log/<f> — see there
            req->send(409, "text/plain",
                "Pause logging before downloading the archive.");
            return;
        }
        if (g_activeDownloads > 0) {   // one transfer at a time — see /log/<f>
            req->send(409, "text/plain",
                "Busy: another download is already in progress.");
            return;
        }
        struct DlState {
            File file;
            DlState()  { g_activeDownloads++; sdlog_downloadBegin(); }
            ~DlState() { if (file) file.close(); g_activeDownloads--; sdlog_downloadEnd(); }
        };
        auto state = std::make_shared<DlState>();
        state->file = SD_MMC.open(sdlog_getExportPath(), FILE_READ);
        if (!state->file) { req->send(404, "text/plain", "Archive missing"); return; }
        Serial.printf("🌐 Download: %s (%lu B)\n",
                      sdlog_getExportPath(), (unsigned long)state->file.size());
        req->client()->setNoDelay(true);  // see the /log/<f> handler for why
        auto* resp = req->beginChunkedResponse("application/x-tar",
            [state](uint8_t* buf, size_t maxLen, size_t index) -> size_t {
                esp_task_wdt_reset();
                if (!state->file) return 0;
                if ((uint32_t)state->file.position() != (uint32_t)index)
                    state->file.seek((uint32_t)index);
                return state->file.read(buf, maxLen);
            });
        String disposition = "attachment; filename=\"";
        if (exportFileName[0])          disposition += exportFileName;
        else if (sdlog_exportIsSubset()) disposition += "rcx_logs_selected.tar";
        else                             disposition += "rcx_logs.tar";
        disposition += "\"";
        resp->addHeader("Content-Disposition", disposition);
        resp->addHeader("Cache-Control", "no-store");
        req->send(resp);
    });

    // ── /can/sniff POST — toggle CAN diagnostic/sniffer mode ─────────────────
    webServer.on("/can/sniff", HTTP_POST, [](AsyncWebServerRequest* req) {
        if (req->hasParam("on")) {
            bool on = req->getParam("on")->value() != "0";
            // EXCLUSIVITY (default, for stability): the canraw sniffer logs EVERY
            // frame on the bus (500 kbps → thousands/sec). Stacked on gps/can/imu/
            // sat writes that can outrun the SD card's sustained rate and back up
            // queues until frames drop. It is NOT a corruption risk — the branches
            // and queues are disjoint — only a throughput risk. So by default we
            // still force the log channels off when the sniffer turns on.
            //
            // force=1 OVERRIDE: keep the logging channels running alongside the
            // sniffer. Used to capture GPS/IMU/CAN and raw frames at once for
            // vehicle-dynamics channel discovery. The web UI gates this behind a
            // confirm dialog that spells out the SD-throughput trade-off, so the
            // user is accepting possible dropped frames knowingly. Watch the
            // sniffer 'drops'/'overflow' counters (/can/snapshot) during such runs.
            bool force = req->hasParam("force") && req->getParam("force")->value() != "0";
            if (on && !force) sdlog_setConfig(false, false, false, false);
            can_setSniffer(on);
        }
        char json[24];
        snprintf(json, sizeof(json), "{\"sniffer\":%s}", can_getSniffer() ? "true" : "false");
        AsyncWebServerResponse* r = req->beginResponse(200, "application/json", json);
        r->addHeader("Cache-Control", "no-store");
        req->send(r);
    });

    // ── /can/snapshot GET — live snapshot of every CAN ID seen ───────────────
    // {sniffer,overflow,drops,ids:[{id,dlc,count,age,data:[8]}]}. Reads a copy
    // of the snapshot table under the sniffer mutex (see can_bus.cpp) — bounded
    // and never blocks the CAN task.
    webServer.on("/can/snapshot", HTTP_GET, [](AsyncWebServerRequest* req) {
        std::string json = "{\"sniffer\":";
        json += can_getSniffer() ? "true" : "false";
        if (can_getSniffer()) {
            CanSniffEntry* snap = (CanSniffEntry*)malloc(128 * sizeof(CanSniffEntry));
            int n = snap ? can_getSniffSnapshot(snap, 128) : 0;
            uint32_t now = millis();
            json += ",\"overflow\":" + std::to_string((unsigned)can_getSniffOverflow());
            json += ",\"drops\":"    + std::to_string((unsigned)sdlog_getCanRawDrops());
            json += ",\"ids\":[";
            for (int i = 0; i < n; i++) {
                if (i) json += ',';
                json += "{\"id\":"    + std::to_string((unsigned)snap[i].id);
                json += ",\"dlc\":"   + std::to_string((unsigned)snap[i].dlc);
                json += ",\"count\":" + std::to_string((unsigned)snap[i].count);
                json += ",\"age\":"   + std::to_string((unsigned)(now - snap[i].lastMs));
                json += ",\"data\":[";
                for (int b = 0; b < 8; b++) { if (b) json += ','; json += std::to_string((unsigned)snap[i].data[b]); }
                json += "]}";
            }
            json += "]";
            if (snap) free(snap);
        } else {
            json += ",\"overflow\":0,\"drops\":0,\"ids\":[]";
        }
        json += "}";
        webSendBody(req, "application/json", std::move(json));
    });

    // ── WiFi station list ────────────────────────────────────────────────────
    // Stored passphrases are NEVER returned by any route here. They go to the
    // radio and nowhere else; the dashboard can set one but can't read one back.
    //
    // The mutating routes commit to NVS, and a flash commit briefly disables the
    // instruction cache on both cores — which stalls the GNSS UART drain. That
    // is acceptable for a deliberate operator action in the paddock, but it is
    // the reason none of this is called automatically or on a timer.
    webServer.on("/wifi", HTTP_GET, [](AsyncWebServerRequest* req) {
        String j = "{\"ap\":\"" + String(wifi_deviceName()) + "\",\"apip\":\""
                 + String(wifi_apIp()) + "\",\"active\":\"";
        if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
            if (status.wifiConnected) j += status.wifiSSID;
            xSemaphoreGive(dataMutex);
        }
        j += "\",\"max\":" + String(WIFI_MAX_NETWORKS) + ",\"nets\":[";
        const int n = wifi_count();
        for (int i = 0; i < n; i++) {
            char ssid[33] = "";
            if (!wifi_getSsid(i, ssid, sizeof(ssid))) continue;
            const String s = jsonEscape(ssid);
            if (i) j += ',';
            j += "{\"ssid\":\"" + s + "\",\"on\":" + (wifi_getEnabled(i) ? "true" : "false") + "}";
        }
        j += "]}";
        auto* r = req->beginResponse(200, "application/json", j);
        r->addHeader("Cache-Control", "no-store");
        req->send(r);
    });

    webServer.on("/device/name", HTTP_POST, [](AsyncWebServerRequest* req) {
        if (!req->hasParam("name", true)) { req->send(400, "text/plain", "missing name"); return; }
        const String name = req->getParam("name", true)->value();
        // The name is also the AP's SSID, so it inherits the 802.11 length limit.
        if (!wifi_setDeviceName(name.c_str())) {
            req->send(400, "text/plain", "name must be 1-32 characters");
            return;
        }
        // The reply must start with "Renamed" — the dashboard keys the field-clear
        // confirmation off that prefix.
        req->send(200, "text/plain",
                  "Renamed. The configuration WiFi network takes the new name "
                  "after a restart.");
    });

    webServer.on("/wifi/add", HTTP_POST, [](AsyncWebServerRequest* req) {
        if (!req->hasParam("ssid", true)) { req->send(400, "text/plain", "missing ssid"); return; }
        const String ssid = req->getParam("ssid", true)->value();
        const String pass = req->hasParam("pass", true) ? req->getParam("pass", true)->value() : String();
        if (ssid.length() == 0)  { req->send(400, "text/plain", "empty ssid"); return; }
        if (ssid.length() > 32)  { req->send(400, "text/plain", "ssid too long"); return; }
        if (pass.length() > 64)  { req->send(400, "text/plain", "password too long"); return; }
        // A passphrase is either absent (open network) or at least WPA2's 8-char
        // minimum. Anything between is a typo that would fail silently at
        // association time, which is far harder to diagnose than a rejection.
        if (pass.length() > 0 && pass.length() < 8) {
            req->send(400, "text/plain", "password must be blank or at least 8 characters");
            return;
        }
        if (!wifi_add(ssid.c_str(), pass.c_str())) {
            req->send(507, "text/plain", "list full");
            return;
        }
        req->send(200, "text/plain", "Saved. Applies on the next reconnect.");
    });

    webServer.on("/wifi/remove", HTTP_POST, [](AsyncWebServerRequest* req) {
        if (!req->hasParam("i", true)) { req->send(400, "text/plain", "missing i"); return; }
        const int i = req->getParam("i", true)->value().toInt();
        // Call exactly once — wifi_remove() shifts the list, so a second call
        // with the same index would delete a different, innocent entry.
        const bool ok = wifi_remove(i);
        req->send(ok ? 200 : 404, "text/plain", ok ? "Removed." : "No such entry.");
    });

    webServer.on("/wifi/enable", HTTP_POST, [](AsyncWebServerRequest* req) {
        if (!req->hasParam("i", true) || !req->hasParam("on", true)) {
            req->send(400, "text/plain", "missing i/on");
            return;
        }
        const int  i  = req->getParam("i", true)->value().toInt();
        const bool on = (req->getParam("on", true)->value() == "1");
        req->send(wifi_setEnabled(i, on) ? 200 : 404, "text/plain", "ok");
    });

    webServer.on("/wifi/move", HTTP_POST, [](AsyncWebServerRequest* req) {
        if (!req->hasParam("i", true) || !req->hasParam("d", true)) {
            req->send(400, "text/plain", "missing i/d");
            return;
        }
        const int i = req->getParam("i", true)->value().toInt();
        const int d = req->getParam("d", true)->value().toInt();
        req->send(wifi_move(i, d) ? 200 : 400, "text/plain", "ok");
    });

    // ── NTRIP caster list ────────────────────────────────────────────────────
    // Usernames are listed so the operator can tell accounts apart; caster
    // passwords are never returned, for the same reason WiFi passphrases aren't.
    // ── CAN database (DBC) storage ────────────────────────────────────────────
    // Every handler here is storage-only and touches no decode state. None of
    // them reach the SD card directly: the card belongs to dbcTask, because
    // doing filesystem work in the async_tcp task overran the Task-WDT.
    webServer.on("/dbc", HTTP_GET, [](AsyncWebServerRequest* req) {
        DbcFileEntry files[DBC_MAX_FILES];
        int n = dbc_getSnapshot(files, DBC_MAX_FILES);

        String j = "{\"active\":\"";
        j += jsonEscape(dbc_getActive());
        j += "\",\"scanning\":";
        j += dbc_scanPending() ? "true" : "false";
        j += ",\"status\":\"";
        j += jsonEscape(dbc_lastStatus());
        j += "\",\"audited\":\"";
        j += jsonEscape(dbc_getAudited());
        j += "\",\"audit\":\"";
        j += jsonEscape(dbc_auditText());
        j += "\",\"files\":[";
        for (int i = 0; i < n; i++) {
            if (i) j += ",";
            j += "{\"name\":\"";
            j += jsonEscape(files[i].name);
            j += "\",\"size\":";
            j += String(files[i].size);
            j += "}";
        }
        j += "]}";
        req->send(200, "application/json", j);
    });

    // The upload handler streams into the PSRAM staging buffer. The completion
    // callback fires after the final chunk, so the reply reports what the store
    // accepted rather than what the browser sent.
    webServer.on("/dbc/upload", HTTP_POST,
        [](AsyncWebServerRequest* req) {
            req->send(200, "text/plain", dbc_lastStatus());
        },
        [](AsyncWebServerRequest* req, const String& filename, size_t index,
           uint8_t* data, size_t len, bool final) {
            if (index == 0 && !dbc_uploadBegin(filename.c_str())) return;
            if (len && !dbc_uploadChunk(data, len))               return;
            if (final) dbc_uploadEnd();
        });

    webServer.on("/dbc/select", HTTP_POST, [](AsyncWebServerRequest* req) {
        String n = req->hasParam("name", true) ? req->getParam("name", true)->value() : String();
        if (!dbc_setActive(n.c_str())) { req->send(400, "text/plain", "Invalid filename."); return; }
        req->send(200, "text/plain", dbc_lastStatus());
    });

    webServer.on("/dbc/delete", HTTP_POST, [](AsyncWebServerRequest* req) {
        String n = req->hasParam("name", true) ? req->getParam("name", true)->value() : String();
        if (!dbc_requestDelete(n.c_str())) { req->send(400, "text/plain", "Invalid filename or busy."); return; }
        req->send(200, "text/plain", "Deleting\u2026");
    });

    webServer.on("/casters", HTTP_GET, [](AsyncWebServerRequest* req) {
        String j = "{\"max\":" + String(NTRIP_MAX_CASTERS) + ",\"list\":[";
        const int n = ntrip_casterCount();
        for (int i = 0; i < n; i++) {
            NtripCasterInfo c;
            if (!ntrip_casterInfo(i, &c)) continue;
            if (i) j += ',';
            j += "{\"host\":\"" + jsonEscape(c.host)
               + "\",\"port\":\"" + jsonEscape(c.port)
               + "\",\"mount\":\"" + jsonEscape(c.mount)
               + "\",\"on\":" + (c.enabled ? "true" : "false")
               + ",\"def\":" + (c.isDefault ? "true" : "false")
               + ",\"active\":" + (c.active ? "true" : "false") + "}";
        }
        j += "]}";
        auto* r = req->beginResponse(200, "application/json", j);
        r->addHeader("Cache-Control", "no-store");
        req->send(r);
    });

    webServer.on("/casters/enable", HTTP_POST, [](AsyncWebServerRequest* req) {
        if (!req->hasParam("i", true) || !req->hasParam("on", true)) {
            req->send(400, "text/plain", "missing i/on");
            return;
        }
        const int  i  = req->getParam("i", true)->value().toInt();
        const bool on = (req->getParam("on", true)->value() == "1");
        req->send(ntrip_setCasterEnabled(i, on) ? 200 : 404, "text/plain", "ok");
    });

    webServer.on("/casters/add", HTTP_POST, [](AsyncWebServerRequest* req) {
        if (!req->hasParam("host", true)) { req->send(400, "text/plain", "missing host"); return; }
        const String host  = req->getParam("host", true)->value();
        const String port  = req->hasParam("port",  true) ? req->getParam("port",  true)->value() : String("2101");
        const String user  = req->hasParam("user",  true) ? req->getParam("user",  true)->value() : String();
        const String pass  = req->hasParam("pass",  true) ? req->getParam("pass",  true)->value() : String();
        const String mount = req->hasParam("mount", true) ? req->getParam("mount", true)->value() : String();
        if (host.length() == 0 || host.length() > 47) {
            req->send(400, "text/plain", "host must be 1-47 characters");
            return;
        }
        if (!ntrip_saveCaster(host.c_str(),
                              port.length() ? port.c_str() : "2101",
                              user.c_str(), pass.c_str(), mount.c_str())) {
            req->send(507, "text/plain", "caster list full");
            return;
        }
        req->send(200, "text/plain", "Saved. Caster selection restarts now.");
    });

    webServer.on("/casters/reset", HTTP_POST, [](AsyncWebServerRequest* req) {
        // Raises a flag consumed inside ntrip_loop() on wifiNtripTask — no
        // NTRIP state is touched from this (async_tcp) task.
        ntrip_requestReset();
        req->send(200, "text/plain", "Timers cleared. Reselecting now.");
    });

    webServer.on("/casters/remove", HTTP_POST, [](AsyncWebServerRequest* req) {
        if (!req->hasParam("i", true)) { req->send(400, "text/plain", "missing i"); return; }
        const int i = req->getParam("i", true)->value().toInt();
        // Call once: the list is rebuilt around the removal, so a repeat call
        // with the same index would target a different caster.
        const bool ok = ntrip_removeCaster(i);
        req->send(ok ? 200 : 400, "text/plain",
                  ok ? "Removed." : "Built-in casters can be disabled but not removed.");
    });

    webServer.onNotFound([](AsyncWebServerRequest* req) {
        Serial.printf("🌐 404: %s %s\n",
                      req->method() == HTTP_GET ? "GET" : "OTHER", req->url().c_str());
        req->send(404, "text/plain", "Not found");
    });

    Serial.println("🌐 Web routes registered");
}

void webserver_begin() {
    if (!webBegun) {
        webServer.begin();
        webBegun = true;
        Serial.printf("🌐 Web server started: http://%s:%d/\n",
                      WiFi.localIP().toString().c_str(), WEBSERVER_PORT);
    }
}

#else
void webserver_init()  {}
void webserver_begin() {}
#endif
