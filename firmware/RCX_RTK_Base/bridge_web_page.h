#pragma once
/*
 * bridge_web_page.h — dashboard UI shell (served from PROGMEM)
 * ===========================================================
 * STABILITY: this is a STATIC asset sent straight from flash by rcxSendStaticPage()
 * (bounded pieces, GNSS UART drained between them) — it is never rebuilt per request,
 * so it adds zero heap churn. Every KB added here is more time on the wire. All live
 * data arrives via the small /api/status JSON poll; all actions are tiny POSTs.
 * EDITING THIS PAGE: regenerate bridge_web_page_gz.h in the same change
 * (python3 qa/qa/pagegz.py <firmware dir>). Browsers that accept gzip are served that
 * compressed copy. The QA gate fails while it is stale, and the build stops when its
 * recorded page length differs from this page's.
 * Keep it dependency-free (vanilla JS, no libraries or CDNs). The one external fetch
 * is the Esri satellite tile behind the position thumbnails, loaded by the phone's
 * browser only when the phone has internet; the device never fetches it.
 *
 * API CONTRACT (this file is the CLIENT; the .ino defines the SERVER):
 *   GET  /api/status   → one JSON object polled ~1.5 s. Field names here MUST match
 *                        the snprintf format strings in handleApiStatus() exactly.
 *                        If you add/rename a field on one side, change the other.
 *   GET  /api/wifilist → { networks:[{ssid,active}], ... } (passwords never sent)
 *   POST /api/addwifi  /api/delwifi      — body: ssid=&pw=  /  ssid=
 *   POST /api/casteradd/api/casterdel    — body: host=&port=&mount=&pw=  /  host=&mount=
 *   POST /api/casteren — body: host=&mount=&en=0|1   (enable/disable a caster)
 *   POST /api/setpos /api/clearpos       — manual base position entry / clear
 *   POST /api/config   — body: time=&acc=  (survey duration / accuracy limit)
 *   POST /api/setname  — body: name=…      (rename this device; applies on reboot)
 *   POST /api/log      — body: ch=&on=     (toggle an SD log channel or the display)
 *   POST /api/force    — lock the running survey's average now (409 outside a survey)
 *   POST /api/reconfig — force a full module reconfigure + reset (defers to loop())
 *   POST /api/query    — re-poll module config (debug)
 *   GET  /logs.json, /dl, /rtcm.raw, /caster0|1/raw — log listing/download/debug
 *   GET  /dl?f=rcx_logbook.csv — the card's running logbook (session times, saved positions)
 *   WiFi networks and casters are added from the dashboard and stored in NVS;
 *   they can be deleted or disabled from there.
 *
 * STABILITY NOTE (matches the .ino's top-of-file rule): do not rename DOM ids,
 * card titles, or visible strings unless asked — field workflows depend on them.
 */

static const char BRIDGE_INDEX_HTML[] PROGMEM = R"HTML(<!DOCTYPE html>
<html lang="en"><head>
<meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>RCX RTK Base</title>
<style>
:root{--bg:#f2f4f7;--card:#ffffff;--line:#c3c9d4;--ink:#0a0c10;--mut:#4a5260;
--good:#00661a;--warn:#8a4b00;--bad:#b00020;--accent:#0b4fa8;
--goodbg:#dff3e3;--warnbg:#fbeed8;--badbg:#fadfe3}
*{box-sizing:border-box}
/* SUNLIGHT READABILITY: a phone held outdoors loses a dark theme entirely — the screen
   cannot outrun ambient light, so dark text on a near-white ground is what stays legible.
   Every colour here is >= 4.5:1 on both the page and card backgrounds (WCAG AA), checked
   numerically; status is carried by text and a coloured chip, never hue alone, so it
   still reads in glare and for colour-blind users. Weight is bumped because thin strokes
   are the first thing to disappear in bright light. */
body{margin:0;font:15px/1.5 system-ui,sans-serif;background:var(--bg);color:var(--ink);
-webkit-text-size-adjust:100%;font-weight:500}
header{display:flex;align-items:center;gap:10px;padding:14px 18px;border-bottom:1px solid var(--line);position:sticky;top:0;background:var(--bg);z-index:5}
header h1{font-size:16px;margin:0;font-weight:600}
.dot{width:10px;height:10px;border-radius:50%;background:var(--bad);box-shadow:0 0 8px currentColor}
.dot.on{background:var(--good)}
.wrap{max-width:1100px;margin:0 auto;padding:16px;display:grid;gap:14px;grid-template-columns:repeat(auto-fit,minmax(300px,1fr))}
.card{background:var(--card);border:1px solid var(--line);border-radius:10px;padding:14px;
border-left:6px solid var(--line);transition:border-color .2s,background .2s}
/* CARD-LEVEL STOPLIGHT: the left edge and a faint wash carry each section's state, so
   the page can be triaged at arm's length in sunlight without reading any values. The
   wash stays pale so text contrast is unaffected; the edge is what actually reads. */
.card.s-good{border-left-color:var(--good);background:#f4fcf6}
.card.s-warn{border-left-color:var(--warn);background:var(--warnbg)}
.card.s-bad{border-left-color:var(--bad);background:var(--badbg)}
.card h2{margin:0 0 10px;font-size:12px;letter-spacing:.08em;text-transform:uppercase;color:var(--accent);font-weight:700}
.big{font-size:34px;font-weight:700;font-variant-numeric:tabular-nums}
.row{display:flex;justify-content:space-between;gap:8px;padding:3px 0;border-bottom:1px dashed var(--line)}
.row:last-child{border-bottom:0}
.row .k{color:var(--mut)}
.row .v{font-variant-numeric:tabular-nums;text-align:right;font-weight:600}
/* STOPLIGHT CHIPS: status values get a tinted pill, not just coloured text. The fill is
   what carries at arm's length in sunlight, and it keeps the state legible when glare
   washes out hue differences. */
.good,.warn,.bad,.hot{font-weight:700;border-radius:5px;padding:1px 7px}
.good{color:var(--good);background:var(--goodbg)}
.warn,.hot{color:var(--warn);background:var(--warnbg)}
.bad{color:var(--bad);background:var(--badbg)}
.pill{display:inline-block;padding:2px 8px;border-radius:20px;font-size:12px;background:#222836}
button,.btn{font:inherit;background:var(--accent);color:#fff;border:0;border-radius:6px;padding:8px 12px;cursor:pointer;text-decoration:none;display:inline-block}
button.sec,.btn.sec{background:#e3e7ee;color:var(--ink);border:1px solid var(--line)}
button:active{transform:translateY(1px)}
input{font:inherit;background:#fff;color:var(--ink);border:1px solid #8d95a3;border-radius:6px;padding:8px;width:100%}
input:focus{outline:2px solid var(--accent);outline-offset:1px}
label.sw{display:flex;align-items:center;justify-content:space-between;gap:10px;padding:6px 0;border-bottom:1px dashed var(--line)}
label.sw:last-child{border-bottom:0}
.tg{width:44px;height:24px;border-radius:20px;background:#9aa2b1;position:relative;cursor:pointer;transition:.15s;flex:0 0 auto;border:1px solid #6b7280}
.tg.on{background:var(--good)}
.tg::after{content:"";position:absolute;top:2px;left:2px;width:20px;height:20px;border-radius:50%;background:#fff;transition:.15s}
.tg.on::after{left:22px}
.grid2{display:grid;grid-template-columns:1fr 1fr;gap:8px}
pre{background:#0a0c11;border:1px solid var(--line);border-radius:6px;padding:8px;margin:6px 0 0;overflow-x:auto;white-space:pre-wrap;word-break:break-all;font-size:12px;color:#a9ffcf}
.files{max-height:180px;overflow:auto}
/* The log list grows in place. Capping it made a long session's files reachable only
   through a 180px window, which is the one place on this page where the whole list is
   the point — every entry is a file the operator may want to pull off the device.
   max-height is restated because .files sets it and overflow alone would not clear it:
   the cap would survive and the list would simply be clipped instead of scrolled. */
.filelist{max-height:none;overflow:visible}
/* Sized to show a full base stream without scrolling: four MSM observation types, the
   station and antenna descriptors, and all four ephemeris types (~11 rows), with room
   to spare. Still scrolls if a stream carries more. */
.types{max-height:400px;overflow:auto}
.files a{color:var(--accent);text-decoration:none}
.muted{color:var(--mut);font-size:12.5px}
.flex{display:flex;gap:8px;flex-wrap:wrap}
</style></head><body>
<header><span id="dot" class="dot"></span><h1 id="dev">RCX RTK Base</h1><button type="button" class="btn sec" style="padding:2px 8px;margin-left:8px;font-size:11px" onclick="renameDevice()" title="Rename this device">Rename</button><span id="up" class="muted"></span></header>
<div class="wrap">

  <div class="card" id="cardSvin">
    <h2 id="svinTitle">Survey-In</h2>
    <div class="big" id="svinTimer">--:--</div>
    <div class="muted" id="svinState">—</div>
    <div class="row"><span class="k">Reference source</span><span class="v" id="svinSrc">—</span></div>
    <div class="row"><span class="k">Valid</span><span class="v" id="svinValid">—</span></div>
    <div class="row"><span class="k">Accuracy (mean)</span><span class="v" id="svinAcc">—</span></div>
    <div class="row" id="svinObsRow"><span class="k">Observations</span><span class="v" id="svinObs">—</span></div>
    <div class="row"><span class="k">PPP (module-reported)</span><span class="v" id="pppSup">—</span></div>
    <div class="row"><span class="k">Galileo E6 satellites</span><span class="v" id="pppE6">—</span></div>
    <div class="row"><span class="k">Module PPP solution</span><span class="v" id="pppNav">—</span></div>
    <div class="muted" id="pppNote" style="margin-top:4px"></div>
  </div>

  <div class="card" id="cardPos">
    <h2>Position & Satellites</h2>
    <div class="row"><span class="k">Fix</span><span class="v" id="fix">—</span></div>
    <div class="row"><span class="k">Satellites</span><span class="v big" id="sats" style="font-size:22px">—</span></div>
    <div class="row"><span class="k">HDOP</span><span class="v" id="hdop">—</span></div>
    <div class="row"><span class="k">Est. error</span><span class="v" id="epe">—</span></div>
    <div class="row"><span class="k">Lat / Lon</span><span class="v" id="ll">—</span></div>
    <div class="row"><span class="k">Alt (MSL)</span><span class="v" id="alt">—</span></div>
    <div class="row"><span class="k">Alt (ellipsoidal)</span><span class="v" id="altEll">—</span></div>
    <div class="row"><span class="k">Verify on a map</span><span class="v"><a id="llMap" href="#" target="_blank" rel="noopener noreferrer">—</a></span></div>
    <canvas id="mapLive" width="256" height="256" style="display:none;max-width:100%;margin-top:6px;border-radius:6px"></canvas>
    <div class="muted" id="mapLiveMsg" style="font-size:12px"></div>
  </div>

  <div class="card" id="cardTemp">
    <h2>Temperature</h2>
    <div class="row"><span class="k">CPU (ESP32)</span><span class="v" id="chipT">—</span></div>
    <div class="row"><span class="k">IMU (QMI8658)</span><span class="v" id="boardT">—</span></div>
    <div class="muted" id="tempNote" style="margin-top:6px">green &lt; warm &lt; hot &lt; danger · IMU reads heat-soak in the trapped-air pocket under the SD card — not case ambient</div>
  </div>

  <div class="card" id="cardRtcm">
    <h2>RTCM Stream</h2>
    <div class="row"><span class="k">Corrections Generated</span><span class="v" id="obs">—</span></div>
    <div class="row"><span class="k">Frames/s</span><span class="v" id="fps">—</span></div>
    <div class="row"><span class="k">Bytes/s</span><span class="v" id="bps">—</span></div>
    <div class="row"><span class="k">CRC-valid frames</span><span class="v" id="vframes">—</span></div>
    <div class="row"><span class="k">Candidate / CRC-fail / frame-fail</span><span class="v" id="vfail">—</span></div>
    <div class="row"><span class="k">Stream completeness</span><span class="v" id="rtcmMissing">—</span></div>
    <div class="row"><span class="k">Antenna descriptor</span><span class="v" id="antDesc">—</span></div>
    <div class="row"><span class="k">Publishable</span><span class="v" id="qual">—</span></div>
    <div class="muted" id="qualWhy" style="margin-top:4px"></div>
    <div class="muted" style="margin:8px 0 4px">Message types (name · count · rate)</div>
    <div id="types" class="types"><span class="muted">—</span></div>
  </div>

  <div class="card" id="castersCard">
    <h2>Casters</h2>
    <div id="casters"></div>
    <div class="muted" style="margin:12px 0 4px;border-top:1px solid var(--line);padding-top:10px">Add a caster</div>
    <div class="row"><span class="k">Host</span><input id="cHost" type="text" placeholder="rtk2go.com" autocomplete="off" oninput="mountCheckSoon()" style="flex:1;min-width:0"></div>
    <div class="row"><span class="k">Port</span><input id="cPort" type="number" value="2101" style="flex:1;min-width:0"></div>
    <div class="row"><span class="k">Mount</span><input id="cMount" type="text" placeholder="ABCD" autocomplete="off" maxlength="16" oninput="mountCheckSoon()" style="flex:1;min-width:0"></div>
    <div class="muted" id="mountWarn" style="margin:4px 0 0"></div>
    <div class="muted" style="margin:6px 0 0">
      Centipede: <a href="https://centipede.fr" target="_blank" rel="noopener">centipede.fr</a> &middot;
      <a href="https://docs.centipede.fr" target="_blank" rel="noopener">docs.centipede.fr</a><br>
      RTK2go: <a href="http://rtk2go.com" target="_blank" rel="noopener">rtk2go.com</a>
    </div>
    <div class="row"><span class="k">Password</span><span style="flex:1;min-width:0;display:flex;gap:6px">
      <input id="cPw" type="text" placeholder="mount password" autocomplete="off" style="flex:1;min-width:0">
      <button type="button" id="cPwBtn" class="btn sec" style="padding:2px 10px;flex:0 0 auto" title="Show/hide password" onclick="pwToggle(this,'cPw')">Hide</button>
    </span></div>
    <div class="flex" style="margin-top:8px"><button class="btn" onclick="casterAdd()">Add Caster</button></div>
    <div class="muted" id="casterMsg" style="margin-top:6px"></div>
  </div>

  <div class="card" id="cardWifi">
    <h2>WiFi Networks</h2>
    <div class="row"><span class="k">Uplink</span><span id="wfState">…</span></div>
    <div id="wifiList" class="files" style="margin-bottom:10px"><span class="muted">Loading…</span></div>
    <div class="muted" style="margin:10px 0 4px">Add a network</div>
    <div class="row"><span class="k">SSID</span><input id="wfSsid" type="text" placeholder="NetworkName" autocomplete="off" style="flex:1;min-width:0"></div>
    <div class="row"><span class="k">Password</span><span style="flex:1;min-width:0;display:flex;gap:6px">
      <input id="wfPw" type="password" placeholder="password (blank = open network)" autocomplete="new-password" style="flex:1;min-width:0">
      <button type="button" id="wfPwBtn" class="btn sec" style="padding:2px 10px;flex:0 0 auto" title="Show/hide password" onclick="pwToggle(this,'wfPw')">Show</button>
    </span></div>
    <div class="flex" style="margin-top:8px">
      <button class="btn sec" onclick="addWifi(false)">Add</button>
      <button class="btn" onclick="addWifi(true)">Add &amp; Connect</button>
    </div>
    <div class="muted" id="wifiMsg" style="margin-top:6px"></div>
  </div>

  <div class="card">
    <h2>Base Configuration</h2>
    <div class="grid2">
      <div><div class="muted">Survey time (hh:mm:ss)</div><input id="cfgTime" type="text" inputmode="numeric" placeholder="01:00:00" autocomplete="off"></div>
      <div><div class="muted">Accuracy limit (m)</div><input id="cfgAcc" type="number" min="0.1" step="0.1"></div>
    </div>
    <div class="flex" style="margin-top:10px">
      <button onclick="applyCfg(0)">Apply (keep surveying)</button>
      <button class="sec" onclick="applyCfg(1)">Apply &amp; restart survey</button>
      <button class="sec" onclick="act('/api/reconfig',null,'New survey started')">Start new survey</button>
      <button class="sec" id="btnLockNow" disabled title="Available while a survey is averaging" onclick="act('/api/force',null,'Locking survey')">Lock survey now</button>
      <button class="sec" onclick="act('/api/reconfig',{full:1},'Full reconfigure')">Full reconfigure</button>
      <button class="sec" onclick="act('/api/query',null,'Queried module')">Read module config</button>
    </div>
    <div class="row" style="margin-top:8px"><span class="k">Receiver reports</span><span class="v" id="modCfg">—</span></div>
    <div class="muted" style="margin-top:8px">
      <b>Apply (keep surveying)</b> — retargets the window and accuracy limit immediately. The survey keeps every observation it has already collected; the receiver is not touched.<br>
      <b>Apply &amp; restart survey</b> — same values, but discards collected observations and starts the window over. Still no receiver reset.<br>
      <b>Start new survey</b> — discards the current survey and any saved coordinate in use, and begins a fresh one immediately.<br>
      <b>Lock survey now</b> — ends the running survey early and locks the average collected so far, with the same checks as a survey that runs its full window. Available only while a survey is averaging; the coordinate is only as good as the time averaged. Until converged PPP has run a fifth as long as the averaging that did not converge (at least one minute; thirty minutes always suffices) and its two halves agree, the coordinate is the average of every fix, possibly metres from the antenna, and is saved and shown as UNCONVERGED with its accuracy; the PPP card says which rule applies and why. To change the coordinate of a running base, use manual position entry.<br>
      <b>Full reconfigure</b> — rewrites the receiver's entire configuration (message set, rates, protocol) and resets it. Takes about 8 s and forces a cold re-acquisition, so corrections stop until it reacquires. Only needed if the module's configuration is in doubt.<br>
      <b>Read module config</b> — asks the receiver what mode and survey settings it currently holds, and shows them above. Read-only; changes nothing.
    </div>
    <div class="muted" id="cfgNote" style="margin-top:8px"></div>
  </div>

  <div class="card">
    <h2>Logging Channels</h2>
    <label class="sw"><span>GPS (gps_*.csv)</span><span class="tg" data-ch="gps"></span></label>
    <label class="sw"><span>Satellites (sat_*.csv)</span><span class="tg" data-ch="sat"></span></label>
    <label class="sw"><span>RTCM light / status (status_*.csv)</span><span class="tg" data-ch="status"></span></label>
    <label class="sw"><span>RTCM detailed (rtcm_*.csv)</span><span class="tg" data-ch="rtcm"></span></label>
    <label class="sw"><span>RTCM raw capture (.bin)</span><span class="tg" data-ch="raw"></span></label>
    <label class="sw"><span>Diagnostic events (event_*.csv)</span><span class="tg" data-ch="event"></span></label>
    <label class="sw"><span>On-device telemetry display</span><span class="tg" data-ch="disp"></span></label>
    <div class="muted" id="logNote" style="margin-top:8px"></div>
  </div>

  <div class="card">
    <h2>Latest Messages</h2>
    <div class="muted">Last PQTM</div><pre id="mPqtm">—</pre>
    <div class="muted">Last NMEA</div><pre id="mNmea">—</pre>
  </div>

  <div class="card">
    <h2>Absolute Position</h2>
    <div class="row"><span class="k">Saved source</span><span class="v" id="posSource">—</span></div>
    <div class="row"><span class="k">Saved lat</span><span class="v" id="posLat">—</span></div>
    <div class="row"><span class="k">Saved lon</span><span class="v" id="posLon">—</span></div>
    <div class="row"><span class="k">Saved alt (ellipsoidal)</span><span class="v" id="posAlt">—</span></div>
    <div class="row"><span class="k">Saved H-acc</span><span class="v" id="posAcc">—</span></div>
    <div class="row"><span class="k">Verify on a map</span><span class="v"><a id="posMap" href="#" target="_blank" rel="noopener noreferrer">—</a></span></div>
    <canvas id="mapSaved" width="256" height="256" style="display:none;max-width:100%;margin-top:6px;border-radius:6px"></canvas>
    <div class="muted" id="mapSavedMsg" style="font-size:12px"></div>
    <div class="muted" id="posNote" style="margin-top:4px"></div>
    <div class="muted" style="margin:8px 0 4px">Boot position check (statistical, 30 s)</div>
    <div class="row"><span class="k">Check state</span><span class="v" id="pcState">—</span></div>
    <div class="row"><span class="k">Live solution</span><span class="v" id="pcSol">—</span></div>
    <div class="row"><span class="k">Fixes / dist</span><span class="v" id="pcFixes">—</span></div>
    <div class="flex" style="margin-top:8px">
      <button class="btn sec" onclick="if(confirm('Clear saved position? Next boot will survey-in.'))clearPos()">Clear Saved</button>
    </div>
    <div class="muted" style="margin:12px 0 4px">Manual entry (surveyed / post-processed reference)</div>
    <div class="row"><span class="k">Lat</span><input id="inLat" type="text" inputmode="decimal" placeholder="36.123456789" style="flex:1;min-width:0"></div>
    <div class="row"><span class="k">Lon</span><input id="inLon" type="text" inputmode="decimal" placeholder="-76.123456789" style="flex:1;min-width:0"></div>
    <div class="row"><span class="k">Alt (m, ellipsoidal)</span><input id="inAlt" type="text" inputmode="decimal" placeholder="12.34" style="flex:1;min-width:0"></div>
    <button class="btn" style="margin-top:8px" onclick="savePos()">Save &amp; Apply Position</button>
    <div class="muted" id="setposMsg" style="margin-top:6px"></div>
  </div>

  <div class="card">
    <h2>Previous Locations</h2>
    <div class="flex"><button class="btn sec" id="prevBtn" onclick="loadPrevLocations()">Load from logs</button></div>
    <div class="muted" id="prevMsg" style="margin:6px 0">Lists the base positions saved in the SD card's logbook. Tap a row's coordinates to select them, or press Use to fill the manual entry fields.</div>
    <div id="prevList"></div>
  </div>

  <div class="card">
    <h2>Log Files</h2>
    <div class="flex">
      <button class="btn" id="scCur" onclick="setScope('current')">Current session</button>
      <button class="btn sec" id="scRec" onclick="setScope('recent')">Last 12</button>
      <button class="btn sec" id="scAll" onclick="setScope('all')">All</button>
      <button class="btn sec" id="scRefresh" onclick="refreshFiles()" title="Re-scan the card now. Briefly pauses SD logging.">Refresh</button>
    </div>
    <div class="muted" style="margin:10px 0 4px"><span id="filesLabel">Current session</span> — tap a file to download</div>
    <div class="muted" style="margin:0 0 6px;color:var(--warn)">Casting may be interrupted while logs download.</div>
    <div class="files filelist" id="files">…</div>
    <div class="muted" style="margin:6px 0 0">All sessions: <a href="/dl?f=rcx_logbook.csv">rcx_logbook.csv</a> (session times and saved positions)</div>
    <div class="muted" style="margin:10px 0 4px">Live RAM captures</div>
    <div class="flex">
      <a class="btn sec" href="/rtcm.raw">Validated RTCM</a>
    </div>
  </div>

</div>
<script>
const $=id=>document.getElementById(id);
let casterDefaultsSet=false;   // see poll() — pre-fills "Add a caster" once, non-destructively
// Stoplight band for a numeric reading, matching the LCD's stoplight() exactly so the
// two displays never disagree. higherIsBetter marks readings where a LARGER number is
// healthier (satellites, frames/s) rather than smaller (HDOP, error, age). A missing or
// zero reading is left unstyled instead of being shown as green.
function band(v,okAt,warnAt,higherIsBetter){
 v=+v; if(!isFinite(v)||v<=0) return '';
 return higherIsBetter ? (v>=okAt?'good':(v>=warnAt?'warn':'bad'))
                       : (v<=okAt?'good':(v<=warnAt?'warn':'bad'));
}
// Paint a whole card with its section's state. Mirrors the LCD's section colouring so
// the two surfaces always agree.
function cardState(id,st){
 const el=$(id); if(!el) return;
 el.classList.remove('s-good','s-warn','s-bad');
 if(st) el.classList.add('s-'+st);
}
// Worst-of across several band() results. '' (no reading) is ignored rather than
// treated as healthy, so an unpopulated field never turns a card green.
function worstOf(){
 let r='';
 for(const a of arguments){
  if(a==='bad') return 'bad';
  if(a==='warn') r='warn';
  else if(a==='good'&&r!=='warn') r=r||'good';
 }
 return r;
}
function cls(el,c){el.className=el.className.replace(/\b(good|warn|hot|bad)\b/g,'').trim();if(c)el.classList.add(c);}
// Show/hide toggle for a password field — just flips the input's own type so you
// can check what you typed before submitting. Nothing is fetched from the device:
// stored passwords are never sent to the browser (see API contract above).
function pwToggle(btn,id){
 const el=$(id),show=el.type==='password';
 el.type=show?'text':'password';
 btn.textContent=show?'Hide':'Show';
}
// Temperature → colour band. Chip (die) runs hotter than board (ambient); the board
// bands mirror the firmware thresholds, set below PETG / screen-adhesive limits.
// Thresholds in °F, mirroring CHIP_T_*/BOARD_T_* in the firmware — keep the two in step.
function tband(f,chip){if(chip){return f>=203?'bad':f>=185?'hot':f>=158?'warn':'good';}
                       return f>=185?'bad':f>=167?'hot':f>=149?'warn':'good';}
// Durations are hh:mm:ss everywhere — fmtDur previously rolled hours into the minutes
// field, so a 90-minute survey read "90:00". secToDur is the single formatter.
function fmtDur(s){return secToDur(s);}
// HTML-escape before concatenating anything a remote party controls. The caster's
// rejection banner is text from an outside server, and jsonSanitize() on the device
// strips quotes, backslashes and control characters but not angle brackets — so this is
// the step that keeps a hostile or simply malformed banner from injecting markup into
// the dashboard.
function esc(v){
 return String(v==null?'':v).replace(/&/g,'&amp;').replace(/</g,'&lt;')
        .replace(/>/g,'&gt;').replace(/"/g,'&quot;');
}
// Coordinates are rendered to nine decimal places everywhere, matching the precision the
// device actually carries and casts. At this latitude the ninth decimal is ~0.11 mm, the
// eighth ~1.1 mm and the seventh ~11 mm — so a seven-decimal readout rounds away more
// than a centimetre, which is the entire quantity an RTK base exists to control. Two
// cards showing the same coordinate to different precision also read as two different
// coordinates. Anything with no fix behind it shows a dash rather than 0.000000000.
function fmtDeg(v){
 if(v==null||!isFinite(v)) return '—';
 return Number(v).toFixed(9);
}
function fmtLatLon(lat,lon){
 if(lat==null||lon==null) return '—';
 if(Math.abs(lat)<1e-9&&Math.abs(lon)<1e-9) return '—';
 return fmtDeg(lat)+', '+fmtDeg(lon);
}
// Point a map at a coordinate so it can be eyeballed against where the antenna actually
// is. The link carries the full precision the device reports rather than the rounded
// text shown beside it, so following it verifies the stored value and not a display
// artefact. 0,0 is the receiver's placeholder for "no fix", never a real base position,
// so it is treated as absent rather than linked to the Gulf of Guinea.
function setMapLink(el,lat,lon,label){
 const ok = lat!=null && lon!=null && (Math.abs(lat)>1e-6 || Math.abs(lon)>1e-6);
 if(!ok){ el.textContent='—'; el.removeAttribute('href'); return; }
 el.href='https://www.google.com/maps/search/?api=1&query='+
         encodeURIComponent(Number(lat).toFixed(9)+','+Number(lon).toFixed(9));
 el.textContent=label||'Open in Google Maps';
}
function fmtBytes(b){b=+b||0;if(b<1024)return b+' B';if(b<1048576)return(b/1024).toFixed(1)+' KB';return(b/1048576).toFixed(2)+' MB';}
function fmtRate(bps){bps=+bps||0;if(bps<1024)return bps+' B/s';return(bps/1024).toFixed(1)+' KB/s';}
// fetch() with a deadline. A plain fetch waits as long as the device does, and the device
// can be busy for 10 s (PPP lock sequence) — see the in-flight guards below.
// Browsers without AbortController (iOS before 12.1) get a plain fetch with no deadline
// rather than a page that renders nothing.
function fetchT(url,opts,ms){
 if(typeof AbortController!=='function') return fetch(url,opts||{});
 const ac=new AbortController(); const t=setTimeout(()=>ac.abort(),ms);
 return fetch(url,Object.assign({},opts||{},{signal:ac.signal})).finally(()=>clearTimeout(t));
}
// ONE STATUS REQUEST AT A TIME (keep). setInterval fires every 1.5 s whether or not the last
// reply came back, so while the device was busy the requests queued up (6 sockets per
// open page at Chrome's per-host limit), button POSTs waited behind stale polls, and the
// device served a backlog of answers nobody would read.
// A call that arrives while one is in flight (e.g. the refresh after a button press) is
// remembered and run once more when the current one finishes, so it is never lost.
let pollBusy=false, pollAgain=false;
async function poll(){
 if(pollBusy){ pollAgain=true; return; }
 pollBusy=true;
 try{ do{ pollAgain=false; await pollOnce(); }while(pollAgain); } finally{ pollBusy=false; }
}
async function pollOnce(){
 try{
  const r=await fetchT('/api/status',{cache:'no-store'},5000);const d=await r.json();
  $('dev').textContent=d.dev; if(d.dev&&document.title!==d.dev)document.title=d.dev;   // tab title follows the device name
  $('dot').classList.toggle('on',!!d.wifi);
  // One-time Centipede defaults for the "Add a caster" form — only while every field is
  // still untouched, so this never clobbers something the operator is mid-typing.
  if(!casterDefaultsSet&&!$('cHost').value&&!$('cMount').value&&!$('cPw').value){
   casterDefaultsSet=true;
   $('cHost').value='crtk.net';$('cPort').value='2101';
   if(d.suggestMount)$('cMount').value=d.suggestMount;
   $('cPw').value='centipede';
  }
  $('up').textContent=(d.wifi?(d.ssid+' · '+d.ip+' · '):'')+'up '+fmtDur(d.up);
  $('up').title=d.core?('arduino-esp32 core/ESP-IDF '+d.core):'';   // build platform, hover to see
  const s=d.svin||{};
  { const b=$('btnLockNow'); if(b) b.disabled=!s.lockable; }   // device decides; see forceLockAvailable()
  const sp0=d.savedpos||{};
  // Reference source: a manually-entered or reused saved position is a FIXED REFERENCE,
  // not a survey-in. Distinguish it so the card doesn't masquerade as an active survey.
  const isManual = sp0.valid && sp0.src==='manual';
  // s.saved is the device telling us it is running on a boot-CONFIRMED saved coordinate:
  // no survey running, none pending, none needed. It has always been sent and was never
  // read here, which left the healthiest state this base has — fixed on a confirmed
  // position, publishing valid RTCM — permanently amber, titled "Survey-In", while its own
  // body text said "Fixed on saved position". A card that cannot go green in its best state
  // teaches you to ignore the colour.
  const isFixed  = !!s.fixed || !!s.skipped || !!s.saved || isManual;
  // A receiver tracking nothing cannot corroborate any survey claim — see the device's
  // receiverHasSignal(). Called out on its own because the shape of the failure is a card
  // that looks entirely healthy: converged, valid 2, a plausible accuracy, all of it
  // describing a survey that finished somewhere else.
  const noSignal = s.sig===0;
  const srcLabel = noSignal ? 'No satellite signal'
                 : isManual ? 'Manual fixed'
                 : (s.saved ? 'Saved position (confirmed)'
                 : (s.skipped ? 'Saved position (reused)'
                 : (sp0.valid && sp0.src==='survey' ? 'Survey (this session)'
                 : (s.valid==2 ? 'Survey (converged)' : 'Survey-in (running)'))));
  $('svinTitle').textContent = isFixed ? 'Fixed Reference' : 'Survey-In';
  $('svinSrc').textContent = srcLabel;
  cls($('svinSrc'), noSignal?'bad':(isFixed?'good':'warn'));
  $('svinTimer').textContent=((s.fixed||s.saved)?'FIXED':(s.skipped?'INSTANT':
    (noSignal?'NO SIGNAL':(s.valid==2?'COMPLETE':fmtDur(s.remain)))));
  cls($('svinTimer'),noSignal?'bad':'');
  $('svinState').textContent=s.state||'—';
  $('svinValid').textContent=s.valid; cls($('svinValid'),s.valid==2?'good':'warn');
  // Receiver's self-reported configuration (from "Read module config").
  const mo=d.mod||{};
  $('modCfg').textContent = (mo.mode===1||mo.mode===2)
    ? ((mo.mode===2?'base':'rover')+', svin '+(mo.svin===1?'survey-in':(mo.svin===2?'fixed':'—'))+
       ', '+secToDur(mo.dur||0)+', '+(mo.acc||0).toFixed(2)+' m')
    : 'not probed yet';
  // mo.refused: the device stopped resetting a receiver that will not take base mode or
  // the adopted coordinate; shown here and as the caster hold reason until it clears.
  if(mo.refused){ $('modCfg').textContent+=' — '+mo.refused; }
  cls($('modCfg'), mo.refused?'bad':'');
  cardState('cardSvin', worstOf(noSignal?'bad':(isFixed?'good':'warn'), s.valid==2?'good':'warn'));
  // Accuracy: never render 0.000 m for a state where nothing is known — that reads as a
  // perfect survey and is the most precise-looking value the format can produce. The
  // device sends accknown alongside the number precisely because the number cannot carry
  // "no figure yet" on its own; it is 0 from power-on until the first PQTMSVINSTATUS, and
  // for the whole of a rover-mode PPP survey, during which that message never arrives.
  // A user-entered manual reference has no known accuracy either.
  $('svinAcc').textContent = noSignal ? '— (no signal)'
                           : isManual ? 'n/a (user-entered)'
                           : (s.accknown===0 ? '— (not yet measured)'
                           : ((s.acc||0).toFixed(3)+' m'));
  cls($('svinAcc'), (noSignal||isManual||s.accknown===0) ? 'muted' : '');
  // Observations/target are meaningless for a fixed reference — hide the row entirely.
  $('svinObsRow').style.display = isFixed ? 'none' : '';
  $('svinObs').textContent=secToDur(s.obs||0)+' / '+secToDur(s.target||0);
  const p=d.pos||{};
  // GGA field 6 quality indicator. Decoded because the bare number is meaningless at a
  // glance. 5 is the RTK-float code, and this module reports its PPP solution with the same
  // code, told apart by the PPP service's station ID in GGA field 14 (pppsol.sta: 9002 E6
  // HAS, 9001 B2b), as the LCD's label is.
  const FIXQ={0:'No fix',1:'3D / autonomous',2:'DGPS',3:'PPS',4:'RTK fixed',
              5:'RTK float',6:'Dead reckoning',7:'Manual / base',8:'Simulation'};
  const fixSta=(d.pppsol||{}).sta;
  const fixTxt=(p.fix===5&&(fixSta===9002||fixSta===9001))?('PPP ('+(fixSta===9002?'E6 HAS':'B2b')+')')
              :(FIXQ[p.fix]||'unknown');
  $('fix').textContent=p.fix+' — '+fixTxt;
  cls($('fix'),p.fix>=4?'good':(p.fix>0?'warn':'bad'));
  $('sats').textContent=p.sats; cls($('sats'),band(p.sats,12,7,true));
  $('hdop').textContent=(p.hdop||0).toFixed(2); cls($('hdop'),band(p.hdop,1.0,2.0,false));
  $('epe').textContent=(p.epe||0).toFixed(3)+' m'; cls($('epe'),band(p.epe,1.0,3.0,false));
  cardState('cardPos', worstOf(p.fix>=4?'good':(p.fix>0?'warn':'bad'),
                               band(p.sats,12,7,true), band(p.hdop,1.0,2.0,false),
                               band(p.epe,1.0,3.0,false)));
  $('ll').textContent=fmtLatLon(p.lat,p.lon);
  $('alt').textContent=(p.alt||0).toFixed(2)+' m';
  $('altEll').textContent=p.hell?(p.hell.toFixed(2)+' m'):'—';
  setMapLink($('llMap'),p.lat,p.lon,'Open live fix in Google Maps');
  mapThumb('mapLive',p.lat,p.lon);
  const tp=d.temp||{};
  if(tp.chipOk){$('chipT').textContent=(tp.chipF||0).toFixed(0)+' °F';cls($('chipT'),tband(tp.chipF,true));}
  else{$('chipT').textContent='—';cls($('chipT'),'');}
  if(tp.boardOk){$('boardT').textContent=(tp.boardF||0).toFixed(0)+' °F';cls($('boardT'),tband(tp.boardF,false));}
  else{$('boardT').textContent='— (no IMU)';cls($('boardT'),'');}
  cardState('cardTemp', worstOf(tp.chipOk?tband(tp.chipF,true):'',
                                tp.boardOk?tband(tp.boardF,false):''));
  // Message types this base is configured to publish that the receiver has never once
  // produced. A rover is entitled to the whole set; anything listed here it is not
  // getting, and the write that asked for it reported nothing back.
  const miss=d.missing||[];
  $('rtcmMissing').textContent = miss.length ? ('missing '+miss.join(', ')) : 'complete';
  cls($('rtcmMissing'), miss.length ? 'warn' : 'good');
  // The antenna descriptor carried in RTCM 1033. Rovers key their phase-centre
  // corrections on this string, so an empty one costs a few centimetres of vertical
  // accuracy on every rover connected — silently, since the message itself is fine.
  const an=d.ant||{};
  $('antDesc').textContent = !an.seen ? '— (no 1033 yet)'
                           : (an.len ? an.desc : 'empty — rovers apply no phase-centre correction');   // textContent: no escaping
  cls($('antDesc'), !an.seen ? 'muted' : (an.len ? 'good' : 'warn'));
  // Correction-quality gate. This is the answer to "is this device publishing, and if
  // not, why not" — distinct from the survey card, which only says whether the survey
  // converged. A red state here means corrections are being deliberately withheld.
  const q=d.qual||{};
  const qOk=q.ok!==0;
  $('qual').textContent=qOk?'Yes':'Withheld';
  cls($('qual'),qOk?'good':'bad');
  $('qualWhy').textContent=qOk
    ? ('frame integrity '+(100-100*(q.crcrate||0)).toFixed(2)+'% over the last window')
    : ('Corrections withheld: '+(q.why||'quality gate not met'));
  // What the RECEIVER says about PPP, not what the firmware asked it to do. sup is -1
  // until PQTMCFGPPP,R has been answered or has gone unanswered; 0 means the module
  // rejected the command, which on this hardware means the firmware predates PPP support.
  // E6 satellites are the direct measure: no E6, no HAS corrections, whatever the
  // configuration claims.
  const pp=d.ppp||{};
  const pppLbl={'-1':'Not yet queried','0':'Not supported by this firmware','1':'Enabled'};
  $('pppSup').textContent=pppLbl[String(pp.sup)]||'—';
  cls($('pppSup'),pp.sup===1?'good':(pp.sup===0?'bad':'muted'));
  // The E6 count only means something while a rover-mode survey is running. Once the
  // base is fixed the receiver is no longer solving, the count drops to zero, and a red
  // zero there reads as a fault when it is the normal end state. Blank instead.
  const surveying = pp.act === 1;
  $('pppE6').textContent = surveying ? (pp.e6==null?'—':pp.e6) : '';
  // The module's own PQTMPPPNAV, verbatim. Everything else on this card describes the
  // ordinary solution, so this is the only line that can say whether the receiver has a
  // PPP fix at all. "not reporting" means the message is not arriving — which is itself
  // the answer, since a receiver running PPP publishes it.
  $('pppNav').textContent = (pp.nav && pp.navage>=0 && pp.navage<30)
    ? pp.nav : (pp.nav ? ('stale ('+pp.navage+'s): '+pp.nav) : 'not reporting');
  cls($('pppNav'), (pp.nav && pp.navage>=0 && pp.navage<30) ? 'good' : 'warn');
  cls($('pppE6'), !surveying ? 'muted' : (pp.e6>0?(pp.e6ok?'good':'warn'):'bad'));
  $('pppNote').textContent = pp.sup===1
    ? ((!surveying || pp.e6>0) ? (pp.cfg||'')
             : 'PPP is enabled but no Galileo E6 signals are being tracked — HAS corrections cannot arrive, so the survey is averaging ordinary autonomous fixes.')
    : (pp.sup===0 ? (pp.cfg||'The module rejected PQTMCFGPPP.')
                  : 'Press "Read module config" to ask the receiver directly.');
  // pp.fail: the receiver's own output makes every survey fail (the survey keeps retrying);
  // it also becomes the caster hold reason.
  // While a survey averages: which mean a lock now would use, and the numbers behind it.
  if(surveying && pp.why){ $('pppNote').textContent=($('pppNote').textContent?($('pppNote').textContent+' · '):'')+'Lock now: '+pp.why; }
  if(pp.fail){ $('pppNote').textContent=pp.fail; }
  cls($('pppNote'), pp.fail?'bad':'');
  $('pppNote').classList.toggle('muted', !pp.fail);   // .muted is declared after .bad and would grey it
  const rt=d.rtcm||{};
  // INTERNAL SIGNAL ONLY — this says the receiver is producing RTCM observations for
  // us, not that anything is being published. Whether corrections actually reach the
  // outside world is the Casters card's job. The old "observations streaming" wording
  // read like the base was broadcasting.
  $('obs').textContent=rt.obs?'YES — corrections generated':'NO — no corrections generated';
  cls($('obs'),rt.obs?'good':'bad');
  // A base that has stopped producing corrections is the single most important failure
  // to notice at a glance, so the frame rate is graded rather than shown as a bare number.
  $('fps').textContent=rt.fps; cls($('fps'),band(rt.fps,4,1,true));
  $('bps').textContent=rt.bps; $('vframes').textContent=rt.valid;
  $('vfail').textContent=(rt.cand||0)+' / '+(rt.crcfail||0)+' / '+(rt.framefail||0);
  cls($('vfail'),(rt.crcfail||rt.framefail)?'warn':'good');
  cardState('cardRtcm', worstOf(rt.obs?'good':'bad', band(rt.fps,4,1,true),
                                (rt.crcfail||rt.framefail)?'warn':'good'));
  const tt=d.types||[];
  $('types').innerHTML=tt.length?tt.map(t=>{
    const age=(t.age==null)?'':((t.age/1000).toFixed(1)+'s ago');
    return '<div class="row"><span class="'+(t.obs?'good':'')+'">'+t.t+' · '+t.n+'</span>'+
           '<span class="muted">×'+t.c+' · '+age+'</span></div>';
  }).join(''):'<span class="muted">none</span>';
  // ── Local caster, shown as the first entry in this list ──────────────────
  // It is a caster like any other from the operator's point of view — a place rovers
  // take corrections from — so it belongs here rather than in a card of its own. It is
  // served by this unit, so it has no host to configure and cannot be deleted; only
  // enabled or disabled.
  const lc=d.local||{};
  let h='', casterWorst='';
  {
    lcOn = !!lc.on;
    // While the base is not publishable (d.held) the local caster sends nothing, the same
    // gate the upstream casters apply, so connected rovers are shown as held.
    const lcKlass = lc.on ? ((lc.clients>0 && !d.held) ? 'good' : 'warn') : 'muted';
    const lcLabel = lc.on ? ((lc.clients||0)+' rover'+(lc.clients===1?'':'s')+' connected'+(d.held?' · held, base not publishable':'')) : 'Disabled';
    // Both addresses are listed when there is an uplink: the LAN address is the one a
    // rover already on the same network uses, and the AP address keeps working whether
    // or not the base is joined to anything.
    const lanAddr = (d.wifi && d.ip) ? (d.ip+':'+(lc.port||2101)) : '';
    const apAddr  = (lc.apip||'192.168.4.1')+':'+(lc.port||2101);
    h+='<div style="margin-bottom:12px;opacity:'+(lc.on?'1':'0.55')+'">'+
       '<div class="row"><span class="k">'+esc(lc.name||'Local-Wifi')+
       ' <span class="muted">'+esc(d.dev||'this base')+'</span></span>'+
       '<span class="v '+lcKlass+'">'+lcLabel+'</span></div>';
    if(lc.on){
      h+='<div class="muted">'+(lanAddr?('network '+lanAddr+' · '):'')+'AP '+apAddr+'</div>';
    }
    // Owner decision: a rover fed over the AP has priority, so a lost uplink is not searched
    // for while it is connected (searching would pull the radio off the AP's channel).
    if(lc.pause===2){
      h+='<div class="warn">Uplink search paused — local rover has priority. Search resumes about a minute after the rover stops being fed (it disconnects or local casting is turned off).</div>';
    }else if(lc.pause===1){
      h+='<div class="muted">Uplink search paused while the dashboard is in use (up to 60 s after the last action).</div>';
    }
    h+='<div class="flex" style="margin-top:6px"><button class="btn sec" style="padding:2px 8px" '+
       'onclick="lcToggle()">'+(lc.on?'Disable':'Enable')+'</button></div></div>';
    if(lc.on) casterWorst=worstOf(casterWorst,lcKlass);
  }
  (d.casters||[]).forEach((c,i)=>{
   const streaming=(c.state||'').indexOf('Streaming')===0;
   const ok=c.hand&&c.age>=0&&c.age<10;
   const errored=(c.state==='Error');
   const enabled=(c.en!==0);
   // Host and mount reach the buttons through data- attributes and the page through esc():
   // they are operator text stored on the device, and inserted raw any markup in them ran.
   const hostA=esc(c.host||''), mountA=esc(c.mount||'');
   // Status label. A disabled caster is neutral grey, not an error.
   let label,klass;
   if(!enabled){ label='Disabled'; klass='muted'; }
   else if(d.held && !streaming){
     // The gate that is actually holding it, in its own words. q.why carries the
     // correction-quality reason; falling back to the survey only when quality is fine
     // keeps the label from blaming survey-in for an antenna or frame-integrity fault.
     const q=d.qual||{}, mr=(d.mod&&d.mod.refused)||'', ps=(d.ppp&&d.ppp.fail)||'';
     label='Holding — '+(mr ? mr+' — power-cycle the receiver'
                        : ps ? ps
                        : q.ok===0&&q.why ? q.why
                        : (q.refok===0 ? 'confirming the receiver reference position'
                        : (d.svin&&d.svin.valid==2?'starting up':'survey-in')));
     klass='warn';
   }
   // c.retry: seconds to the device's next connection attempt while it cools down after a
   // failure (-1 when not waiting); the reason stays on the line below.
   else if(errored){ label=c.retry>0?('Error — retry in '+fmtDur(c.retry)):'Error'; klass='bad'; }
   else if(c.retry>0){ label='Retry in '+fmtDur(c.retry); klass='warn'; }
   else { label=c.state||''; klass=(ok?'good':(c.hand?'warn':'bad')); }
   // A disabled caster is an operator choice and does not colour the card.
   if(enabled) casterWorst=worstOf(casterWorst,klass);
   const toggle='<button class="btn sec" style="padding:2px 8px" data-h="'+hostA+'" data-m="'+mountA+'" onclick="casterEnable(this.dataset.h,this.dataset.m,'+(enabled?0:1)+')">'+(enabled?'Disable':'Enable')+'</button>';
   const del='<button class="btn sec" style="padding:2px 8px;margin-left:6px" data-h="'+hostA+'" data-m="'+mountA+'" onclick="casterDel(this.dataset.h,this.dataset.m)" title="Remove">✕</button>';
   h+='<div style="margin-bottom:12px;opacity:'+(enabled?'1':'0.55')+'"><div class="row"><span class="k">'+(hostA||('Caster '+(i+1)))+':'+esc(c.port||2101)+
      ' <span class="muted">'+mountA+'</span></span><span class="v '+klass+'">'+esc(label)+'</span></div>';
   if(enabled && c.err){
     h+='<div class="muted '+(errored?'bad':'')+'">↳ '+esc(c.err)+'</div>';
     if(c.resp) h+='<div class="muted" style="font-size:0.85em;word-break:break-all">↳ caster said: '+esc(c.resp)+'</div>';
   }
   if(enabled){
     h+='<div class="muted">bw '+fmtBytes(c.bytes)+' total · '+fmtRate(c.live)+' now · '+fmtRate(c.avg)+' avg</div>'+
        '<div class="muted">frames '+c.frames+' · drop '+c.drop+' · last '+(c.age<0?'never':c.age+'s')+'</div>';
     // offered vs accepted separates "the caster is not taking our bytes" from "our own
     // socket never let them out". They are equal on a healthy link; a gap between them,
     // or bytes stuck in the buffer, is where a stalled push actually lives.
     if(c.offered!=null && (c.offered!==c.bytes || c.buf>0 || c.cong)){
       h+='<div class="muted">offered '+fmtBytes(c.offered)+' · buffered '+(c.buf||0)+' B'+
          (c.cong?' · socket not writable':'')+'</div>';
     }
   }
   h+='<div class="flex" style="margin-top:6px">'+toggle+del+'</div></div>';
  });
  // NO CASTER CONFIGURED is a real problem, not an empty list: without one the base
  // surveys and logs but publishes corrections to nobody. Say so plainly and paint red.
  if(!(d.casters||[]).length){
    h+='<div class="muted" style="margin-top:4px">No upstream caster configured — corrections are only available to rovers on this base\u2019s own network. Add one below to publish them.</div>';
  }
  // The reply carries as many casters as fit; ncast is how many are configured.
  const shownC=(d.casters||[]).length;
  if(d.ncast>shownC){
    h+='<div class="warn" style="margin-top:4px">'+(d.ncast-shownC)+' more caster(s) configured but not shown (status reply full).</div>';
  }
  $('casters').innerHTML=h;
  // The card is only red when there is nowhere at all to get corrections: no upstream
  // caster AND the local one switched off.
  // WiFi card: green when associated to an uplink, amber when running AP-only (the base
  // still surveys, logs and serves local rovers — it just cannot reach an upstream
  // caster), red only if the AP is down too, which means nothing can reach the device.
  cardState('cardWifi', d.wifi ? 'good' : (d.local&&d.local.apip ? 'warn' : 'bad'));
  // What the uplink search is doing, as the LCD footer says it (lvl 3: the last attempt failed).
  const ws=d.wsrch||{};
  $('wfState').textContent=d.wifi?('connected to '+d.ssid):(ws.t||'—');
  cls($('wfState'),d.wifi?'good':(ws.lvl===3?'bad':'warn'));

  cardState('castersCard', ((d.casters||[]).length || lc.on) ? (casterWorst||'warn') : 'bad');
  if(d.log){
   document.querySelectorAll('.tg').forEach(t=>t.classList.toggle('on',!!d.log[t.dataset.ch]));
   window.curSessionStr=String(d.log.session);
   $('logNote').textContent=(d.log.sd?('SD ready · session '+String(d.log.session).padStart(4,'0')):(d.log.sdfail?'SD card FAILED - not mounted, nothing is being logged (check or replace the card, then restart)':'SD NOT ready'))+' · dropped records '+d.log.drop+(d.log.shed?(' · sat rows skipped '+d.log.shed):'')+(d.log.wfail?(' · SD WRITE FAILURES '+d.log.wfail+' (card full or failing)'):'');
  }
  if(d.msg){$('mPqtm').textContent=d.msg.pqtm||'—';$('mNmea').textContent=d.msg.nmea||'—';}
  // Saved position card
  const sp=d.savedpos||{};
  // ppp-auto is a survey that did not converge: an average that can be metres from the
  // antenna (see the note and accuracy). Said in words, not left to the source code.
  const unconv = sp.valid && sp.src==='ppp-auto';
  $('posSource').textContent=sp.valid?((sp.src||'?')+(unconv?' (UNCONVERGED)':'')):'none';
  cls($('posSource'),sp.valid?(unconv?'warn':'good'):'muted');
  $('posLat').textContent=sp.valid?fmtDeg(sp.lat):'—';
  $('posLon').textContent=sp.valid?fmtDeg(sp.lon):'—';
  // Saved altitude is ELLIPSOIDAL metres — the value actually fed into the ECEF the base
  // broadcasts. Surfacing it makes the "where is 474 m coming from" question answerable.
  $('posAlt').textContent=sp.valid?(sp.alt.toFixed(3)+' m'):'—';
  // A manual reference has no measured accuracy — show n/a rather than a fake "0 cm".
  $('posAcc').textContent = sp.valid ? (sp.src==='manual' ? 'n/a (user-entered)'
                                   : (sp.hacc>=1 ? sp.hacc.toFixed(2)+' m' : (sp.hacc*100).toFixed(0)+' cm')) : '—';
  cls($('posAcc'), unconv ? 'warn' : '');
  setMapLink($('posMap'), sp.valid?sp.lat:null, sp.valid?sp.lon:null, 'Open saved position in Google Maps');
  mapThumb('mapSaved', sp.valid?sp.lat:null, sp.valid?sp.lon:null);
  // Provenance of the saved coordinate — how it was actually produced.
  $('posNote').textContent = (sp.valid && sp.note) ? sp.note : '';
  // Boot position-confidence check card
  const pc=d.poscheck||{};
  const pcLabel={idle:'Idle',collecting:'Collecting\u2026',confirmed:'Confirmed \u2713',moved:'Moved \u2717',timeout:'Timeout \u2192 survey'};
  $('pcState').textContent=(pcLabel[pc.state]||pc.state||'—')+(pc.reason?(' \u2014 '+pc.reason):'');
  cls($('pcState'),{confirmed:'good',collecting:'warn',moved:'bad',timeout:'warn'}[pc.state]||'');
  $('pcSol').textContent=(pc.sol||'—')+(pc.soltype!=null?(' ('+pc.soltype+')'):'');
  cls($('pcSol'),pc.soltype>=8?'good':(pc.soltype>0?'warn':''));
  if(pc.state==='collecting'){
    $('pcFixes').textContent=(pc.n||0)+' / '+(pc.need||0)+' fixes · '+(pc.elapsed||0)+'s';
  }else if(pc.state==='confirmed'||pc.state==='moved'){
    $('pcFixes').textContent='dist '+(pc.dist||0).toFixed(2)+'m vs thresh '+(pc.thresh||0).toFixed(2)+'m';
  }else{
    $('pcFixes').textContent='—';
  }
  if(d.cfg&&d.cfg.tmin) cfgTmin=d.cfg.tmin;
  if(!cfgTouched){
   $('cfgTime').value=secToDur((d.cfg&&d.cfg.time)||s.target||0);
   $('cfgAcc').value=(d.cfg&&d.cfg.acc)||'';
  }
 }catch(e){$('dot').classList.remove('on');}
}
// Clears the saved coordinate AND starts a fresh survey — the device does both, since
// clearing alone left the module publishing the old fixed position until a reboot.
async function clearPos(){
  if(!confirm('Clear the saved base position and start a new survey?')) return;
  const msg=$('setposMsg');
  if(msg){msg.textContent='Clearing…';cls(msg,'');}
  try{
    const r=await post('/api/clearpos');
    const t=await r.text();
    if(msg){msg.textContent=r.ok?('Cleared — '+(t||'new survey started')):('Error: '+t);cls(msg,r.ok?'good':'bad');}
  }catch(e){ if(msg){msg.textContent='Request failed';cls(msg,'bad');} }
  poll();
}
async function savePos(){
  const lat=$('inLat').value.trim(),lon=$('inLon').value.trim(),alt=$('inAlt').value.trim();
  // Height is required (ellipsoidal, m): a blank used to be sent as 0 and published a
  // reference tens of metres below the antenna. The device refuses a missing height too.
  if(!lat||!lon||!alt){$('setposMsg').textContent='Enter lat, lon and ellipsoidal height';cls($('setposMsg'),'bad');return;}
  try{
    const body='lat='+encodeURIComponent(lat)+'&lon='+encodeURIComponent(lon)+'&alt='+encodeURIComponent(alt);
    const hdr={'Content-Type':'application/x-www-form-urlencoded'};
    let r=await fetch('/api/setpos',{method:'POST',headers:hdr,body:body});
    let t=await r.text();
    // The device refuses a height far from the current one (likely MSL or a typo). The
    // operator can confirm it deliberately, e.g. to correct a base whose stored height is wrong.
    if(r.status===409 && t.indexOf('MSL entered')>=0 &&
       confirm(t+'\n\nApply this height anyway?')){
      r=await fetch('/api/setpos',{method:'POST',headers:hdr,body:body+'&heightok=1'});
      t=await r.text();
    }
    $('setposMsg').textContent=r.ok?'Saved & applied \u2713':('Error: '+t);
    cls($('setposMsg'),r.ok?'good':'bad');
  }catch(e){$('setposMsg').textContent='Request failed';cls($('setposMsg'),'bad');}
  poll();
}
let cfgTouched=false;
// Shortest survey time the device accepts (venue check window + settle; status cfg.tmin).
// 0 until the first poll, when the device's own check decides.
let cfgTmin=0;
$('cfgTime').addEventListener('input',()=>cfgTouched=true);
$('cfgAcc').addEventListener('input',()=>cfgTouched=true);
let lcOn=false;   // mirrors d.local.on from the last poll; see lcToggle()
async function lcToggle(){
 const on=!lcOn;
 try{await fetch('/api/localcast',{method:'POST',headers:{'Content-Type':'application/x-www-form-urlencoded'},body:'on='+(on?1:0)});}catch(e){}
 poll();
}
// POST with a form-encoded BODY, not a bare query string. A POST carrying only a query
// string is parsed inconsistently by the ESP32 WebServer, so hasArg() could come back
// empty and the handler would quietly do nothing — which is why these buttons appeared
// dead. The endpoints that always worked (/api/logdel, /api/localcast) send a body, so
// everything now does. Also reports the actual outcome instead of always saying "Sent".
async function post(url, params){
 const body = params ? new URLSearchParams(params).toString() : '';
 return fetch(url, {method:'POST',
   headers:{'Content-Type':'application/x-www-form-urlencoded'}, body: body});
}
async function act(u, params, label){
 const note=$('cfgNote');
 if(note) note.textContent = (label||'Working')+'…';
 try{
  const r=await post(u, params);
  const t=await r.text();
  if(note) note.textContent = r.ok ? ((label||'Done')+' — '+(t||'ok')) : ('Error: '+(t||r.status));
 }catch(e){ if(note) note.textContent='Request failed'; }
 poll();
}
// hh:mm:ss <-> seconds. Accepts hh:mm:ss, mm:ss or a bare seconds count so an existing
// habit of typing "3600" still works; always renders back as hh:mm:ss.
// A bare number is SECONDS ("3600" = one hour). With colons it is a duration:
// hh:mm:ss, or mm:ss. Anything else is rejected rather than silently misread.
function durToSec(v){
 const str=String(v==null?'':v).trim();
 if(!str) return NaN;
 if(!/^\d+(:\d{1,2}){0,2}$/.test(str)) return NaN;
 const p=str.split(':').map(x=>parseInt(x,10));
 if(p.some(x=>isNaN(x)||x<0)) return NaN;
 if(p.length===1) return p[0];                       // bare seconds
 if(p.length===2) return p[0]*60+p[1];               // mm:ss
 return p[0]*3600+p[1]*60+p[2];                      // hh:mm:ss
}
function secToDur(s){s=Math.max(0,Math.floor(+s||0));
 const h=Math.floor(s/3600),m=Math.floor(s%3600/60),x=s%60;
 return (h<10?'0':'')+h+':'+(m<10?'0':'')+m+':'+(x<10?'0':'')+x;}
// restart=0 retargets the survey in place — the receiver keeps the observations it has
// already accumulated. restart=1 starts the survey over, still without resetting the
// receiver, so there is no cold re-acquisition either way.
async function applyCfg(restart){
 const t=durToSec($('cfgTime').value),a=$('cfgAcc').value;
 if(isNaN(t)||t<Math.max(1,cfgTmin)||t>86400){
  $('cfgNote').textContent='Survey time must be '+secToDur(Math.max(1,cfgTmin))+' to 24:00:00'+
   (cfgTmin?' (the venue check needs '+cfgTmin+' s to confirm a survey).':'.');return;}
 $('cfgNote').textContent=restart?'Applying — restarting survey…':'Applying — survey continues…';
 try{
  const p={time:t, acc:a}; if(restart) p.restart=1;
  const r=await post('/api/config', p);
  // Echo the interpreted duration back so a typo like "60" (a minute, not an hour) is
  // obvious immediately, and normalise the field to hh:mm:ss.
  if(r.ok){
   $('cfgTime').value=secToDur(t);
   $('cfgNote').textContent=(restart?'Applied — survey restarted. ':'Applied — survey still running. ')+
     'Window '+secToDur(t)+', accuracy limit '+a+' m.';
   cfgTouched=false;
  }else{
   $('cfgNote').textContent='Error applying: '+(await r.text());
  }
 }catch(e){$('cfgNote').textContent='Request failed';}
}
document.querySelectorAll('.tg').forEach(t=>t.addEventListener('click',async()=>{
 const on=!t.classList.contains('on');t.classList.toggle('on',on);
 try{await fetch('/api/log',{method:'POST',headers:{'Content-Type':'application/x-www-form-urlencoded'},body:'ch='+t.dataset.ch+'&on='+(on?1:0)});}catch(e){}
}));
let logScope='current';
const scopeLabel={current:'Current session',recent:'Last 12 files',all:'All files'};
function setScope(s){logScope=s;
 ['scCur','scRec','scAll'].forEach(id=>$(id).className='btn sec');
 $({current:'scCur',recent:'scRec',all:'scAll'}[s]).className='btn';
 $('filesLabel').textContent=scopeLabel[s];loadFiles(true);}   // explicit action -> rescan
function refreshFiles(){ loadFiles(true); }
async function loadFiles(force){
 try{const r=await fetch('/logs.json?scope='+logScope+(force?'&refresh=1':''),{cache:'no-store'});const j=await r.json();
  const note=((logScope!=='all'&&j.total!=null)?(' <span class="muted">('+(j.files||[]).length+' of '+j.total+')</span>'):'')+
   (j.tarskip?(' <span class="muted">· a one-archive download of this set leaves out the '+j.tarskip+' oldest (4 GiB limit)</span>'):'');
  $('filesLabel').innerHTML=scopeLabel[logScope]+note;
  $('files').innerHTML=(j.files||[]).map(f=>'<div class="row"><span><a data-f="'+esc(f.n)+'" href="/dl?f='+encodeURIComponent(f.n)+'">'+esc(f.n)+'</a><br><span class="muted lt" data-f="'+esc(f.n)+'"></span></span><span class="muted">'+esc(f.kb)+' KB</span><button type="button" class="btn sec" style="padding:2px 8px;margin-left:8px;flex:0 0 auto" data-f="'+esc(f.n)+'" onclick="logDel(this.dataset.f)">Del</button></div>').join('')||'<div class="muted">no files</div>';
  applyLogTimes();
  annotateLogTimes();
 }catch(e){$('files').innerHTML='<div class="muted">list unavailable</div>';}
}
// Deletes one log file. The active session's own files always come back rejected by
// the device (see handleApiLogDelete) — only older sessions can actually be removed.
async function logDel(name){
 if(!confirm('Delete "'+name+'"? This cannot be undone.'))return;
 try{
  const r=await fetch('/api/logdel',{method:'POST',headers:{'Content-Type':'application/x-www-form-urlencoded'},body:'f='+encodeURIComponent(name)});
  if(r.ok){loadFiles();}else{const t=await r.text();alert('Error: '+t);}
 }catch(e){alert('Request failed');}
}
// The file list is served from a device-side snapshot; a rescan pauses SD logging, so
// it is requested rarely and on demand rather than on a short timer.
poll();loadFiles();setInterval(poll,1500);setInterval(loadFiles,300000);

// ── Satellite thumbnail ──────────────────────────────────────────────────────
// One Esri World Imagery tile at the deepest zoom the service has for the spot, with the
// position marked on it. Everything happens in this browser: the device only supplies the
// coordinate. The image needs internet on THIS device (a phone joined only to the base's
// own access point has none), so every failure path ends in a plain message and a slow
// retry; nothing here can block or break the dashboard poll.
const MAP_MAX_ZOOM=19, MAP_MIN_ZOOM=10, MAP_RETRY_MS=60000, MAP_LOAD_TIMEOUT_MS=8000;
const MAP_TILE_URL='https://services.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/';
const MAP_ATTRIB='Imagery: Esri, Maxar, Earthstar Geographics, and the GIS User Community';
const mapState={};
function mapTile(lat,lon,z){
 const n=Math.pow(2,z), x=(lon+180)/360*n, lr=lat*Math.PI/180;
 const y=(1-Math.log(Math.tan(lr)+1/Math.cos(lr))/Math.PI)/2*n;
 return {x:Math.floor(x),y:Math.floor(y),fx:x-Math.floor(x),fy:y-Math.floor(y)};
}
function mapLoadImg(url,ms){
 return new Promise(res=>{
  const im=new Image(); let done=false;
  const fin=ok=>{ if(done) return; done=true; clearTimeout(tm); res(ok?im:null); };
  const tm=setTimeout(()=>fin(false),ms);
  im.onload=()=>fin(true); im.onerror=()=>fin(false);
  im.src=url;
 });
}
function mapDraw(st,cv,fx,fy){
 const g=cv.getContext('2d'); g.drawImage(st.img,0,0,cv.width,cv.height);
 const px=fx*cv.width, py=fy*cv.height;
 g.lineWidth=3; g.strokeStyle='#ffffff'; g.beginPath(); g.arc(px,py,10,0,6.2832); g.stroke();
 g.lineWidth=1.5; g.strokeStyle='#d00000'; g.beginPath(); g.arc(px,py,10,0,6.2832); g.stroke();
 g.beginPath(); g.moveTo(px-16,py); g.lineTo(px+16,py); g.moveTo(px,py-16); g.lineTo(px,py+16); g.stroke();
}
async function mapFetchBest(lat,lon){
 // Deepest zoom first. If the deepest fails, a shallow probe separates "no internet" from
 // "no imagery that deep here" before stepping down, so an offline phone costs two requests.
 const top=await mapLoadImg(MAP_TILE_URL+MAP_MAX_ZOOM+'/'+mapTile(lat,lon,MAP_MAX_ZOOM).y+'/'+mapTile(lat,lon,MAP_MAX_ZOOM).x,MAP_LOAD_TIMEOUT_MS);
 if(top) return {img:top,z:MAP_MAX_ZOOM};
 const probe=await mapLoadImg(MAP_TILE_URL+MAP_MIN_ZOOM+'/'+mapTile(lat,lon,MAP_MIN_ZOOM).y+'/'+mapTile(lat,lon,MAP_MIN_ZOOM).x,MAP_LOAD_TIMEOUT_MS);
 if(!probe) return null;
 for(let z=MAP_MAX_ZOOM-1;z>MAP_MIN_ZOOM;z--){
  const t=mapTile(lat,lon,z);
  const im=await mapLoadImg(MAP_TILE_URL+z+'/'+t.y+'/'+t.x,MAP_LOAD_TIMEOUT_MS);
  if(im) return {img:im,z:z};
 }
 return {img:probe,z:MAP_MIN_ZOOM};
}
function mapThumb(id,lat,lon){
 try{
  const cv=$(id), msg=$(id+'Msg'); if(!cv||!msg) return;
  const ok=lat!=null&&lon!=null&&isFinite(lat)&&isFinite(lon)&&(Math.abs(lat)>1e-6||Math.abs(lon)>1e-6)&&Math.abs(lat)<85;
  if(!ok){ cv.style.display='none'; msg.textContent=''; return; }
  const st=mapState[id]||(mapState[id]={busy:false,img:null,key:'',z:0,retryAt:0,fx:-1,fy:-1});
  if(st.img){
   const t=mapTile(lat,lon,st.z), key=st.z+'/'+t.x+'/'+t.y;
   if(key===st.key){
    // Same tile: repaint when the marker has moved by at least a pixel, or when the
    // canvas was hidden by an invalid coordinate in between.
    const hidden=cv.style.display!=='block';
    if(hidden||Math.abs(t.fx-st.fx)*cv.width>=1||Math.abs(t.fy-st.fy)*cv.height>=1){ st.fx=t.fx; st.fy=t.fy; mapDraw(st,cv,t.fx,t.fy); }
    if(hidden){ cv.style.display='block'; msg.textContent=MAP_ATTRIB+' · zoom '+st.z; }
    return;
   }
  }
  // The coordinate is on a different tile now: never leave the old site's picture and
  // marker on screen while the new one loads or after it fails to load.
  if(st.img){ st.img=null; st.key=''; cv.style.display='none'; }
  if(st.busy||Date.now()<st.retryAt) return;
  st.busy=true; msg.textContent='Loading satellite image…';
  mapFetchBest(lat,lon).then(r=>{
   st.busy=false;
   if(!r){ st.retryAt=Date.now()+MAP_RETRY_MS; msg.textContent='Satellite image unavailable (needs internet on this device)'; return; }
   st.img=r.img; st.z=r.z;
   const t=mapTile(lat,lon,r.z); st.key=r.z+'/'+t.x+'/'+t.y; st.fx=t.fx; st.fy=t.fy;
   cv.style.display='block'; mapDraw(st,cv,t.fx,t.fy);
   msg.textContent=MAP_ATTRIB+' · zoom '+r.z;
  }).catch(()=>{ st.busy=false; st.retryAt=Date.now()+MAP_RETRY_MS; });
 }catch(e){}
}

// ── Logbook: log times and previous locations ────────────────────────────────
// rcx_logbook.csv (read through /dl) holds one 'boot' line per session, tagged with the
// file number in that session's file names (file_no) and written once the
// receiver knows UTC time, with the device uptime at that moment: utc - uptime is when
// the session booted, which is when each of its files was created. 'position' lines are
// the saved base positions. The device only serves the file; parsing, converting to
// local time and building file names happens here. Each read briefly pauses SD logging,
// so the copy is reused and only re-read on request or while the current session's
// boot line has not arrived yet (at most every 30 s).
const sessionStart={};     // session number -> boot time, ms since epoch (UTC)
let logbook=null, logbookAt=0, logbookReq=null;
function fileSession(n){ const m=/_(\d+)/.exec(n); return m?String(parseInt(m[1],10)):null; }
function parseLogbook(text){
 const rows=[];
 text.split(/\r?\n/).forEach(ln=>{
  if(!ln||ln.startsWith('utc_time')) return;   // header
  const c=ln.split(','); if(c.length<12) return;
  rows.push({utc:c[0],s:String(parseInt(c[1],10)),up:Number(c[2]),kind:c[3],src:c[4],
   lat:parseFloat(c[5]),lon:parseFloat(c[6]),h:parseFloat(c[7]),hacc:parseFloat(c[8]),note:c.slice(11).join(',')});
 });
 rows.forEach(x=>{
  if(x.kind!=='boot') return;
  const t=Date.parse(x.utc); if(isFinite(t)&&isFinite(x.up)) sessionStart[x.s]=t-x.up;
 });
 return rows;
}
function getLogbook(force){
 const cur=window.curSessionStr||null;
 const fresh=logbook&&(Date.now()-logbookAt<30000||(cur!=null&&sessionStart[cur]!=null));
 if(!force&&fresh) return Promise.resolve(logbook);
 if(logbookReq) return logbookReq;
 logbookReq=(async()=>{
  try{
   const r=await fetch('/dl?f=rcx_logbook.csv',{cache:'no-store'});
   if(r.status===404){ logbook=[]; logbookAt=Date.now(); return logbook; }   // no logbook on this card yet
   if(!r.ok) return logbook;
   logbook=parseLogbook(await r.text()); logbookAt=Date.now(); return logbook;
  }catch(e){ return logbook; }
  finally{ logbookReq=null; }
 })();
 return logbookReq;
}
function p2(n){ return String(n).padStart(2,'0'); }
function stampLocal(ms){
 const d=new Date(ms), off=-d.getTimezoneOffset(), a=Math.abs(off);
 return d.getFullYear()+p2(d.getMonth()+1)+p2(d.getDate())+'-'+p2(d.getHours())+p2(d.getMinutes())+p2(d.getSeconds())+
        (off<0?'-':'+')+p2(Math.floor(a/60))+p2(a%60);
}
function stampedName(n,ms){
 const i=n.lastIndexOf('.'); return i>0?n.slice(0,i)+'_'+stampLocal(ms)+n.slice(i):n+'_'+stampLocal(ms);
}
function applyLogTimes(){
 try{
  document.querySelectorAll('#files a[data-f]').forEach(a=>{
   const n=a.dataset.f, s=fileSession(n), ms=s!=null?sessionStart[s]:null;
   const lt=document.querySelector('#files span.lt[data-f="'+n+'"]');
   if(ms==null){ if(lt) lt.textContent=''; return; }
   if(lt) lt.textContent=new Date(ms).toLocaleString();
   // The save-as name travels in the URL (as=): the reply's Content-Disposition header
   // outranks the download attribute, so the attribute alone never took effect.
   const sn=stampedName(n,ms);
   a.setAttribute('download',sn);
   a.href='/dl?f='+encodeURIComponent(n)+'&as='+encodeURIComponent(sn);
  });
 }catch(e){}
}
async function annotateLogTimes(){
 try{ await getLogbook(false); applyLogTimes(); }catch(e){}
}

// ── Previous locations ───────────────────────────────────────────────────────
// Lists every saved position in the logbook, newest first, so the coordinates of an
// earlier site can be selected or loaded into the manual-entry fields when the antenna
// goes back up in the same place.
function satMapUrl(lat,lon){
 return 'https://www.google.com/maps/@?api=1&map_action=map&center='+lat.toFixed(9)+','+lon.toFixed(9)+'&zoom=20&basemap=satellite';
}
let prevRows=[];
function renderPrev(){
 const rows=prevRows.slice().sort((a,b)=>(b.t||0)-(a.t||0)||b.s-a.s);
 const seen={}, out=[];
 rows.forEach(r=>{
  const k=r.lat.toFixed(6)+'|'+r.lon.toFixed(6)+'|'+r.h.toFixed(1);
  if(seen[k]){ seen[k].n++; return; }
  r.n=1; seen[k]=r; out.push(r);
 });
 $('prevList').innerHTML=out.map((r,i)=>
  '<div style="padding:6px 0;border-bottom:1px dashed var(--line)">'+
  '<div class="muted">'+(r.t?new Date(r.t).toLocaleString():('session '+r.s))+' · '+esc(r.src)+
  (r.src==='ppp-auto'?' (UNCONVERGED)':'')+   // a survey that did not converge: possibly metres off (see posSource)
  (isFinite(r.hacc)?(' · est. h-acc '+(r.hacc*100).toFixed(0)+' cm'):'')+(r.n>1?(' · recorded '+r.n+'×'):'')+'</div>'+
  '<div style="user-select:all;-webkit-user-select:all;font-family:ui-monospace,monospace">'+
  r.lat.toFixed(9)+', '+r.lon.toFixed(9)+', '+r.h.toFixed(3)+'</div>'+
  '<div class="flex" style="margin-top:4px"><button type="button" class="btn sec" style="padding:2px 10px" onclick="usePrev('+i+')">Use</button>'+
  '<a class="btn sec" style="padding:2px 10px" target="_blank" rel="noopener noreferrer" href="'+satMapUrl(r.lat,r.lon)+'">Satellite map</a></div>'+
  (r.note?('<div class="muted" style="font-size:12px">'+esc(r.note)+'</div>'):'')+
  '</div>').join('')||'';
 window.prevShown=out;
}
function usePrev(i){
 const r=(window.prevShown||[])[i]; if(!r) return;
 $('inLat').value=r.lat.toFixed(9); $('inLon').value=r.lon.toFixed(9); $('inAlt').value=r.h.toFixed(3);
 $('setposMsg').textContent='Loaded from logs — press Save & Apply Position to use it.';
 $('inLat').scrollIntoView({block:'center'});
}
let prevBusy=false;
async function loadPrevLocations(){
 if(prevBusy) return; prevBusy=true; $('prevBtn').disabled=true; prevRows=[]; $('prevList').innerHTML='';
 try{
  $('prevMsg').textContent='Reading the logbook…';
  const lb=await getLogbook(true);
  if(lb==null){ $('prevMsg').textContent='Could not read the logbook.'; return; }
  prevRows=lb.filter(x=>x.kind==='position'&&isFinite(x.lat)&&isFinite(x.lon)&&isFinite(x.h)).map(x=>{
   const t=Date.parse(x.utc);
   return {s:parseInt(x.s,10)||0,t:isFinite(t)?t:null,src:x.src||'?',lat:x.lat,lon:x.lon,h:x.h,hacc:x.hacc,note:x.note};
  });
  renderPrev();
  $('prevMsg').textContent=prevRows.length?(prevRows.length+' saved position(s) in the logbook.'):'No saved positions in the logbook yet.';
  applyLogTimes();
 }catch(e){ $('prevMsg').textContent='Could not read the logbook.'; }
 finally{ prevBusy=false; $('prevBtn').disabled=false; }
}
// ── Device rename ─────────────────────────────────────────────────────────────
// The name becomes the AP's SSID, so it carries 802.11's 31-character ceiling and its
// printable-ASCII restriction. Both are enforced on the device in setDeviceName(); they
// are repeated here so a too-long name is refused while it is still in front of the
// operator rather than coming back as an error after the round trip.
const DEVICE_NAME_MAX=31;
async function renameDevice(){
 const cur=$('dev').textContent||'';
 const name=(prompt('New device name — max '+DEVICE_NAME_MAX+
   ' characters; shown at once on this page and the LCD, used as the WiFi network name after a reboot:',cur)||'').trim();
 if(!name) return;
 if(name.length>DEVICE_NAME_MAX){
  alert('Too long: '+name.length+' characters. The AP SSID limit is '+DEVICE_NAME_MAX+'.');return;}
 if(!/^[\x20-\x7E]+$/.test(name)){
  alert('Use printable ASCII only — other characters are not portable across WiFi clients.');return;}
 try{
  const r=await fetch('/api/setname',{method:'POST',headers:{'Content-Type':'application/x-www-form-urlencoded'},
   body:'name='+encodeURIComponent(name)});
  if(r.ok){ $('dev').textContent=name; document.title=name; alert('Saved. The WiFi network name changes on the next reboot.'); }
  else{const t=await r.text();alert('Error: '+t);}
 }catch(e){alert('Request failed');}
}
// ── WiFi management ──────────────────────────────────────────────────────────
let wifiListBusy=false, wifiListAgain=false;   // one request at a time, as for poll()
async function loadWifiList(){
 if(wifiListBusy){ wifiListAgain=true; return; }
 wifiListBusy=true;
 try{ do{ wifiListAgain=false; await loadWifiListOnce(); }while(wifiListAgain); } finally{ wifiListBusy=false; }
}
async function loadWifiListOnce(){
 try{
  const r=await fetchT('/api/wifilist',{cache:'no-store'},5000);
  const d=await r.json();
  const nets=d.networks||[];
  if(!nets.length){$('wifiList').innerHTML='<div class="muted">No networks configured</div>';return;}
  $('wifiList').innerHTML=nets.map(n=>{
   const active=n.active?'<span class="good"> ✓ connected</span>':'';
   const del='<button class="btn sec" style="padding:2px 8px;margin-left:8px" data-s="'+esc(n.ssid)+'" onclick="delWifi(this.dataset.s)" title="Remove">✕</button>';
   return '<div class="row" style="align-items:center"><span>'+esc(n.ssid)+active+'</span>'+del+'</div>';
  }).join('');
 }catch(e){$('wifiList').innerHTML='<div class="muted">unavailable</div>';}
}
async function addWifi(connect){
 const ssid=($('wfSsid').value||'').trim();
 const pw=$('wfPw').value||'';
 if(!ssid){$('wifiMsg').textContent='SSID required';return;}
 $('wifiMsg').textContent='Adding…';
 try{
  const r=await fetch('/api/addwifi',{method:'POST',headers:{'Content-Type':'application/x-www-form-urlencoded'},
   body:'ssid='+encodeURIComponent(ssid)+'&pw='+encodeURIComponent(pw)+'&connect='+(connect?'1':'0')});
  if(r.ok){$('wfSsid').value='';$('wfPw').value='';$('wfPw').type='password';$('wfPwBtn').textContent='Show';$('wifiMsg').textContent=connect?'Saved — connecting…':'Saved.';loadWifiList();}
  else{const t=await r.text();$('wifiMsg').textContent='Error: '+t;}
 }catch(e){$('wifiMsg').textContent='Request failed';}
}
async function delWifi(ssid){
 if(!confirm('Remove "'+ssid+'" from saved networks?'))return;
 try{
  const r=await fetch('/api/delwifi',{method:'POST',headers:{'Content-Type':'application/x-www-form-urlencoded'},
   body:'ssid='+encodeURIComponent(ssid)});
  if(r.ok){$('wifiMsg').textContent='Removed.';loadWifiList();}
  else{const t=await r.text();$('wifiMsg').textContent='Error: '+t;}
 }catch(e){$('wifiMsg').textContent='Request failed';}
}
loadWifiList();setInterval(loadWifiList,10000);
// ── Caster management ────────────────────────────────────────────────────────
async function casterEnable(host,mount,en){
 try{
  const r=await fetch('/api/casteren',{method:'POST',headers:{'Content-Type':'application/x-www-form-urlencoded'},
   body:'host='+encodeURIComponent(host)+'&mount='+encodeURIComponent(mount)+'&en='+en});
  if(!r.ok){const t=await r.text();$('casterMsg').textContent='Error: '+t;}
  poll();
 }catch(e){$('casterMsg').textContent='Request failed';}
}
// Ask the device to check the caster's live source table for this mount name. Every call
// opens a real socket to the caster, so it waits for the operator to actually stop typing
// rather than firing part-way through a name: a four-character Centipede mount typed at
// any normal speed would otherwise generate three or four connections, each of which
// looks like a probe from the caster's side and each of which reports a partial name as
// "available". Two seconds is comfortably longer than an inter-keystroke gap and short
// enough to still feel like live feedback.
const MOUNT_CHECK_IDLE_MS=2000;
// The device refuses a check that starts within 1.5 s of the previous one ending, so a
// check is never sent while one is running, nor less than MOUNT_CHECK_GAP_MS after it.
const MOUNT_CHECK_GAP_MS=1600;
let mountCheckTimer=null, mountCheckBusy=false, mountCheckEndMs=0;
function mountCheckSoon(){
 clearTimeout(mountCheckTimer);
 const w=$('mountWarn');
 if(w){ w.className='muted'; w.textContent=''; }
 mountCheckTimer=setTimeout(mountCheckWhenFree,MOUNT_CHECK_IDLE_MS);
}
function mountCheckWhenFree(){
 const wait=mountCheckBusy?500:Math.max(0,mountCheckEndMs+MOUNT_CHECK_GAP_MS-Date.now());
 if(wait>0){ clearTimeout(mountCheckTimer); mountCheckTimer=setTimeout(mountCheckWhenFree,wait); return; }
 mountCheckBusy=true;
 mountCheck().finally(()=>{ mountCheckBusy=false; mountCheckEndMs=Date.now(); });
}
async function mountCheck(){
 const w=$('mountWarn'); if(!w) return;
 const host=$('cHost').value.trim(), mount=$('cMount').value.trim(), port=$('cPort').value||2101;
 w.className='muted'; w.textContent='';
 if(!host||!mount) return;
 w.textContent='Checking '+host+' for "'+mount+'"…';
 try{
  const r=await fetch('/api/mountcheck?host='+encodeURIComponent(host)+'&port='+encodeURIComponent(port)+
                      '&mount='+encodeURIComponent(mount),{cache:'no-store'});
  const j=await r.json();
  if(!j.ok){ w.className='muted'; w.textContent='Could not check: '+(j.err||'unknown'); return; }
  if(j.inuse){
   w.className='bad'; w.style.display='inline-block';
   w.textContent='Mount name "'+mount+'" is already in use on '+host+'. Choose another.';
  }else{
   w.className='good'; w.style.display='inline-block';
   w.textContent='"'+mount+'" is not currently in use on '+host+'.';
  }
 }catch(e){ w.className='muted'; w.textContent='Could not check the caster.'; }
}
async function casterAdd(){
 const host=($('cHost').value||'').trim();
 const mount=($('cMount').value||'').trim();
 const port=($('cPort').value||'2101').trim();
 const pw=$('cPw').value||'';
 if(!host||!mount){$('casterMsg').textContent='Host and mount required';return;}
 // Centipede rejects anything but a 4-character A-Z/0-9 mount name; catch it here
 // rather than after a failed handshake. Other casters set their own rules.
 if(/(^|\.)crtk\.net$/i.test(host)&&!/^[A-Z0-9]{4}$/.test(mount)){
  $('casterMsg').textContent='Centipede mount must be exactly 4 characters, A-Z or 0-9.';return;}
 $('casterMsg').textContent='Adding…';
 try{
  const r=await fetch('/api/casteradd',{method:'POST',headers:{'Content-Type':'application/x-www-form-urlencoded'},
   body:'host='+encodeURIComponent(host)+'&port='+encodeURIComponent(port)+'&mount='+encodeURIComponent(mount)+'&pw='+encodeURIComponent(pw)});
  if(r.ok){$('cHost').value='';$('cMount').value='';$('cPw').value='';$('cPw').type='text';$('cPwBtn').textContent='Hide';casterDefaultsSet=false;$('casterMsg').textContent='Added.';poll();}
  else{const t=await r.text();$('casterMsg').textContent='Error: '+t;}
 }catch(e){$('casterMsg').textContent='Request failed';}
}
async function casterDel(host,mount){
 if(!confirm('Remove caster "'+host+'/'+mount+'"?'))return;
 try{
  const r=await fetch('/api/casterdel',{method:'POST',headers:{'Content-Type':'application/x-www-form-urlencoded'},
   body:'host='+encodeURIComponent(host)+'&mount='+encodeURIComponent(mount)});
  if(r.ok){$('casterMsg').textContent='Removed.';poll();}
  else{const t=await r.text();$('casterMsg').textContent='Error: '+t;}
 }catch(e){$('casterMsg').textContent='Request failed';}
}
</script></body></html>)HTML";
