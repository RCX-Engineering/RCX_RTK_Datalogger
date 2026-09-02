/*
 * ppp_web.cpp — see ppp_web.h. Standalone PPP status page (served from PROGMEM)
 * plus the JSON/control endpoints. Styled to match the RCX1 dashboard theme.
 */

#include "ppp_web.h"

static WebServer*      sServer = nullptr;
static HardwareSerial* sGnss   = nullptr;

// Self-contained status page. Vanilla JS, no external fetches. Polls /api/ppp.
static const char PPP_PAGE[] PROGMEM = R"HTML(<!DOCTYPE html>
<html lang="en"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>RCX1 &middot; PPP Survey</title>
<style>
:root{--bg:#0f1115;--card:#1a1d24;--line:#2a2f3a;--ink:#e7ebf0;--mut:#8b93a3;
--good:#3fb950;--warn:#d29922;--bad:#f85149;--accent:#388bfd}
*{box-sizing:border-box}
body{margin:0;font:14px/1.45 system-ui,sans-serif;background:var(--bg);color:var(--ink)}
header{display:flex;align-items:center;gap:10px;padding:14px 18px;border-bottom:1px solid var(--line)}
header h1{font-size:16px;margin:0;font-weight:600}
a{color:var(--accent);text-decoration:none}
.wrap{max-width:620px;margin:0 auto;padding:16px;display:grid;gap:14px}
.card{background:var(--card);border:1px solid var(--line);border-radius:10px;padding:14px}
.card h2{margin:0 0 10px;font-size:12px;letter-spacing:.08em;text-transform:uppercase;color:var(--mut)}
.row{display:flex;justify-content:space-between;gap:8px;padding:4px 0;border-bottom:1px dashed var(--line)}
.row:last-child{border-bottom:0}
.row .k{color:var(--mut)}
.row .v{font-variant-numeric:tabular-nums;text-align:right}
.good{color:var(--good)}.warn{color:var(--warn)}.bad{color:var(--bad)}
.dot{display:inline-block;width:10px;height:10px;border-radius:50%;background:var(--bad);
box-shadow:0 0 8px currentColor;vertical-align:middle;margin-right:6px}
.dot.on{background:var(--good)}
.pill{display:inline-block;padding:2px 10px;border-radius:20px;font-size:12px;background:#222836}
.bar{height:8px;border-radius:6px;background:#0d1017;border:1px solid var(--line);overflow:hidden;margin-top:6px}
.bar>i{display:block;height:100%;width:0;background:var(--accent);transition:width .4s}
button{font:inherit;background:var(--accent);color:#fff;border:0;border-radius:6px;padding:8px 12px;cursor:pointer}
button.sec{background:#2a2f3a}
input{font:inherit;background:#0d1017;color:var(--ink);border:1px solid var(--line);border-radius:6px;padding:7px;width:110px}
</style></head><body>
<header><span class="dot" id="e6dot"></span><h1>PPP Survey (E6 HAS)</h1>
<span style="flex:1"></span><a href="/">&larr; dashboard</a></header>
<div class="wrap">
 <div class="card">
  <div class="row"><span class="k">State</span><span class="v"><span id="state" class="pill">&ndash;</span></span></div>
  <div class="bar"><i id="bar"></i></div>
  <div class="row" style="margin-top:8px"><span class="k">Progress</span><span class="v" id="prog">&ndash;</span></div>
  <div class="row"><span class="k"><span class="dot" id="e6dot2"></span>E6 sats / avg C/N0</span><span class="v" id="e6">0 / &ndash;</span></div>
  <div class="row"><span class="k">EPE 2D / 3D (m)</span><span class="v" id="epe">&ndash; / &ndash;</span></div>
  <div class="row"><span class="k">Best 2D / 3D (m)</span><span class="v" id="best">&ndash;</span></div>
  <div class="row"><span class="k">Acceptance limits (m)</span><span class="v" id="lim">&ndash;</span></div>
  <div class="row"><span class="k">Converged samples</span><span class="v" id="samples">0</span></div>
  <div class="k" id="gateNote" style="margin-top:6px"></div>
  <div class="row"><span class="k">Valid fixes</span><span class="v" id="valid">0</span></div>
  <div class="row"><span class="k">Mean position</span><span class="v" id="pos">&ndash;</span></div>
  <div class="row"><span class="k">Locked ECEF</span><span class="v" id="ecef">&ndash;</span></div>
 </div>
 <div class="card">
  <h2>Control</h2>
  <div style="display:flex;gap:8px;align-items:center">
   <input id="dur" type="text" inputmode="numeric" value="00:15:00" autocomplete="off">
   <span class="k">hh:mm:ss</span>
   <button onclick="pppStart()">Start PPP survey</button>
   <button class="sec" onclick="pppAbort()">Abort</button>
  </div>
  <div class="k" id="durNote" style="margin-top:6px"></div>
  <p class="k" style="margin:10px 0 0">Red E6 light = weak/no HAS corrections here. PPP won't
  converge &mdash; abort and run a conventional survey-in instead.</p>
 </div>
</div>
<script>
const f=(x,d=2)=>x==null?'\u2013':Number(x).toFixed(d);
// hh:mm:ss <-> seconds, the same pair the main dashboard uses, so a duration means the
// same thing on both pages. Every survey window on this device is entered and displayed
// in this form; a units-ambiguous field here is a whole survey window spent on a value
// that was read differently than it was meant. A bare number is accepted as SECONDS, so
// the habit of typing 3600 still works.
function durToSec(v){
 const str=String(v==null?'':v).trim();
 if(!str) return NaN;
 if(!/^\d+(:\d{1,2}){0,2}$/.test(str)) return NaN;
 const p=str.split(':').map(x=>parseInt(x,10));
 if(p.some(x=>isNaN(x)||x<0)) return NaN;
 if(p.length===1) return p[0];
 if(p.length===2) return p[0]*60+p[1];
 return p[0]*3600+p[1]*60+p[2];
}
function secToDur(s){s=Math.max(0,Math.floor(+s||0));
 const h=Math.floor(s/3600),m=Math.floor(s%3600/60),x=s%60;
 return (h<10?'0':'')+h+':'+(m<10?'0':'')+m+':'+(x<10?'0':'')+x;}
async function poll(){
 try{
  const p=await (await fetch('/api/ppp')).json();
  const st=document.getElementById('state');st.textContent=p.state;
  st.className='pill '+(p.state=='DONE'?'good':p.state=='FAILED'?'bad':p.state=='SURVEYING'?'warn':'');
  const pct=p.dur?Math.min(100,Math.floor(100*p.elapsed/p.dur)):0;
  document.getElementById('bar').style.width=pct+'%';
  document.getElementById('prog').textContent=p.dur?(secToDur(p.elapsed)+' / '+secToDur(p.dur)+' ('+pct+'%)'):'\u2013';
  const on=p.e6ok?' on':'';
  document.getElementById('e6dot').className='dot'+on;
  document.getElementById('e6dot2').className='dot'+on;
  document.getElementById('e6').textContent=p.e6sats+' / '+f(p.e6cnr,1);
  document.getElementById('epe').textContent=f(p.epe2d)+' / '+f(p.epe3d)+(p.conv?' \u2713':'');
  document.getElementById('best').textContent=f(p.best2d)+' / '+f(p.best3d);
  document.getElementById('lim').textContent=f(p.lim2d)+' / '+f(p.lim3d);
  // A survey that ends with too few accepted samples locks the unfiltered autonomous mean
  // instead of the converged one, and the difference is invisible unless the reason is
  // stated. Naming which gate is doing the rejecting turns "PPP isn't working" into a
  // number to act on.
  const gn=document.getElementById('gateNote');
  if(p.best2d==null||p.best3d==null){ gn.textContent='No EPE reported yet.'; }
  else if(p.samples>=p.need){ gn.textContent='Converged set is large enough to lock.'; }
  else {
   const b2=p.best2d>p.lim2d, b3=p.best3d>p.lim3d;
   const scatOk = p.scat!=null && p.scat<=p.scatlim;
   gn.textContent = scatOk
     ? ('Converged on measured scatter ('+f(p.scat,3)+' m) — '+p.samples+' of '+p.need+' needed.')
     : ((b2||b3)
        ? ('Rejecting every fix: best '+(b2?('2D '+f(p.best2d)+' m > '+f(p.lim2d)):'')+
           (b2&&b3?' and ':'')+(b3?('3D '+f(p.best3d)+' m > '+f(p.lim3d)):'')+
           ' m, and scatter '+(p.scat==null?'not yet measured':f(p.scat,3)+' m > '+f(p.scatlim)+' m')+'.')
        : ('Accepting fixes — '+p.samples+' of '+p.need+' needed.'));
  }
  document.getElementById('samples').textContent=p.samples;
  document.getElementById('valid').textContent=p.valid;
  document.getElementById('pos').textContent=p.lat==null?'\u2013':(f(p.lat,9)+', '+f(p.lon,9));
  document.getElementById('ecef').textContent=p.x==null?'\u2013':(f(p.x,2)+', '+f(p.y,2)+', '+f(p.z,2));
 }catch(e){}
}
setInterval(poll,1500);poll();
function pppStart(){
 const note=document.getElementById('durNote');
 const sec=durToSec(document.getElementById('dur').value);
 // Echo the interpreted duration back, so a value that was read differently than it was
 // meant is obvious before a survey window has been spent on it.
 if(isNaN(sec)||sec<30||sec>86400){
  note.textContent='Enter a duration between 00:00:30 and 24:00:00.';return;}
 document.getElementById('dur').value=secToDur(sec);
 note.textContent='Starting a '+secToDur(sec)+' survey\u2026';
 fetch('/api/pppstart',{method:'POST',headers:{'Content-Type':'application/x-www-form-urlencoded'},body:'sec='+sec});}
function pppAbort(){fetch('/api/pppabort',{method:'POST'});}
</script></body></html>)HTML";

// ── Handlers (reference the stored server/gnss) ──────────────────────────────
static void ppp_handle_page()  { if (sServer) sServer->send_P(200, "text/html", PPP_PAGE); }

static void ppp_handle_json()  {
    // Grown for the acceptance-limit and best-2D fields. ppp_survey_status_json() blanks
    // the whole buffer rather than emitting a truncated object, so an undersized buffer
    // shows as an empty page instead of a parse error.
    char buf[640];
    ppp_survey_status_json(buf, sizeof buf);
    if (sServer) sServer->send(200, "application/json", buf);
}

static void ppp_handle_start() {
    // Same bounds the dashboard enforces, applied again here because the endpoint is
    // reachable without it. 0 falls through to the module default in ppp_survey_begin();
    // anything outside 30 s .. 24 h is a typo rather than an intent.
    const long raw = (sServer && sServer->hasArg("sec")) ? sServer->arg("sec").toInt() : 0;
    if (raw != 0 && (raw < 30 || raw > 86400)) {
        if (sServer) sServer->send(400, "text/plain", "duration must be 30..86400 s");
        return;
    }
    if (sGnss) ppp_survey_begin(*sGnss, (uint32_t)raw);
    if (sServer) sServer->send(200, "text/plain", "ok");
}

static void ppp_handle_abort() {
    ppp_survey_abort();
    if (sServer) sServer->send(200, "text/plain", "ok");
}

void ppp_web_register(WebServer& server, HardwareSerial& gnss)
{
    sServer = &server;
    sGnss   = &gnss;
    server.on("/ppp",          HTTP_GET,  ppp_handle_page);
    server.on("/api/ppp",      HTTP_GET,  ppp_handle_json);
    server.on("/api/pppstart", HTTP_POST, ppp_handle_start);
    server.on("/api/pppabort", HTTP_POST, ppp_handle_abort);
}
