const conn = document.getElementById('conn');
const sentEl = document.getElementById('sent');
const gpEl = document.getElementById('gp');
let gait = 0, armed = false, dancePaused = false, lastInput = 0;
let activeView = 'drive';  // drive | motors | demos | rl | calibrate | debug
let calAxis = 'all';
let calTimer = null;
let selJoint = null, selSid = null;
let motorsTimer = null;
let linkOk = null;           // null=unknown, true/false after first ping
let linkFailStreak = 0;
let lastPingOkAt = 0;
let backendKind = 'robot';   // robot | sim
let hubMode = false;
let hubTarget = 'robot';
let targetHasRobot = true;
let targetHasSim = false;
let robotTargetAvailable = true;
let simTargetAvailable = false;
let robotTargetTransient = false;
let robotTargetUrl = '';
let targetLineMsg = {robot:null, sim:null};
let lastRobotState = null;
let lastFeedback = null;
let lastTargetHealthMsg = '';
let targetReconnectSince = {robot:0, sim:0};
const TARGET_RESTART_GRACE_MS = 15000;
let simFrames = true;
let simNativeViewer = false;
let simTimer = null, simBusy = false, simFrameBusy = false;
let simFrameLastAt = 0;
// SERVO ARM GATE (separate from `armed`, which just means "a stick is pushed").
// Defaults OFF on every page load and the firmware boots DISARMED, so nothing
// drives a servo until the human presses Enable. All servo-driving sends are
// gated on this flag; ARM/DISARM/E-stop are the only power controls.
let servosArmed = false;
let maxVx = 40, maxVy = 29, maxOmega = 0.7;

function savedRobotUrl(){
  try{ return localStorage.getItem('hexapod.robotUrl') || ''; }
  catch(e){ return ''; }
}
function saveRobotUrl(url){
  try{
    if(url) localStorage.setItem('hexapod.robotUrl', url);
    else localStorage.removeItem('hexapod.robotUrl');
  }catch(e){}
}
function htmlEscape(x){
  return String(x == null ? '' : x).replace(/[&<>"']/g, ch => ({
    '&':'&amp;', '<':'&lt;', '>':'&gt;', '"':'&quot;', "'":'&#39;',
  }[ch]));
}

// --- link heartbeat --------------------------------------------------------
function setLink(ok, detail){
  linkOk = !!ok;
  document.body.classList.toggle('link-down', !ok);
  if(ok){
    linkFailStreak = 0;
    lastPingOkAt = Date.now();
    conn.textContent = detail || 'online';
    conn.className = 'ok';
  } else {
    conn.textContent = detail || 'offline';
    conn.className = 'bad';
    const bar = document.getElementById('offlinebar');
    const target = hubMode ? (hubTarget === 'both'
      ? 'robot + MuJoCo sim' : hubTarget)
      : (backendKind === 'sim' ? 'MuJoCo sim' : 'Uno Q');
    if(bar) bar.textContent = `● Lost connection to ${target} — retrying…`
      +(detail ? (' ('+detail+')') : '');
  }
}
function isRobotRestartLikeError(text){
  const low = String(text || '').toLowerCase();
  return /robot proxy failed/.test(low)
    && /(connection refused|errno 61|connection reset|connection aborted|temporarily unavailable|remote end closed)/.test(low);
}
function targetHealthItems(meta){
  if(!meta || !meta.hub || meta.ok !== false) return [];
  const targets = meta.targets || {};
  const active = meta.active || {};
  const bad = [];
  [['robot', 'robot', targets.robot || {}],
   ['sim', 'MuJoCo sim', targets.sim || {}]]
    .forEach(([key, name, t])=>{
      if(active[key] && t.available && t.ok === false)
        bad.push({
          key,
          name,
          error: t.error || 'not responding',
          transient: key === 'robot' && isRobotRestartLikeError(t.error),
        });
    });
  return bad;
}
function targetHealthMsg(meta){
  const bad = targetHealthItems(meta);
  return bad.length
    ? bad.map(t=> t.name + ': ' + t.error).join(' · ')
    : 'target not responding';
}
function requestReceiptLine(d, label){
  const prefix = label ? label + ' received' : 'request received';
  const h = d && d.hub;
  if(!h || typeof h !== 'object' || !h.robot || !h.sim){
    if(d && (d.sim || (d.robot && d.robot.sim))) return prefix + ' — MuJoCo OK';
    if(d && d.robot && d.robot.ok !== false) return prefix + ' — robot OK';
    return prefix;
  }
  const part = (name, x)=>{
    const ok = x && x.ok !== false && !x.error;
    return name + (ok ? ' OK' : ' failed'
      + (x && x.error ? ': '+x.error : ''));
  };
  return prefix + ' — ' + part('robot', h.robot) + ' · '
    + part('MuJoCo', h.sim);
}
function responseHomeLabel(d){
  return (d && d.params && d.params.home) ? d.params.home : (d && d.home);
}
function applyBackendMeta(meta){
  if(!meta) return;
  const prevBackendKind = backendKind;
  const prevHubMode = hubMode;
  const prevHubTarget = hubTarget;
  const prevTargetHasRobot = targetHasRobot;
  const prevTargetHasSim = targetHasSim;
  const prevFrames = simFrames;
  const prevNativeViewer = simNativeViewer;
  hubMode = !!meta.hub || meta.service === 'hexapod-hub';
  if(hubMode){
    const targets = meta.targets || {};
    const robotMeta = targets.robot || {};
    const simMeta = targets.sim || {};
    hubTarget = meta.target || 'sim';
    targetHasRobot = !!(meta.active && meta.active.robot);
    targetHasSim = !!(meta.active && meta.active.sim);
    robotTargetTransient = !!(targetHasRobot && robotMeta.available
      && robotMeta.ok === false && isRobotRestartLikeError(robotMeta.error));
    robotTargetAvailable = !!robotMeta.available && robotMeta.ok !== false;
    simTargetAvailable = !!simMeta.available && simMeta.ok !== false;
    robotTargetUrl = robotMeta.url || robotTargetUrl || savedRobotUrl();
  } else {
    const kind0 = meta.kind
      || (meta.service === 'hexapod-sim' ? 'sim' : 'robot');
    targetHasSim = kind0 === 'sim';
    targetHasRobot = kind0 !== 'sim';
    robotTargetAvailable = targetHasRobot;
    simTargetAvailable = targetHasSim;
    robotTargetTransient = false;
    robotTargetUrl = '';
    hubTarget = targetHasSim ? 'sim' : 'robot';
  }
  const kind = targetHasRobot ? 'robot' : (targetHasSim ? 'sim' : 'robot');
  const frames = meta.frames !== false;
  const nativeViewer = !!meta.viewer;
  const changed = kind !== prevBackendKind || frames !== prevFrames
    || nativeViewer !== prevNativeViewer || hubMode !== prevHubMode
    || hubTarget !== prevHubTarget
    || targetHasRobot !== prevTargetHasRobot
    || targetHasSim !== prevTargetHasSim;
  backendKind = kind;
  simFrames = frames;
  simNativeViewer = nativeViewer;
  document.body.classList.toggle('hub-backend', hubMode);
  document.body.classList.toggle('target-both', hubTarget === 'both');
  document.body.classList.toggle('robot-active', targetHasRobot);
  document.body.classList.toggle('sim-active', targetHasSim);
  document.body.classList.toggle('robot-configured',
    hubMode && robotTargetAvailable);
  document.body.classList.toggle('sim-configured',
    hubMode && simTargetAvailable);
  document.body.classList.toggle('sim-backend', targetHasSim);
  document.body.classList.toggle('sim-native-viewer',
    targetHasSim && simNativeViewer);
  document.body.classList.toggle('sim-browser-frames',
    targetHasSim && simFrames);
  const robotInput = document.getElementById('roboturl');
  if(robotInput && robotTargetUrl
      && document.activeElement !== robotInput
      && robotInput.value !== robotTargetUrl)
    robotInput.value = robotTargetUrl;
  if(!simFrames){
    const img = document.getElementById('simframe');
    if(img) img.removeAttribute('src');
    simFrameBusy = false;
  }
  if(changed){
    updateArmUI();
    rlPaintReadinessDefault();
    if(activeView === 'rl') simPollMaybe();
  }
  paintTargetRows();
}
async function heartbeat(){
  const ac = new AbortController();
  // 5 s budget: macOS mDNS re-resolution of hexapod.local can stall a
  // request ~5 s (measured 08-11); a 2 s abort turned every stall into
  // a false "lost connection" flap.
  const t = setTimeout(()=> ac.abort(), 5000);
  try{
    const r = await fetch('/api/ping?t='+Date.now(), {
      cache:'no-store', signal: ac.signal});
    clearTimeout(t);
    if(!r.ok) throw new Error('HTTP '+r.status);
    const j = await r.json().catch(()=>({}));
    applyBackendMeta(j);
    if(j && j.ok === false && j.hub){
      setLink(true, 'online');
      const items = targetHealthItems(j);
      const loud = [];
      const now = Date.now();
      items.forEach(item=>{
        if(item.transient){
          if(!targetReconnectSince[item.key])
            targetReconnectSince[item.key] = now;
          setTargetLineMsg(item.key, 'robot web restarting… reconnecting',
            'warn');
          if(now - targetReconnectSince[item.key]
              > TARGET_RESTART_GRACE_MS)
            loud.push(item.name + ': ' + item.error);
        } else {
          targetReconnectSince[item.key] = 0;
          setTargetLineMsg(item.key, item.error, 'bad');
          loud.push(item.name + ': ' + item.error);
        }
      });
      const msg = loud.join(' · ');
      if(msg && msg !== lastTargetHealthMsg){
        lastTargetHealthMsg = msg;
        showSent(msg, true);
      }
      return;
    }
    if(j && j.ok === false) throw new Error(j.error || 'ping failed');
    if(hubMode){
      if(robotTargetAvailable){
        targetReconnectSince.robot = 0;
        clearTargetLineMsg('robot');
      }
      if(simTargetAvailable){
        targetReconnectSince.sim = 0;
        clearTargetLineMsg('sim');
      }
    }
    lastTargetHealthMsg = '';
    setLink(true, 'online');
  }catch(e){
    clearTimeout(t);
    linkFailStreak++;
    // Require 2 misses so a single blip doesn't flash the banner.
    if(linkFailStreak >= 2 || linkOk === null)
      setLink(false, e && e.name==='AbortError' ? 'timeout' : 'no response');
    else {
      conn.textContent = 'reconnecting…';
      conn.className = 'warn';
    }
  }
}
// Non-overlapping loop: with a 5 s worst case a fixed 1.5 s interval
// would stack concurrent pings during a stall.
(async function hbLoop(){
  while(true){
    await heartbeat();
    await new Promise(r=> setTimeout(r, 1500));
  }
})();
heartbeat();
document.addEventListener('visibilitychange', ()=>{
  if(document.visibilityState === 'visible') heartbeat();
});

// --- command transport -----------------------------------------------------
async function cmd(line, opts){
  const headers = {};
  if(opts && opts.globalStop) headers['X-Hexapod-Global-Stop'] = '1';
  try {
    const r = await fetch('/cmd', {method:'POST', body:line, headers});
    if(!r.ok) throw 0;
    setLink(true);
  } catch(e){ setLink(false, 'cmd failed'); }
}
// Persistent, copyable last-error bar. The header #sent blip is tiny,
// transient and unselectable on a phone (operator request 08-10).
const errbarEl = document.getElementById('errbar');
const errbarText = document.getElementById('errbar-text');
function showErr(line){
  // Only touch the DOM when the text actually changes — rewriting
  // textContent on every status poll destroyed the user's text
  // selection mid-highlight (operator 08-11: "can't copy the error").
  if(errbarText.textContent !== String(line))
    errbarText.textContent = String(line);
  if(errbarEl.style.display !== 'flex')
    errbarEl.style.display = 'flex';
}
document.getElementById('errbar-close').onclick =
  ()=>{ errbarEl.style.display = 'none'; };
document.getElementById('errbar-copy').onclick = async ()=>{
  const t = errbarText.textContent;
  try{
    await navigator.clipboard.writeText(t);   // needs https / localhost
  }catch(e){
    const ta = document.createElement('textarea');   // http:// fallback
    ta.value = t; document.body.appendChild(ta);
    ta.select(); document.execCommand('copy'); ta.remove();
  }
  const b = document.getElementById('errbar-copy');
  b.textContent = 'Copied ✓';
  setTimeout(()=>{ b.textContent = 'Copy'; }, 1200);
};
// Header status panel copy button: grabs every status line (link state,
// robot activity, controller, last command) as labelled text — for pasting
// into a chat/issue without screenshotting the corner of the screen.
document.getElementById('statuscopy').onclick = async ()=>{
  const parts = [
    'link: '+conn.textContent,
    'robot: '+document.getElementById('robotact').textContent];
  const gpT = gpEl.textContent.trim();
  if(gpT) parts.push('controller: '+gpT);
  const sentT = sentEl.textContent.trim();
  if(sentT) parts.push('last: '+sentT);
  const rconT = (document.getElementById('rcon-lines')?.innerText || '').trim();
  if(rconT) parts.push('robot console:\n'+rconT);
  const t = parts.join('\n');
  try{
    await navigator.clipboard.writeText(t);   // needs https / localhost
  }catch(e){
    const ta = document.createElement('textarea');   // http:// fallback
    ta.value = t; document.body.appendChild(ta);
    ta.select(); document.execCommand('copy'); ta.remove();
  }
  const b = document.getElementById('statuscopy');
  b.textContent = '✓';
  setTimeout(()=>{ b.textContent = 'Copy'; }, 1200);
};
function isOkReceipt(line){
  return /received\s+—.*\bOK\b/i.test(String(line || ''));
}
function showSent(line, isErr){
  sentEl.textContent = line;
  const text = String(line || '');
  const looksBad = /refus|fail|error|not ready|missing|timeout|no bus|unknown|denied|abort/i
    .test(text);
  if(isErr || (looksBad && !isOkReceipt(text)))
    showErr(line);
  else if(isOkReceipt(text) && isOkReceipt(errbarText.textContent))
    errbarEl.style.display = 'none';
}

// Sticky robot console: small, always-visible breadcrumbs from the robot's
// event stream. It intentionally filters heartbeat/polling/MCU register spam
// so failures like "button fired but backend refused" are visible immediately.
const rconState = document.getElementById('rcon-state');
const rconDrive = document.getElementById('rcon-drive');
const rconServos = document.getElementById('rcon-servos');
const rconDetail = document.getElementById('rcon-detail');
const rconLines = document.getElementById('rcon-lines');
const rconCopy = document.getElementById('rcon-copy');
let robotConsoleText = '';
function rconSetChip(el, text, cls=''){
  if(!el) return;
  el.textContent = text;
  el.className = 'robot-console-chip' + (cls ? ' '+cls : '');
}
function rconTime(ts){
  try{
    return new Date(ts).toLocaleTimeString([], {
      hour:'2-digit', minute:'2-digit', second:'2-digit', hour12:false,
    });
  }catch(e){ return '--:--:--'; }
}
function rconShort(s, n=180){
  s = String(s == null ? '' : s).replace(/\s+/g, ' ').trim();
  return s.length > n ? s.slice(0, n-1)+'…' : s;
}
function rconBodySummary(body){
  if(body == null || body === '') return '';
  if(typeof body === 'string') return body;
  if(typeof body !== 'object') return String(body);
  const bits = [];
  for(const k of ['mode','direction','vx','vy','wz','speed','button','status']){
    if(body[k] != null) bits.push(k+'='+body[k]);
  }
  return bits.length ? bits.join(' ') : JSON.stringify(body).slice(0, 140);
}
function rconSkipEvent(e){
  const msg = String(e && e.msg || '');
  const kind = String(e && e.kind || '');
  const src = String(e && e.src || '');
  if(kind === 'mcu' || kind === 'servo_fb') return true;
  if(msg === 'SCAN' || /^W[12]\b/.test(msg)) return true;
  if(src === 'http' && kind === 'cmd'){
    if(/^GET \/api\/(robot|events|errors|ping|calibrate|rl\/drive|rl\/roles|rl\/policies)\b/.test(msg))
      return true;
    if(/^GET \/(app\.js|style\.css|favicon\.svg)/.test(msg)) return true;
  }
  return false;
}
function rconInterestingEvent(e){
  if(!e || rconSkipEvent(e)) return false;
  const kind = String(e.kind || '');
  const src = String(e.src || '');
  const msg = String(e.msg || '');
  if(e.level === 'error' || kind === 'error') return true;
  if(kind === 'ui' || kind === 'standup') return true;
  if(kind === 'log' && src === 'print') return true;
  if(src === 'bench') return true;
  if(kind === 'cmd' && /^POST /.test(msg)) return true;
  return false;
}
function rconEventClass(e){
  if(e.level === 'error' || e.kind === 'error') return 'error';
  if(e.kind === 'ui') return 'ui';
  if(e.kind === 'standup' || e.kind === 'log') return 'ok';
  return '';
}
function rconEventText(e){
  const data = e.data || {};
  let msg = String(e.msg || e.kind || 'event');
  const body = rconBodySummary(data.body || data);
  if(e.kind === 'ui' && data.status) msg += ' · '+data.status;
  else if(e.kind === 'cmd' && body) msg += ' · '+body;
  else if(e.kind === 'error' && data.error && !msg.includes(data.error))
    msg += ' · '+data.error;
  return `${rconTime(e.ts)} ${e.kind || 'event'} ${e.src || ''}: ${rconShort(msg)}`;
}
function paintRobotConsole(robot, drive, events){
  if(robot){
    const act = robot.activity || (robot.armed ? 'armed' : 'limp');
    rconSetChip(rconState,
      (robot.armed ? 'ARMED' : 'LIMP')+' · '+act,
      robot.armed ? 'ok' : 'bad');
    const servo = robot.servo || {};
    const live = servo.live != null ? servo.live : '?';
    const exp = servo.expected != null ? servo.expected : 18;
    const servoOk = servo.ok && servo.imu_ok && !((servo.missing||[]).length);
    const hot = (servo.hot || []).length || (servo.tripped || []).length;
    rconSetChip(rconServos,
      `${live}/${exp} servos`+(servo.imu_ok ? ' · IMU' : ' · no IMU'),
      servoOk && !hot ? 'ok' : (servoOk ? 'warn' : 'bad'));
  } else {
    rconSetChip(rconState, 'robot ?', 'warn');
    rconSetChip(rconServos, 'servos ?', 'warn');
  }
  if(drive && drive.ok){
    rconSetChip(rconDrive, drive.active ? 'drive ACTIVE' : 'drive inactive',
      drive.active ? 'ok' : '');
  } else {
    rconSetChip(rconDrive, 'drive ?', 'warn');
  }

  const detailBits = [];
  if(robot && robot.detail) detailBits.push(robot.detail);
  if(robot && robot.drive_status) detailBits.push('drive: '+robot.drive_status);
  if(drive && drive.result && (drive.result.ended || drive.result.error))
    detailBits.push('last: '+(drive.result.ended || drive.result.error));
  if(rconDetail) rconDetail.textContent = rconShort(
    detailBits.join(' · ') || 'No robot detail yet.', 240);

  const useful = (events || []).filter(rconInterestingEvent).slice(-5);
  if(useful.length){
    const html = useful.map(e=>
      `<div class="robot-console-line ${rconEventClass(e)}">`
      + htmlEscape(rconEventText(e))+'</div>').join('');
    if(rconLines && rconLines.innerHTML !== html) rconLines.innerHTML = html;
    robotConsoleText = useful.map(rconEventText).join('\n');
  } else if(rconLines && !robotConsoleText){
    rconLines.textContent = 'No notable robot output yet.';
  }
}
async function refreshRobotConsole(){
  if(document.visibilityState !== 'visible') return;
  try{
    const [rr, dr, er] = await Promise.allSettled([
      fetch('/api/robot', {cache:'no-store'}),
      fetch('/api/rl/drive', {cache:'no-store'}),
      fetch('/api/events?n=120', {cache:'no-store'}),
    ]);
    const robot = rr.status === 'fulfilled' && rr.value.ok
      ? await rr.value.json() : null;
    const drive = dr.status === 'fulfilled' && dr.value.ok
      ? await dr.value.json() : null;
    const ev = er.status === 'fulfilled' && er.value.ok
      ? await er.value.json() : {};
    if(robot) paintRobotActivity(robot);
    paintRobotConsole(robot, drive, ev.events || []);
  }catch(e){
    rconSetChip(rconState, 'console offline', 'warn');
  }
}
if(rconCopy){
  rconCopy.onclick = async ()=>{
    const parts = [
      rconState?.textContent || '',
      rconDrive?.textContent || '',
      rconServos?.textContent || '',
      rconDetail?.textContent || '',
      robotConsoleText,
    ].filter(Boolean);
    const t = parts.join('\n');
    try{ await navigator.clipboard.writeText(t); }
    catch(e){
      const ta = document.createElement('textarea');
      ta.value = t; document.body.appendChild(ta);
      ta.select(); document.execCommand('copy'); ta.remove();
    }
    rconCopy.textContent = 'Copied';
    setTimeout(()=>{ rconCopy.textContent = 'Copy'; }, 1200);
  };
}
setInterval(refreshRobotConsole, 2000);
refreshRobotConsole();
document.addEventListener('visibilitychange', ()=>{
  if(document.visibilityState === 'visible') refreshRobotConsole();
});
function fmtDeg(v, d=1){
  return (v==null || Number.isNaN(Number(v))) ? '—' : Number(v).toFixed(d);
}
function feedbackPitchLabel(fb){
  if(!fb || !fb.ok) return '';
  const body = fb.body_pitch_deg;
  const raw = fb.pitch_deg;
  if(body != null) return 'body pitch '+fmtDeg(body)+'°';
  if(raw != null) return 'raw pitch '+fmtDeg(raw)+'°';
  return '';
}
function paintFeedback(fb){
  if(!fb || !fb.ok) return;
  lastFeedback = fb;
  const line = feedbackPitchLabel(fb);
  const gp = $('gp');
  if(gp && line) gp.textContent = line;
  const q = $('qpitch');
  if(q) q.textContent = line || '—';
}

// --- on-screen joysticks ----------------------------------------------------
function makeStick(canvas, horizontalOnly){
  const ctx = canvas.getContext('2d');
  let active=false, nx=0, ny=0;          // normalized -1..1, ny up = +
  function resize(){ const r=canvas.getBoundingClientRect();
    canvas.width=r.width*devicePixelRatio; canvas.height=r.height*devicePixelRatio;
    ctx.setTransform(devicePixelRatio,0,0,devicePixelRatio,0,0); draw(); }
  function draw(){
    const w=canvas.width/devicePixelRatio, h=canvas.height/devicePixelRatio;
    if(w <= 0 || h <= 0) return;
    ctx.clearRect(0,0,w,h);
    const cx=w/2, cy=h/2, R=Math.max(1, Math.min(w,h)/2-12);
    ctx.strokeStyle='#2a3142'; ctx.lineWidth=2;
    ctx.beginPath();
    if(horizontalOnly){ ctx.moveTo(cx-R,cy); ctx.lineTo(cx+R,cy); }
    else { ctx.arc(cx,cy,R,0,7); }
    ctx.stroke();
    ctx.fillStyle='#2b6cff';
    const kx=cx+nx*R, ky=cy-(horizontalOnly?0:ny)*R;
    ctx.beginPath(); ctx.arc(kx,ky,16,0,7); ctx.fill();
  }
  function set(e){
    const r=canvas.getBoundingClientRect();
    const cx=r.width/2, cy=r.height/2;
    const R=Math.max(1, Math.min(r.width,r.height)/2-12);
    let dx=(e.clientX-r.left-cx)/R, dy=-(e.clientY-r.top-cy)/R;
    const m=Math.hypot(dx,dy); if(m>1){ dx/=m; dy/=m; }
    nx=Math.max(-1,Math.min(1,dx)); ny=horizontalOnly?0:Math.max(-1,Math.min(1,dy));
    lastInput=performance.now(); armed=true; dancePaused=false; draw();
  }
  canvas.addEventListener('pointerdown',e=>{active=true; canvas.setPointerCapture(e.pointerId); set(e);});
  canvas.addEventListener('pointermove',e=>{ if(active) set(e); });
  function release(){ active=false; nx=0; ny=0; draw(); }
  canvas.addEventListener('pointerup',release);
  canvas.addEventListener('pointercancel',release);
  window.addEventListener('resize',resize); resize();
  return { get x(){return nx;}, get y(){return ny;} };
}
const driveStick = makeStick(document.getElementById('drive'), false);
const turnStick  = makeStick(document.getElementById('turn'),  true);

// --- keyboard ---------------------------------------------------------------
const keys = new Set();
document.addEventListener('keydown', e=>{
  const k=e.key.toLowerCase();
  if(['w','a','s','d','q','e','arrowup','arrowdown','arrowleft','arrowright',' '].includes(k)) e.preventDefault();
  if(k===' '){ disc('P'); return; }
  if(k==='c'){ disc('C'); return; }
  if(['w','a','s','d','q','e','arrowup','arrowdown','arrowleft','arrowright'].includes(k)){
    keys.add(k); lastInput=performance.now(); armed=true; dancePaused=false; }
});
document.addEventListener('keyup', e=>keys.delete(e.key.toLowerCase()));
function keyVec(){
  let y=0,x=0,t=0;
  if(keys.has('w')||keys.has('arrowup'))y+=1; if(keys.has('s')||keys.has('arrowdown'))y-=1;
  if(keys.has('d'))x+=1; if(keys.has('a'))x-=1;
  if(keys.has('e')||keys.has('arrowright'))t+=1; if(keys.has('q')||keys.has('arrowleft'))t-=1;
  return {x,y,t};
}

// --- gamepad ----------------------------------------------------------------
// Standard Xbox mapping (Chrome Gamepad API):
//   0=A 1=B 2=X 3=Y  4=LB 5=RB  6=LT 7=RT
// Hold LB/LT/RB/RT, then tap X/Y/A/B → 16 demos (rows get harder).
// Face alone: X=sit, Y=stand, A=set-zero-here, B=stop demo.
let gpPrev = [];
const PAD_FACE = {x:2, y:3, a:0, b:1};          // button indices
const PAD_MODS = [                               // easiest → hardest
  {i:4, key:'LB'}, {i:6, key:'LT'},
  {i:5, key:'RB'}, {i:7, key:'RT'},
];
// 4 modifiers × face order X,Y,A,B
const PAD_DEMOS = [
  ['breathe', 'heartbeat', 'twinkle', 'shimmy'],           // LB
  ['ripple', 'conductor', 'arms_up', 'walk'],              // LT
  ['walk_spin', 'plant_bounce', 'plant_gallop', 'plant_tripod'], // RB
  ['walk_oval', 'plant_star', 'plant_stomp', 'rise_show'], // RT
];
const PAD_FACE_ORDER = ['x', 'y', 'a', 'b'];
function httpsUrl(){
  const p = window.HEXAPOD_HTTPS_PORT || 8443;   // substituted into index.html
  return 'https://'+location.hostname+(p===443?'':(':'+p))+location.pathname;
}
function demoMotionLog(){
  const el = $('dmotionlog');
  return !!(el && el.checked);
}
function quadMotionLog(){
  const el = $('qmotionlog');
  return !!(el && el.checked);
}
function padBtnDown(gp, i){
  const btn = gp.buttons[i];
  if(!btn) return false;
  // Triggers report .value; .pressed is fine once past the browser threshold.
  return !!(btn.pressed || (typeof btn.value === 'number' && btn.value > 0.35));
}
async function padRunDemo(name, label){
  if(needArm()) return;
  dancePaused = true;
  showSent('pad '+label+' → '+name+' sent…');
  try{
    const body = {name:name, speed:demoSpeed(), torque:demoTorque()};
    if(demoMotionLog()) body.motion_log = true;
    if(name==='breathe' || name==='breathe_v'){
      body.size = demoSize(); body.rate = demoRate(); body.softness = demoSoft();
    }
    const res = await fetch('/api/demo',{method:'POST',
      headers:{'Content-Type':'application/json'},
      body: JSON.stringify(body)});
    const j = await res.json();
    const home = responseHomeLabel(j);
    if(j.ok) showSent(requestReceiptLine(j, 'demo '+name)
      +(home?(' via '+home):''));
    else showSent(j.error||('demo '+name+' failed'), true);
    if(j.demo) paintDemoStatus(j.demo);
    if(j.robot) paintRobotActivity(j.robot);
    startDemoPoll();
    refreshRobotState(true);
  }catch(e){ showSent('demo '+name+' failed'); }
}
async function goPoseZero(pose, label){
  // Bench routes arm the bus themselves and glide (Drive `P` was a silent
  // one-shot that looked like a no-op when disarmed / already near plant).
  // Outside the Experiments page, normal upright lower/sit uses STEP-down.
  // If the robot is not standing or is tangled, the backend falls back to
  // safe-zero recovery instead of blindly reverse-playing STEP.
  dancePaused = true;
  if(pose === 'stand') armed = false;
  const tag = label || pose;
  showSent(tag + ' request sent…');
  try{
    const stepSit = pose !== 'stand';
    let r = await fetch(stepSit ? '/api/standup' : '/api/zero',{method:'POST',
      headers:{'Content-Type':'application/json'},
      body: JSON.stringify(stepSit
        ? {mode:'step', speed:10, direction:'down'}
        : {pose: pose})});
    const j = await r.json();
    if(!j.ok){
      showSent(requestReceiptLine(j, tag)+'; failed: '
        +(j.error||'unknown'), true);
      if(j.demo) paintDemoStatus(j.demo);
      if(j.robot) paintRobotActivity(j.robot);
      return;
    }
    setArmed(true);
    showSent(requestReceiptLine(j, tag)+'; '
      +(stepSit ? 'STEP lower' : 'gliding'));
    if(j.demo) paintDemoStatus(j.demo);
    if(j.robot) paintRobotActivity(j.robot);
    startDemoPoll();
    for(let i=0;i<90;i++){
      await new Promise(r=>setTimeout(r,400));
      await refreshRobotState(false);
      const d = lastDemo || {};
      if(d.running) continue;
      const st = String(d.status||'');
      const chk = (d.params||{}).stand_check;
      if(st.startsWith('error') || st.startsWith('LIMP')){
        showSent(st.replace(/^error:\s*/,'')); return;
      }
      if(st.startsWith('done')){
        if(pose==='stand' && chk && chk.max_err_deg!=null)
          showSent('stand verified · tracking '+Number(chk.max_err_deg).toFixed(1)+'°');
        else
          showSent(tag + ' done');
        return;
      }
      if(st.startsWith('aborted')){ showSent(tag + ' aborted'); return; }
      if(st){ showSent(st); return; }
      break;
    }
  }catch(e){ showSent(tag + ' failed'); }
  refreshRobotState(false);
}
async function padSit(){ return goPoseZero('sit', 'sit zero'); }
async function padStand(){ return goPoseZero('stand', 'stand zero'); }
async function padSetZero(){
  showSent('pad A → set-here-as-zero…');
  try{
    const r = await fetch('/api/set_zero',{method:'POST'});
    const j = await r.json();
    setArmed(false);
    if(j.ok) showSent('zero-here OK — '+(j.ok_n||'?')+'/'+(j.count||'?')+' (limp) · plant reset');
    else showSent('zero-here '+(j.error||'failed'));
  }catch(e){ showSent('zero-here failed'); }
}
async function padStopDemo(){
  showSent('pad B → stop demo sent…');
  try{
    const r = await fetch('/api/demo/stop',{method:'POST'});
    const j = await r.json();
    showSent(requestReceiptLine(j, 'Stop demo'));
    if(j.demo) paintDemoStatus(j.demo);
    if(j.robot) paintRobotActivity(j.robot);
  }catch(e){}
  refreshRobotState(true);
}
function pollGamepad(){
  const pads = navigator.getGamepads ? navigator.getGamepads() : [];
  let gp=null; for(const p of pads){ if(p){ gp=p; break; } }
  if(!gp){
    // Browsers only expose gamepads in a secure context (HTTPS/localhost).
    if(!window.isSecureContext)
      gpEl.innerHTML = '🎮 needs HTTPS &rarr; <a href="'+httpsUrl()+
        '" style="color:#7cf">open secure page</a> (accept the warning)';
    else
      gpEl.textContent = '🎮 press a button on the controller to connect';
    return null;
  }
  const modHeld = PAD_MODS.map(m => padBtnDown(gp, m.i));
  const modIdx = modHeld.findIndex(Boolean);
  const modLabel = modIdx >= 0 ? PAD_MODS[modIdx].key : '';
  gpEl.textContent = modLabel
    ? ('pad '+modLabel+' + X/Y/A/B')
    : ('pad '+gp.id.slice(0,18));

  const down = [];
  for(let i=0;i<gp.buttons.length;i++) down[i] = padBtnDown(gp, i);
  const press = i => down[i] && !gpPrev[i];

  // Face edge while a shoulder/trigger is held → demo chord.
  if(modIdx >= 0){
    for(let fi=0; fi<PAD_FACE_ORDER.length; fi++){
      const bi = PAD_FACE[PAD_FACE_ORDER[fi]];
      if(press(bi)){
        const name = PAD_DEMOS[modIdx][fi];
        padRunDemo(name, PAD_MODS[modIdx].key+'+'+PAD_FACE_ORDER[fi].toUpperCase());
      }
    }
  } else {
    if(press(PAD_FACE.x)) padSit();
    if(press(PAD_FACE.y)) padStand();
    if(press(PAD_FACE.a)) padSetZero();
    if(press(PAD_FACE.b)) padStopDemo();
  }
  gpPrev = down;

  // Left stick = walk (axes 0/1); right stick X = turn (axis 2).
  const dz=v=>Math.abs(v)<0.12?0:v;
  const ax0=dz(gp.axes[0]||0), ax1=dz(gp.axes[1]||0);
  const ax2=dz(gp.axes[2]||0);   // right stick X (not triggers)
  if(ax0||ax1||ax2){ lastInput=performance.now(); armed=true; dancePaused=false; }
  return {x:ax0, y:-ax1, t:ax2};
}

function disc(c){
  if(c==='X'){ settleServos(); return; }
  // Stand / Center: use verified glide, not the old one-shot /cmd P|C.
  if(c==='P'){ goPoseZero('stand', '▲ Stand'); forceResend(); return; }
  if(c==='C'){ goPoseZero('sit', 'Center / Sit'); forceResend(); return; }
  if(needArm()) return;
  cmd(c); forceResend();
}

document.querySelectorAll('button[data-cmd]').forEach(btn=>
  btn.onclick=()=> disc(btn.dataset.cmd));
document.getElementById('stop').onclick=settleServos;

const vmax=document.getElementById('vmax'), lift=document.getElementById('lift');
vmax.oninput=()=>{ maxVx=+vmax.value; maxVy=Math.round(maxVx*0.73);
  document.getElementById('vlab').textContent=vmax.value; };
lift.oninput=()=>{ document.getElementById('klab').textContent=lift.value; };
lift.onchange=()=>cmd('K '+lift.value);
maxVx = +vmax.value; maxVy = Math.round(maxVx*0.73);

// --- Bench zero workflow -------------------------------------------------------
// Mirrors rl_move/scripts/tape_measure_walk.py: the operator limps (Motors →
// Limp all, or E-STOP), hand-poses, POST /api/set_zero (top bar), ARMs,
// Stands (P glide), preflights, then the gait gets plain `J vx vy omega`
// and `J 0 0 0` over /cmd — nothing else.
async function setZeroHere(fromMotors){
  // No confirm (operator 08-11: no warning modals). Motors do not
  // move — only the zero point is rewritten.
  showSent('set-here-as-zero…');
  try{
    const r = await fetch('/api/set_zero',{method:'POST'});
    const j = await r.json();
    setArmed(false);
    if(j.ok) showSent('zero-here OK — '+j.ok_n+'/'+j.count+' (limp) · plant reset');
    else showSent('zero-here '+(j.error || ((j.ok_n||0)+'/'+(j.count||0)+' — check Motors table')));
  }catch(e){ showSent('zero-here failed'); }
  if(fromMotors) refreshMotors();
}
document.getElementById('topsetzero').onclick = ()=> setZeroHere(false);
document.getElementById('wpreflight').onclick = async ()=>{
  const out = document.getElementById('wpfout');
  out.textContent = 'Preflight (read-only)…';
  try{
    const r = await fetch('/api/rl/preflight?mode=lower&t='+Date.now(), {cache:'no-store'});
    const d = await r.json();
    const det = [];
    if(d.roll_deg!=null) det.push('roll '+d.roll_deg+'°');
    if(d.pitch_deg!=null) det.push('pitch '+d.pitch_deg+'°');
    if(d.max_pose_delta_deg!=null)
      det.push('pose Δ '+d.max_pose_delta_deg+'° (tol '+d.pose_tol_deg+'°)');
    out.innerHTML = d.ok
      ? '<b style="color:#5fd08a">READY</b> — servos answering, pose near plant · '+det.join(' · ')
      : '<b style="color:#ff7b72">NOT ready</b>: '+(d.error||'?')
        + (det.length ? ' · '+det.join(' · ') : '')
        + ' — fresh Set zero → Stand before walking.';
  }catch(e){ out.textContent = 'preflight failed (link?)'; }
};

// --- gait picker: 0 tripod drag · 1 no-slip tripod (+alpha) · 2 no-slip
// ripple · 3 no-slip wave · 4 SE2 tetrapod · 5 SE2 wave. Alpha only tunes
// gait 1 (the others run their presets), so the slider hides for them. --------
// `gait` (top of file) also rides the manual-drive J stream, so the picker
// applies to both the timed walk pad and the sticks. The controller refuses
// swaps while walking (stop first) and applies alpha at the next phase
// boundary of a live no-slip gait.
const wgaitSel = document.getElementById('wgait');
const walphaEl = document.getElementById('walpha');
function sendGait(){
  gait = parseInt(wgaitSel.value, 10) || 0;
  const a = parseFloat(walphaEl.value) || 0;
  document.getElementById('walab').textContent = a.toFixed(2);
  document.getElementById('walphawrap').style.display =
    gait === 1 ? '' : 'none';
  const line = 'GAIT ' + gait + ' ' + a.toFixed(2);
  cmd(line); showSent(line); forceResend();
}
wgaitSel.onchange = sendGait;

// --- cpg controller loader (cpg track artifacts, 08-23) ---------------------
// CPGLIST/CPGLOAD are plain DriveController.handle() lines (same /cmd
// channel as GAIT/J), so no new HTTP route was needed -- this is just a
// convenience picker over the two commands. Loading never swaps the live
// gait; the operator still picks "SE2 CPG" in the gait select above and
// sends/starts the walk to actually use it.
const wcpgSel = document.getElementById('wcpgsel');
const wcpgStatus = document.getElementById('wcpgstatus');
async function refreshCpgList(){
  wcpgSel.innerHTML = '<option value="">(loading…)</option>';
  try{
    const r = await fetch('/cmd', {method:'POST', body:'CPGLIST'});
    const text = await r.text();
    const rows = JSON.parse(text);
    wcpgSel.innerHTML = '';
    if(!rows.length){
      wcpgSel.innerHTML = '<option value="">(none found)</option>';
      return;
    }
    for(const row of rows){
      const opt = document.createElement('option');
      opt.value = row.name || row.file;
      const gate = row.gate_pass_dr0 === true ? 'PASS'
        : row.gate_pass_dr0 === false ? 'fail' : '?';
      const slip = row.gate_slip_per_m != null
        ? row.gate_slip_per_m.toFixed(2) : '?';
      opt.textContent = (row.error
        ? row.file + ' — ' + row.error
        : (row.name || row.file) + ' (' + (row.gait||'?')
          + ', dr0 gate ' + gate + ', slip/m ' + slip + ')');
      wcpgSel.appendChild(opt);
    }
  }catch(e){
    wcpgSel.innerHTML = '<option value="">(list failed — link?)</option>';
  }
}
document.getElementById('wcpgrefresh').onclick = refreshCpgList;
document.getElementById('wcpgload').onclick = async ()=>{
  const name = wcpgSel.value;
  if(!name){ wcpgStatus.textContent = 'pick a controller first.'; return; }
  const line = 'CPGLOAD ' + name;
  try{
    const r = await fetch('/cmd', {method:'POST', body:line});
    const msg = await r.text();
    wcpgStatus.textContent = msg;
    showSent(line, !r.ok);
  }catch(e){
    wcpgStatus.textContent = 'load failed — link?';
  }
};
refreshCpgList();

walphaEl.oninput = ()=>{
  document.getElementById('walab').textContent =
    (parseFloat(walphaEl.value) || 0).toFixed(2);
};
walphaEl.onchange = sendGait;

// --- scripted gait walk (exact tape_measure_walk.py commands) ---------------
// Start = `J vx vy omega [gait]` (script run_leg), timed stop / STOP button =
// `J 0 0 0` → planted stand, torque stays on (limp is the operator's call).
let walkTimer = null, walkTick = null, walkEndT = 0, walkLine = '';
function stopGaitWalk(msg){
  if(walkTimer){ clearTimeout(walkTimer); walkTimer = null; }
  if(walkTick){ clearInterval(walkTick); walkTick = null; }
  cmd('J 0 0 0'); showSent('J 0 0 0'); forceResend();
  document.getElementById('wstatus').textContent =
    msg || 'stopped — planted stand, torque on (E-STOP / Limp to go limp)';
}
document.getElementById('wstop').onclick = ()=> stopGaitWalk();
document.getElementById('wstart').onclick = async ()=>{
  if(needArm()) return;
  // Same caps as the tape script (teleop known-good envelope).
  const vx = clamp(parseFloat($('wvx').value)||0, -60, 60);
  const vy = clamp(parseFloat($('wvy').value)||0, -40, 40);
  const om = clamp(parseFloat($('wom').value)||0, -0.5, 0.5);
  const dur = clamp(parseFloat($('wdur').value)||20, 3, 60);
  $('wvx').value = vx; $('wvy').value = vy; $('wom').value = om; $('wdur').value = dur;
  if(!vx && !vy && !om){ showSent('walk: zero command — set vx/vy/ω first'); return; }
  armed = false; dancePaused = false;   // keep the stick loop from streaming J over this
  if(walkTimer) clearTimeout(walkTimer);
  if(walkTick) clearInterval(walkTick);
  walkLine = 'J '+vx.toFixed(1)+' '+vy.toFixed(1)+' '+om.toFixed(3)+' '+gait;
  await cmd(walkLine); showSent(walkLine); forceResend();
  walkEndT = performance.now() + dur*1000;
  walkTick = setInterval(()=>{
    const left = Math.max(0, (walkEndT - performance.now())/1000);
    document.getElementById('wstatus').textContent =
      'walking ('+walkLine+') — '+left.toFixed(0)+' s left';
  }, 250);
  walkTimer = setTimeout(()=> stopGaitWalk(
    'done — holding planted stand (torque on). Read the tape.'), dur*1000);
};

// --- Drive telemetry strip (2 Hz /api/feedback, Drive tab only) --------------
let telemTimer = null, telemBusy = false;
function startTelem(){
  stopTelem();
  refreshTelem();
  telemTimer = setInterval(()=>{
    if(activeView==='drive' && document.visibilityState==='visible') refreshTelem();
  }, 500);
}
function stopTelem(){ if(telemTimer){ clearInterval(telemTimer); telemTimer=null; } }
async function refreshTelem(){
  if(telemBusy) return;
  telemBusy = true;
  try{
    const r = await fetch('/api/feedback?t='+Date.now(), {cache:'no-store'});
    const fb = await r.json();
    if(fb && fb.ok){
      paintFeedback(fb);
      $('tmroll').textContent = fmtDeg(
        fb.body_roll_deg != null ? fb.body_roll_deg : fb.roll_deg, 1);
      $('tmpitch').textContent = fmtDeg(
        fb.body_pitch_deg != null ? fb.body_pitch_deg : fb.pitch_deg, 1);
      let total = null;
      for(const m of (fb.joints||[])){
        if(m && m.cur_a!=null) total = (total||0) + Math.abs(+m.cur_a || 0);
      }
      $('tmcur').textContent = total==null ? '—' : total.toFixed(2);
      $('tmlive').textContent = fb.live!=null ? fb.live : '—';
      $('tmstamp').textContent = 'updated '+new Date().toLocaleTimeString()+' · 2 Hz';
    } else {
      $('tmstamp').textContent = (fb && fb.error) ? ('feedback: '+fb.error) : 'feedback unavailable';
    }
  }catch(e){
    $('tmstamp').textContent = 'feedback failed (link?)';
  }finally{
    telemBusy = false;
  }
}

// --- drive loop -------------------------------------------------------------
// Runs at 20 Hz for responsive input, but only SENDS a J packet when the
// command actually changes (plus a slow heartbeat) -- streaming identical
// packets at 20 Hz floods and wedges the MCU<->Linux serial bridge.
function clamp(v,a,b){ return Math.max(a,Math.min(b,v)); }
let lastLine='', lastSendT=0;
function forceResend(){ lastLine=''; }   // call after any discrete command
function loop(){
  if(activeView !== 'drive') return;   // only stream J on Drive tab
  const gpv = pollGamepad();
  let x=0,y=0,t=0;
  if(gpv && (gpv.x||gpv.y||gpv.t)){ x=gpv.x; y=gpv.y; t=gpv.t; }
  else {
    const kv=keyVec();
    x = driveStick.x || kv.x;
    y = driveStick.y || kv.y;
    t = turnStick.x  || kv.t;
  }
  if(servosArmed && armed && !dancePaused){
    const vx = clamp(y,-1,1)*maxVx;          // up = forward
    const vy = clamp(-x,-1,1)*maxVy;         // right = strafe right (chassis -Y)
    const om = clamp(-t,-1,1)*maxOmega;      // right = turn right
    const line = `J ${vx.toFixed(0)} ${vy.toFixed(0)} ${om.toFixed(3)} ${gait}`;
    const now = performance.now();
    if(line !== lastLine || now - lastSendT > 300){   // change or ~3 Hz heartbeat
      cmd(line); showSent(line); lastLine=line; lastSendT=now;
    }
  }
}
setInterval(loop, 50);
window.addEventListener('gamepadconnected', ()=>{ armed=true; });

// --- Debug page: per-servo control + movement test -------------------------
// Firmware protocol used here (prototype_servo_test.ino):
//   "# <j> <deg>"  set ONE joint j (0..17) to an absolute angle + hold
//   "Q <j> <deg>"  quick wiggle joint j by +/-deg around its current angle
//   C center all,  P stand/park,  X relax (limp)
// Per-axis safe limits MUST match the firmware (axisLimits): yaw/hip/knee.
const AXIS_LIM  = [[-35,35],[-80,30],[-20,80]];   // yaw, hip, knee (deg)
const AXIS_NAME = ['yaw','hip','knee'];
let dbgLeg = 0, dbgAxis = 0;
let dbgTestRunning = false, dbgTestAbort = false;
const dbgIndex  = ()=> dbgLeg*3 + dbgAxis;
const dbgLimits = ()=> AXIS_LIM[dbgAxis];
const $ = id => document.getElementById(id);

const robotUrlInput = $('roboturl');
if(robotUrlInput && savedRobotUrl()) robotUrlInput.value = savedRobotUrl();

function paintTargetBadge(id, text, cls){
  const el = $(id);
  if(!el) return;
  el.textContent = text;
  el.className = 'target-badge' + (cls ? ' '+cls : '');
}
function paintTargetMsg(which){
  const el = $(which === 'robot' ? 'robotlinemsg' : 'simlinemsg');
  if(!el) return;
  const msg = targetLineMsg[which];
  if(!msg || !msg.text){
    el.textContent = '';
    el.title = '';
    el.className = 'target-msg';
    return;
  }
  el.textContent = msg.text;
  el.title = msg.text;
  el.className = 'target-msg show ' + (msg.level || 'warn');
}
function setTargetLineMsg(which, text, level){
  if(which !== 'robot' && which !== 'sim') return;
  targetLineMsg[which] = text ? {text:String(text), level:level || 'warn'} : null;
  paintTargetMsg(which);
}
function clearTargetLineMsg(which){
  setTargetLineMsg(which, '', '');
}
function classifyTargetError(text, fallback){
  const low = String(text || '').toLowerCase();
  if(/robot|hexapod|servo|bus|proxy/.test(low)) return 'robot';
  if(/sim|mujoco|mj|policy/.test(low)) return 'sim';
  return fallback || '';
}
function paintTargetRows(){
  paintTargetBadge('robotlinesend', targetHasRobot ? 'active' : 'idle',
    targetHasRobot ? 'route' : '');
  paintTargetBadge('robotlineconn',
    robotTargetTransient ? 'restarting…'
      : (robotTargetAvailable ? 'connected' : 'not connected'),
    robotTargetTransient ? 'warn'
      : (robotTargetAvailable ? 'ok' : 'bad'));
  paintTargetBadge('simlinesend', targetHasSim ? 'active' : 'idle',
    targetHasSim ? 'route' : '');
  paintTargetBadge('simlineconn',
    simTargetAvailable ? 'connected' : 'not connected',
    simTargetAvailable ? 'ok' : 'bad');
  const rb = $('robotconnect');
  if(rb){
    rb.textContent = !robotTargetAvailable ? 'Connect'
      : (targetHasRobot ? 'Disconnect' : 'Connect');
    rb.classList.toggle('on', targetHasRobot);
    rb.title = targetHasRobot
      ? 'Disconnect Robot from this web UI; MuJoCo stays active if connected'
      : 'Connect this laptop web UI to the real robot web server';
  }
  const sb = $('simconnect');
  if(sb){
    sb.textContent = targetHasSim ? 'Disconnect' : 'Connect';
    sb.classList.toggle('on', targetHasSim);
    sb.title = targetHasSim
      ? 'Disconnect MuJoCo from this web UI; Robot stays active if connected'
      : 'Connect this web UI to MuJoCo';
  }
  paintTargetMsg('robot');
  paintTargetMsg('sim');
}
paintTargetRows();

function robotUrlValue(){
  const el = $('roboturl');
  return el ? el.value.trim() : '';
}

async function connectRobotTarget(nextTarget){
  const url = robotUrlValue();
  if(!url){
    showSent('enter robot URL first', true);
    const el = $('roboturl');
    if(el) el.focus();
    return false;
  }
  saveRobotUrl(url);
  showSent('connecting robot…');
  setTargetLineMsg('robot', 'connecting…', 'warn');
  try{
    const r = await fetch('/api/hub', {method:'POST',
      headers:{'Content-Type':'application/json'},
      body: JSON.stringify({robot_url:url, target:nextTarget || 'robot'})});
    const d = await r.json().catch(()=>({ok:false, error:'bad response'}));
    if(!r.ok) throw new Error(d.error || 'connect failed');
    applyBackendMeta(d);
    setArmed(false);
    const resolved = d.targets && d.targets.robot && d.targets.robot.url;
    simPollMaybe();
    if(d.ok){
      clearTargetLineMsg('robot');
      setLink(true, 'online');
      showSent('robot target connected → '+(resolved || url));
      return true;
    } else {
      setTargetLineMsg('robot', d.error || 'unreachable', 'bad');
      setLink(false, d.error || 'robot unavailable');
      showSent('robot connect failed: '+(d.error || 'unreachable'), true);
      return false;
    }
  }catch(e){
    setTargetLineMsg('robot', e.message || e, 'bad');
    showSent('robot connect failed: '+(e.message || e), true);
    return false;
  }
}

async function setHubTarget(target){
  if((target === 'robot' || target === 'both') && !robotTargetAvailable)
    return connectRobotTarget(target);
  try{
    const body = {target};
    const typedUrl = robotUrlValue();
    if((target === 'robot' || target === 'both')
        && typedUrl && typedUrl !== robotTargetUrl){
      body.robot_url = typedUrl;
      saveRobotUrl(typedUrl);
    }
    const r = await fetch('/api/hub', {method:'POST',
      headers:{'Content-Type':'application/json'},
      body: JSON.stringify(body)});
    const d = await r.json();
    if(!d.ok) throw new Error(d.error || 'target switch failed');
    applyBackendMeta(d);
    if(target === 'robot' || target === 'both') clearTargetLineMsg('robot');
    if(target === 'sim' || target === 'both') clearTargetLineMsg('sim');
    setArmed(false);
    const label = target === 'both' ? 'Robot + MuJoCo'
      : (target === 'sim' ? 'MuJoCo' : 'Robot');
    showSent('connected → '+label);
    simPollMaybe();
    return true;
  }catch(e){
    const which = classifyTargetError(e.message || e,
      target === 'sim' ? 'sim' : (target === 'robot' ? 'robot' : ''));
    if(which) setTargetLineMsg(which, e.message || e, 'bad');
    showSent('target switch failed: '+(e.message || e), true);
    return false;
  }
}
function targetWith(which){
  if(which === 'robot')
    return targetHasSim ? 'both' : 'robot';
  return targetHasRobot ? 'both' : 'sim';
}
function targetWithout(which){
  if(which === 'robot'){
    if(targetHasSim || simTargetAvailable) return 'sim';
    return '';
  }
  if(targetHasRobot || robotTargetAvailable) return 'robot';
  return '';
}
async function toggleRobotTarget(){
  if(targetHasRobot){
    const next = targetWithout('robot');
    if(!next){
      showSent('connect MuJoCo before disconnecting Robot', true);
      return false;
    }
    return await setHubTarget(next);
  }
  return await setHubTarget(targetWith('robot'));
}
async function toggleSimTarget(){
  if(targetHasSim){
    const next = targetWithout('sim');
    if(!next){
      showSent('connect Robot before disconnecting MuJoCo', true);
      return false;
    }
    return await setHubTarget(next);
  }
  return await setHubTarget(targetWith('sim'));
}
if($('robotconnect')) $('robotconnect').onclick =
  ()=> toggleRobotTarget();
if($('simconnect')) $('simconnect').onclick =
  ()=> toggleSimTarget();
if($('roboturl')) $('roboturl').addEventListener('keydown', e=>{
  if(e.key === 'Enter' && !targetHasRobot) toggleRobotTarget();
});

async function ensureDemoTarget(item){
  if(!hubMode || !item || !item.target) return true;
  if(item.target === 'robot' && hubTarget !== 'robot'){
    showSent('switching target → robot for '+item.name);
    return await setHubTarget('robot');
  }
  if(item.target === 'sim' && hubTarget !== 'sim'){
    showSent('switching target → sim for '+item.name);
    return await setHubTarget('sim');
  }
  return true;
}

function dbgRefresh(){
  const idx = dbgIndex(), lim = dbgLimits();
  $('dbgidx').textContent = idx;
  $('dbglegn').textContent = dbgLeg;
  $('dbgaxisn').textContent = AXIS_NAME[dbgAxis];
  $('dbglo').textContent = lim[0];
  $('dbghi').textContent = lim[1];
  const a = $('dbgangle');
  a.min = lim[0]; a.max = lim[1];
  a.value = clamp(+a.value, lim[0], lim[1]);
  $('dbganglelab').textContent = a.value;
}

// De-duped / throttled single-servo set (same idea as the J drive loop, which
// avoids flooding the MCU<->Linux serial bridge with identical packets).
let dbgLastLine = '', dbgLastT = 0;
function dbgSend(deg, force){
  const idx = dbgIndex(), lim = dbgLimits();
  deg = Math.round(clamp(deg, lim[0], lim[1]));
  $('dbgangle').value = deg; $('dbganglelab').textContent = deg;   // UI updates even when disarmed
  if(needArm()) return;                                            // but never sends while disarmed
  const line = '# '+idx+' '+deg, now = performance.now();
  if(!force){
    if(line === dbgLastLine && now - dbgLastT < 400) return;  // slow heartbeat only
    if(now - dbgLastT < 50) return;                           // ~20 Hz cap while dragging
  }
  dbgLastLine = line; dbgLastT = now;
  cmd(line); showSent(line);
}

document.querySelectorAll('#dbgleg button').forEach(b=>b.onclick=()=>{
  dbgLeg = +b.dataset.leg;
  document.querySelectorAll('#dbgleg button').forEach(x=>x.classList.toggle('on', x===b));
  dbgRefresh();
});
document.querySelectorAll('#dbgaxis button').forEach(b=>b.onclick=()=>{
  dbgAxis = +b.dataset.axis;
  document.querySelectorAll('#dbgaxis button').forEach(x=>x.classList.toggle('on', x===b));
  dbgRefresh();
});
const dbgAngle = $('dbgangle');
dbgAngle.oninput  = ()=>{ $('dbganglelab').textContent = dbgAngle.value; dbgSend(+dbgAngle.value); };
dbgAngle.onchange = ()=> dbgSend(+dbgAngle.value, true);
$('dbgctr').onclick = ()=> dbgSend(0, true);
$('dbgneg').onclick = ()=> dbgSend(dbgLimits()[0], true);
$('dbgpos').onclick = ()=> dbgSend(dbgLimits()[1], true);
$('dbgdn').onclick  = ()=> dbgSend(+dbgAngle.value - 5, true);
$('dbgup').onclick  = ()=> dbgSend(+dbgAngle.value + 5, true);

const dbgAmp = $('dbgamp');
dbgAmp.oninput = ()=>{ $('dbgamplab').textContent = dbgAmp.value; };
const dbgStatus = t => { $('dbgteststatus').textContent = t; };

$('dbgtestone').onclick = ()=>{
  if(needArm()) return;
  const idx = dbgIndex(), amp = dbgAmp.value;
  cmd('Q '+idx+' '+amp); showSent('Q '+idx+' '+amp);
  dbgStatus('Wiggling servo '+idx+' (leg '+dbgLeg+' '+AXIS_NAME[dbgAxis]+') ±'+amp+'° around its current angle.');
};

const sleep = ms => new Promise(r=>setTimeout(r, ms));
async function dbgTestAll(){
  if(needArm()) return;
  if(dbgTestRunning) return;
  dbgTestRunning = true; dbgTestAbort = false;
  cmd('C'); showSent('C'); dbgStatus('Centering all servos…');
  await sleep(1500);
  const amp = +dbgAmp.value;
  for(let j=0; j<18 && !dbgTestAbort; j++){
    cmd('Q '+j+' '+amp); showSent('Q '+j+' '+amp);
    dbgStatus('Testing servo '+j+'/17  (leg '+Math.floor(j/3)+' '+AXIS_NAME[j%3]+')  ±'+amp+'°…');
    await sleep(2600);   // let each wiggle finish before starting the next
  }
  dbgStatus(dbgTestAbort ? 'Test stopped.' : 'Test complete — all 18 servos wiggled.');
  dbgTestRunning = false;
}
$('dbgtestall').onclick = dbgTestAll;
$('dbgteststop').onclick = ()=>{ dbgTestAbort = true; cmd('C'); showSent('C'); dbgStatus('Stopping…'); };

// --- tab switching ----------------------------------------------------------
const VIEWS = ['drive','motors','demos','dance','rock','quad','rl',
               'experiments','measure','calibrate','debug'];
const TAB_TITLES = {drive:'Drive', motors:'Motors', demos:'Demos',
                    dance:'Dance', rock:'Rock', quad:'Quad', rl:'RL',
                    measure:'Measure', calibrate:'Calibrate', debug:'Debug'};
function showView(which){
  activeView = which;
  VIEWS.forEach(v=>{
    const el = $('view-'+v); if(el) el.classList.toggle('active', v===which);
    const tab = $('tab-'+v); if(tab) tab.classList.toggle('on', v===which);
  });
  document.title = 'Hexapod · '+(TAB_TITLES[which] || which);
  if(location.hash !== '#'+which)
    history.replaceState(null, '', '#'+which);
  if(which !== 'drive') armed = false;   // stop streaming J
  if(which !== 'rl' && drvKeys.size){
    drvKeys.clear(); drvSend();          // leaving RL = keys released
  }
  if(which === 'drive') startTelem();
  else stopTelem();
  if(which === 'motors'){
    // Freeze stand/walk re-hold so the Motors tab can wiggle without the
    // background loop yanking the body toward the plant/crouch pose.
    if(servosArmed){ cmd('HOLD'); forceResend(); }
    refreshMotors(); startMotorsPoll();
  }
  else stopMotorsPoll();
  if(which === 'demos'){ loadDemos(); refreshDemoStatus(); startDemoPoll(); }
  else if(which === 'quad'){ refreshRobotState(true); startDemoPoll(); }
  else if(which === 'dance'){
    loadDance(); refreshDemoStatus(); startDemoPoll();
    $('dancespeed').value = $('dspeed').value;   // stay in sync with Demos
    $('dancespeedlab').textContent = demoSpeed().toFixed(2);
  }
  else if(which === 'rock'){ refreshDemoStatus(); startDemoPoll(); }
  else stopDemoPoll();
  if(which === 'calibrate'){
    if(servosArmed){ cmd('HOLD'); forceResend(); }
    refreshManualGeometry();
    refreshCalibrate(); startCalPoll();
  }
  else stopCalPoll();
  if(which === 'rl'){ refreshRlTab(); simPollMaybe(); }
  else stopSimPoll();
  if(which === 'measure'){ muRefresh(); muPollMaybe(); }
}
$('tab-drive').onclick = ()=> showView('drive');
$('tab-motors').onclick = ()=> showView('motors');
$('tab-demos').onclick = ()=> showView('demos');
$('tab-dance').onclick = ()=> showView('dance');
$('tab-rock').onclick = ()=> showView('rock');
$('tab-quad').onclick = ()=> showView('quad');
$('tab-rl').onclick = ()=> showView('rl');
$('tab-experiments').onclick = ()=> showView('experiments');
$('tab-measure').onclick = ()=> showView('measure');
$('tab-calibrate').onclick = ()=> showView('calibrate');
$('tab-debug').onclick = ()=> showView('debug');
dbgRefresh();

// --- Calibrate checkup -------------------------------------------------------
function startCalPoll(){
  stopCalPoll();
  calTimer = setInterval(()=>{ if(activeView==='calibrate') refreshCalibrate(); }, 800);
}
function stopCalPoll(){ if(calTimer){ clearInterval(calTimer); calTimer=null; } }
function paintPlantInfo(plant){
  if(!plant || !plant.ok) return;
  const hip = Number(plant.hip_deg), knee = Number(plant.knee_deg);
  $('calplanthip').textContent = (hip>=0?'+':'')+hip.toFixed(1);
  $('calplantknee').textContent = (knee>=0?'+':'')+knee.toFixed(1);
  $('calplantsrc').textContent = plant.learned
    ? ('(learned'+(plant.timestamp?(' · '+plant.timestamp):'')+')')
    : '(default +19 / +28)';
}
function paintImuInfo(imu){
  if(!imu || !imu.ok) return;
  if(imu.learned){
    const g = imu.grade || 'ok';
    const gb = imu.gyro_bias_dps || {};
    $('calimustatus').textContent = 'calibrated · '+g;
    $('calimudetail').textContent =
      ' · ω bias ('+
      Number(gb.x||0).toFixed(2)+', '+
      Number(gb.y||0).toFixed(2)+', '+
      Number(gb.z||0).toFixed(2)+') dps'+
      (imu.timestamp?(' · '+imu.timestamp):'');
  } else {
    $('calimustatus').textContent = 'not calibrated';
    $('calimudetail').textContent = ' · raw MPU until you run IMU rest';
  }
}
const CHECKUP_STEPS = [
  {id:'safe_zero', name:'Safe zero start pose',
   detail:'Move through the collision-aware zero path before calibration.'},
  {id:'imu_rest', name:'IMU rest/bias',
   detail:'Hold still while gyro and accel rest offsets are saved.'},
  {id:'geometry_plant', name:'Ground contact geometry',
   detail:'Reach down until contact is detected, then save the plant pose.'},
  {id:'geometry_sweep', name:'Dimension sweep',
   detail:'Collect several floor contact poses to estimate heights and zero hints.'},
  {id:'geometry_plausibility', name:'Geometry plausibility',
   detail:'Check whether contact/FK geometry should be trusted or treated as diagnostic only.'},
  {id:'imu_body_frame', name:'Quad IMU body frame',
   detail:'Rear up and come down while mapping mounted IMU axes to body pitch.'},
  {id:'imu_frame_validation', name:'IMU frame validation',
   detail:'Validate that the saved IMU body-frame map is strong enough for balance trim.'},
  {id:'stability_margin', name:'Stability margin',
   detail:'Bias the planted stance in four directions and record reversible tilt margin.'},
  {id:'mass_shift_response', name:'Mass shift response',
   detail:'Lift small limb groups and measure how much pitch and roll change.'},
  {id:'traction_probe', name:'Traction / slip',
   detail:'Run repeated gentle planted yaw shears to flag floor slip.'},
  {id:'return_zero', name:'Return to zero',
   detail:'Move back through the collision-aware zero path before torque-off.'},
  {id:'proprioception_check', name:'Proprioception check',
   detail:'Compare expected pose with live encoder, current, voltage, and temperature feedback.'},
  {id:'camera_witness', name:'Camera witness',
   detail:'Optional synced video/photo evidence for visible body and foot motion.'},
  {id:'bus_power_health', name:'Bus / power health',
   detail:'Check live servos, bus voltage, peak current, and servo temperature.'},
  {id:'actuator_snapshot', name:'Actuator snapshot',
   detail:'Read live joint angle, current, load, voltage, and temperature.'},
  {id:'report', name:'Report',
   detail:'Write one sim-ready calibration report on the robot.'},
];
function checkupPhaseFromProgress(p){
  if(p && p.phase) return p.phase;
  const msg = String((p && p.msg) || '').toLowerCase();
  if(msg.includes('return zero') || msg.includes('zero return')) return 'return_zero';
  if(msg.includes('proprio')) return 'proprioception_check';
  if(msg.includes('camera')) return 'camera_witness';
  if(msg.includes('bus') || msg.includes('power')) return 'bus_power_health';
  if(msg.includes('safe_zero') || msg.includes('safe zero') || msg.includes('zero:')) return 'safe_zero';
  if(msg.includes('plausibility')) return 'geometry_plausibility';
  if(msg.includes('frame validation')) return 'imu_frame_validation';
  if(msg.includes('stability')) return 'stability_margin';
  if(msg.includes('mass shift')) return 'mass_shift_response';
  if(msg.includes('sweep') || msg.includes('dimension')) return 'geometry_sweep';
  if(msg.includes('traction') || msg.includes('slip')) return 'traction_probe';
  if(msg.includes('imu body') || msg.includes('quad rear')) return 'imu_body_frame';
  if(msg.includes('geo') || msg.includes('ground') || msg.includes('contact')) return 'geometry_plant';
  if(msg.includes('actuator')) return 'actuator_snapshot';
  if(msg.includes('report') || msg.includes('saving')) return 'report';
  if(msg.includes('imu')) return 'imu_rest';
  return null;
}
function checkupPhaseMap(result){
  const m = {};
  for(const p of ((result && result.phases) || [])){
    if(p && p.name) m[p.name] = p;
  }
  return m;
}
function isCalibrationResult(res){
  if(!res) return false;
  const mode = String(res.mode || '');
  return mode === 'checkup' || mode === 'calibration_report' ||
    Array.isArray(res.phases) || !!res.geometry || !!res.report;
}
function calibrationDisplayResult(state){
  if(!state) return null;
  if(state.running) return state.result || null;
  if(isCalibrationResult(state.result)) return state.result;
  return state.latest_report || state.result || null;
}
function checkupPill(status){
  const cls = status === 'ok' ? 'green'
    : status === 'running' || status === 'issue' || status === 'skipped' ? 'yellow'
    : status === 'abort' ? 'red' : '';
  const label = status === 'ok' ? 'done'
    : status === 'running' ? 'running'
    : status === 'issue' ? 'issue'
    : status === 'skipped' ? 'skipped'
    : status === 'abort' ? 'stopped' : 'pending';
  return `<span class="cal-pill ${cls}">${label}</span>`;
}
function renderCheckupSteps({running=false, progress=null, result=null}={}){
  const el = $('calsteps');
  if(!el) return;
  const phases = checkupPhaseMap(result);
  const runningPhase = running ? checkupPhaseFromProgress(progress) : null;
  const runningIdx = CHECKUP_STEPS.findIndex(s=>s.id === runningPhase);
  const rows = CHECKUP_STEPS.map((s, idx)=>{
    const ph = phases[s.id];
    let status = 'pending';
    let detail = s.detail;
    if(running){
      if(idx < runningIdx) status = 'ok';
      if(idx === runningIdx) {
        status = 'running';
        detail = (progress && progress.msg) || detail;
      }
    }
    if(ph){
      status = ph.skipped ? 'skipped' : ph.aborted ? 'abort'
        : ph.ok ? 'ok' : 'issue';
      detail = ph.summary || ph.error || detail;
    } else if(result && s.id === 'report' && result.log_name){
      status = result.ok ? 'ok' : 'issue';
      detail = `Saved ${result.log_name}`;
    }
    return `<div class="cal-step ${status}">`+
      `<div class="cal-step-name">${s.name}</div>`+
      `<div>${checkupPill(status)}</div>`+
      `<div class="cal-step-detail">${detail || ''}</div>`+
      `</div>`;
  });
  el.innerHTML = rows.join('');
}
function calNum(v, digits=1){
  const n = Number(v);
  return Number.isFinite(n) ? n.toFixed(digits) : '—';
}
function calSigned(v, digits=1){
  const n = Number(v);
  if(!Number.isFinite(n)) return '—';
  return (n >= 0 ? '+' : '') + n.toFixed(digits);
}
function calMm(v, digits=1){
  return calNum(v, digits) + ' mm';
}
function calMaybeMm(v, label, digits=1){
  const n = Number(v);
  return Number.isFinite(n) ? `${label} ${n.toFixed(digits)} mm` : null;
}
function calDelta(v, digits=1){
  const n = Number(v);
  if(!Number.isFinite(n)) return '—';
  return (n >= 0 ? '+' : '') + n.toFixed(digits) + ' mm';
}
function fillCalInput(id, value){
  const el = $(id);
  if(!el) return;
  const n = Number(value);
  if(Number.isFinite(n) && document.activeElement !== el) el.value = n;
}
function paintManualGeometry(manual){
  if(!manual || !manual.ok) return;
  fillCalInput('calhipheight', manual.hip_pitch_height_mm);
  fillCalInput('calhipradius', manual.hip_center_radius_mm);
  fillCalInput('calfemur', manual.femur_mm);
  fillCalInput('caltibia', manual.tibia_mm);
}
function renderCalDimensions(res){
  const el = $('caldims');
  if(!el) return;
  if(!res){
    el.innerHTML = '';
    return;
  }
  const report = res.report || res || {};
  const geom = res.geometry || report.geometry || {};
  const hasGeom = !!(
    geom && (geom.nominal_mm || geom.summary || geom.per_leg || geom.ok));
  if(!hasGeom || geom.ok === false){
    const err = geom && geom.error ? ` (${htmlEscape(geom.error)})` : '';
    el.innerHTML = `<div class="hint">Robot dimensions unavailable${err}</div>`;
    return;
  }
  const nom = geom.nominal_mm || {};
  const gsum = geom.summary || {};
  const plant = geom.plant || {};
  const hint = geom.mujoco_hint || {};
  const manual = geom.manual_measurements || {};
  const fit = geom.effective_fit || {};
  const fitSummary = fit.summary || {};
  const seg = fit.segment_fit || {};
  const segLinks = seg.link_lengths_mm || {};
  const contactSweep = geom.contact_sweep || {};
  const manualHeightFit = gsum.manual_height_fit || contactSweep.manual_height_fit || {};
  const links = hint.link_lengths_m || {};
  const manualLinks = hint.manual_link_lengths_m || {};
  const effLinks = hint.effective_link_lengths_m || {};
  const learned = plant.learned ? 'learned plant' : 'default plant';
  const fitSource = fit.source === 'contact_sweep'
    ? `${fit.sample_count || contactSweep.sample_count || 0} contact samples`
    : 'plant-only estimate';
  const sweepRaw = contactSweep.raw_sample_count || contactSweep.sample_count;
  const sweepSource = contactSweep && sweepRaw
    ? `${contactSweep.status || 'unknown'} · ${contactSweep.sample_count || 0}/${sweepRaw} accepted`
    : 'not run';
  const sweepUseText = contactSweep && sweepRaw
    ? (contactSweep.manual_geometry_mismatch
      ? 'diagnostic only; disagrees with measured geometry'
      : (contactSweep.ok ? 'usable contact consistency check' : 'rejected; using fallback geometry'))
    : 'waiting for run';
  const manualBits = [
    calMaybeMm(manual.hip_pitch_height_mm, 'hip h'),
    calMaybeMm(manual.hip_center_radius_mm, 'center→hip'),
    calMaybeMm(manual.femur_mm, 'femur'),
    calMaybeMm(manual.tibia_mm, 'tibia/boot'),
  ].filter(Boolean);
  const manualSource = manual.learned
    ? `operator measurement${manual.timestamp ? (' · '+htmlEscape(manual.timestamp)) : ''}`
    : 'not saved';
  const consistencyBits = [
    gsum.manual_relative_height_mm != null
      ? `old hip+knee FK ${calMm(gsum.manual_relative_height_mm)} (${calDelta(gsum.manual_relative_minus_manual_height_mm)})`
      : null,
    gsum.manual_absolute_height_mm != null
      ? `active tibia-angle FK ${calMm(gsum.manual_absolute_height_mm)} (${calDelta(gsum.manual_absolute_minus_manual_height_mm)})`
      : null,
  ].filter(Boolean);
  const zeroHyp = gsum.manual_zero_hypotheses || {};
  const zeroModels = zeroHyp.models || {};
  function zeroHypBit(label, row){
    if(!row || !row.ok) return null;
    return `${label} ${calSigned(row.hip_zero_deg)}° hip / `+
      `${calSigned(row.knee_zero_deg)}° knee `+
      `(rms ${calMm(row.rms_error_mm)})`;
  }
  const zeroHypBits = [
    zeroHypBit('best', zeroHyp.best_model),
    zeroHypBit('serial best', zeroHyp.best_serial),
    zeroHypBit('tibia-angle best', zeroModels.absolute_knee_best_pair),
    zeroHypBit('hip-only', zeroModels.serial_hip_only),
    zeroHypBit('knee-only', zeroModels.serial_knee_only),
    zeroHypBit('tibia-angle check', zeroModels.absolute_knee_no_offset),
  ].filter(Boolean);
  const rows = [
    ['manual measurements',
      manualBits.length ? manualBits.join(' · ') : 'not saved',
      manualSource],
    ['link lengths',
      `coxa ${calMm(nom.coxa)} · femur ${calMm(nom.femur)} · tibia ${calMm(nom.tibia)}`,
      links.coxa != null
        ? `MuJoCo m: ${calNum(links.coxa, 5)}, ${calNum(links.femur, 5)}, ${calNum(links.tibia, 5)}`
        : 'CAD model'],
    ['hip center radius',
      `nominal ${calMm(gsum.nominal_hip_center_radius_mm)}`
        +(gsum.manual_hip_center_radius_mm != null
          ? ` · measured ${calMm(gsum.manual_hip_center_radius_mm)}`
          : ''),
      gsum.manual_center_minus_nominal_mm != null
        ? `measured minus nominal ${calDelta(gsum.manual_center_minus_nominal_mm)}`
        : 'flat-to-flat/2 + coxa'],
    ['configured contact links',
      `coxa ${calMm(segLinks.coxa)} · femur ${calMm(segLinks.femur)} · tibia ${calMm(segLinks.tibia)}`,
      seg.status
        ? `${htmlEscape(seg.status)} · ${fitSource}`+
          (seg.link_lengths_observable === false ? ' · links not learned from floor contacts' : '')
        : 'waiting for dimension sweep'],
    ['manual FK consistency',
      consistencyBits.length ? consistencyBits.join(' · ') : 'waiting for measured links + hip height',
      'positive delta means FK predicts a taller robot than tape'],
    ['angle convention',
      gsum.active_angle_convention || hint.angle_convention_key || 'unknown',
      hint.angle_convention || 'from calibration geometry model'],
    ['manual zero hypothesis',
      zeroHypBits.length ? zeroHypBits.join(' · ') : 'waiting for measured links + sweep contacts',
      zeroHyp.sample_count
        ? `${zeroHyp.sample_count} contacts fit to measured height/link lengths`
        : 'report-only diagnostic'],
    ['dimension sweep',
      sweepSource,
      sweepUseText],
    ['chassis',
      `flat-to-flat ${calMm(nom.chassis_flat_to_flat)}`,
      'CAD model'],
    ['stand home',
      `hip ${calSigned(plant.hip_deg)}° · knee ${calSigned(plant.knee_deg)}°`,
      learned + (plant.timestamp ? ` · ${htmlEscape(plant.timestamp)}` : '')],
    ['contact/FK height estimate',
      `mean ${calMm(fitSummary.mean_servo_height_mm)} · spread ${calMm(fitSummary.servo_height_spread_mm)}`,
      (manualHeightFit.delta_mm != null
        ? `diagnostic; vs measured hip ${calDelta(manualHeightFit.delta_mm)}`
        : fitSource)],
    ['zero offset hints',
      `max ${calSigned(fitSummary.max_zero_hint_deg)}°`,
      'relative hints from contact-height residuals'],
    ['stance foot z',
      `mean ${calMm(gsum.mean_foot_z_mm)} · spread ${calMm(gsum.foot_z_spread_mm)}`,
      hint.neutral_foot_z_m != null
        ? `MuJoCo neutral z ${calNum(hint.neutral_foot_z_m, 5)} m`
          +(hint.neutral_foot_z_source ? ` · ${htmlEscape(hint.neutral_foot_z_source)}` : '')
        : 'hip-frame vertical'],
    ['stance reach',
      `radial ${calMm(gsum.mean_radial_mm)} · spread ${calMm(gsum.radial_spread_mm)}`,
      'hip-yaw-frame reach'],
  ];
  if(effLinks && effLinks.femur != null){
    rows.push(['MuJoCo effective links',
      `${calNum(effLinks.coxa, 5)}, ${calNum(effLinks.femur, 5)}, ${calNum(effLinks.tibia, 5)} m`,
      'from dimension sweep fit']);
  }
  if(manualLinks && (manualLinks.femur != null || manualLinks.tibia != null)){
    rows.push(['MuJoCo measured links',
      `${calNum(manualLinks.coxa, 5)}, ${calNum(manualLinks.femur, 5)}, ${calNum(manualLinks.tibia, 5)} m`
        +(manualLinks.hip_center_radius != null
          ? ` · center→hip ${calNum(manualLinks.hip_center_radius, 5)} m`
          : ''),
      'operator measurement; check active convention before tuning gait']);
  }
  const perLeg = Array.isArray(geom.per_leg) ? geom.per_leg : [];
  const legRows = perLeg.map(row => (
    `<tr>`+
      `<td>L${htmlEscape(row.leg)}</td>`+
      `<td>${calSigned(row.yaw_deg)}</td>`+
      `<td>${calSigned(row.hip_deg)}</td>`+
      `<td>${calSigned(row.knee_deg)}</td>`+
      `<td>${calNum(row.z_mm)}</td>`+
      `<td>${calNum(row.radial_mm)}</td>`+
    `</tr>`
  )).join('');
  const fitLegs = Array.isArray(fit.per_leg) ? fit.per_leg : [];
  const fitLegRows = fitLegs.map(row => (
    `<tr>`+
      `<td>L${htmlEscape(row.leg)}</td>`+
      `<td>${htmlEscape(row.samples)}</td>`+
      `<td>${calNum(row.servo_height_mm)}</td>`+
      `<td>${calNum(row.height_spread_mm)}</td>`+
      `<td>${calNum(row.rms_residual_mm)}</td>`+
      `<td>${calNum(row.zero_rms_residual_mm)}</td>`+
      `<td>${calSigned(row.hip_zero_hint_deg)}</td>`+
      `<td>${calSigned(row.knee_zero_hint_deg)}</td>`+
    `</tr>`
  )).join('');
  el.innerHTML =
    `<div class="cal-dim-title">Robot dimensions used for sim</div>`+
    `<table class="cal-table cal-dim-summary"><thead><tr>`+
      `<th>item</th><th>value</th><th>source</th>`+
    `</tr></thead><tbody>`+
      rows.map(r => `<tr><td>${htmlEscape(r[0])}</td>`+
        `<td>${r[1]}</td><td>${r[2]}</td></tr>`).join('')+
    `</tbody></table>`+
    (legRows ? (
      `<div class="cal-dim-title sub">Per-leg plant geometry</div>`+
      `<table class="cal-table"><thead><tr>`+
        `<th>leg</th><th>yaw°</th><th>hip°</th><th>knee°</th>`+
        `<th>foot z mm</th><th>radial mm</th>`+
      `</tr></thead><tbody>${legRows}</tbody></table>`
    ) : '')+
    (fitLegRows ? (
      `<div class="cal-dim-title sub">Per-leg contact-height diagnostic</div>`+
      `<table class="cal-table"><thead><tr>`+
        `<th>leg</th><th>samples</th><th>height mm</th>`+
        `<th>spread mm</th><th>rms mm</th><th>zero rms</th><th>hip zero°</th>`+
        `<th>knee zero°</th>`+
      `</tr></thead><tbody>${fitLegRows}</tbody></table>`
    ) : '');
}
function renderCalResult(res){
  if(!res){
    renderCheckupSteps();
    renderCalDimensions(null);
    return;
  }
  const report = res.report || {};
  const geom = res.geometry || report.geometry || {};
  const gsum = (geom && geom.summary) || {};
  const esum = ((geom && geom.effective_fit) || {}).summary || {};
  const act = res.actuators || report.actuators || {};
  const snap = (act && act.snapshot) || {};
  const learned = act && act.learned_model;
  const prop = res.proprioception || report.proprioception || {};
  const bus = res.bus_power || report.bus_power || {};
  const busSnap = bus.snapshot || {};
  const stability = res.stability_margin || report.stability_margin || {};
  const mass = res.mass_shift || report.mass_shift || {};
  $('calcounts').innerHTML =
    (res.ok
      ? '<span class="cal-pill green">saved</span> checkup complete'
      : '<span class="cal-pill yellow">partial</span> checkup complete')+
    (gsum.mean_foot_z_mm!=null ? ` · foot z ${Number(gsum.mean_foot_z_mm).toFixed(1)}mm` : '')+
    (gsum.foot_z_spread_mm!=null ? ` · spread ${Number(gsum.foot_z_spread_mm).toFixed(1)}mm` : '')+
    (gsum.manual_hip_pitch_height_mm!=null ? ` · measured hip ${Number(gsum.manual_hip_pitch_height_mm).toFixed(1)}mm` :
      (esum.mean_servo_height_mm!=null ? ` · FK height ${Number(esum.mean_servo_height_mm).toFixed(1)}mm` : ''))+
    (gsum.model_minus_manual_height_mm!=null ? ` · FK err ${calDelta(gsum.model_minus_manual_height_mm)}` : '')+
    (prop.max_abs_error_deg!=null ? ` · zero err ${Number(prop.max_abs_error_deg).toFixed(1)}°` : '')+
    (stability.max_measured_tilt_delta_deg!=null ? ` · margin tilt ${Number(stability.max_measured_tilt_delta_deg).toFixed(1)}°` : '')+
    (mass.max_pitch_delta_deg!=null ? ` · mass pitch ${Number(mass.max_pitch_delta_deg).toFixed(1)}°` : '')+
    (mass.max_roll_delta_deg!=null ? ` · roll ${Number(mass.max_roll_delta_deg).toFixed(1)}°` : '')+
    (busSnap.min_volt!=null ? ` · Vmin ${Number(busSnap.min_volt).toFixed(2)}V` : '')+
    (busSnap.max_current_a!=null ? ` · Ipeak ${Number(busSnap.max_current_a).toFixed(2)}A` : '')+
    (snap.live_joints!=null ? ` · ${snap.live_joints} actuator snapshots` : '')+
    (learned ? ' · motor model loaded' : '')+
    (res.msg ? `<div class="hint" style="margin-top:6px">${res.msg}</div>` : '');
  $('callog').textContent = res.log_name
    ? `Report: logs/${res.log_name}`
    : (res.path ? `Report: ${res.path}` : 'Report: —');
  renderCheckupSteps({result: res});
  renderCalDimensions(res);
  if(geom.manual_measurements) paintManualGeometry(geom.manual_measurements);
  if(geom.plant) paintPlantInfo(geom.plant);
  if(res.imu || report.imu) paintImuInfo(res.imu || report.imu);
}
async function refreshManualGeometry(){
  try{
    const r = await fetch('/api/geometry/manual?t='+Date.now(), {cache:'no-store'});
    if(!r.ok) return null;
    const d = await r.json();
    paintManualGeometry(d);
    return d;
  }catch(e){
    return null;
  }
}
async function refreshCalibrate(){
  try{
    const r = await fetch('/api/calibrate?t='+Date.now(), {cache:'no-store'});
    if(!r.ok) throw 0;
    const d = await r.json();
    if(d.plant) paintPlantInfo(d.plant);
    if(d.imu) paintImuInfo(d.imu);
    const p = d.progress || {};
    const displayResult = calibrationDisplayResult(d);
    renderCheckupSteps({
      running: !!d.running,
      progress: p,
      result: displayResult,
    });
    if(d.running){
      $('calstatus').textContent = p.msg || 'Running…';
      $('calrun').disabled = true;
    } else {
      $('calrun').disabled = false;
      if(displayResult && displayResult.ok){
        $('calstatus').textContent = displayResult.aborted ? 'Aborted.' : 'Done.';
        renderCalResult(displayResult);
      } else if(displayResult && displayResult.error){
        $('calstatus').textContent = 'Error: '+displayResult.error;
      } else if(!($('calstatus').textContent||'').match(/Done|Error|Aborted/)){
        // keep last status
      }
    }
    if(displayResult) renderCalResult(displayResult);
  }catch(e){
    $('calstatus').textContent = 'Calibrate status failed (link?)';
  }
}
renderCheckupSteps();
async function saveManualGeometry(){
  const read = (id)=>{
    const el = $(id);
    if(!el || String(el.value).trim() === '') return null;
    const n = Number(el.value);
    return Number.isFinite(n) ? n : null;
  };
  const body = {
    hip_pitch_height_mm: read('calhipheight'),
    hip_center_radius_mm: read('calhipradius'),
    femur_mm: read('calfemur'),
    tibia_mm: read('caltibia'),
  };
  $('calstatus').textContent = 'Saving measured geometry…';
  try{
    const r = await fetch('/api/geometry/manual', {
      method:'POST',
      headers:{'Content-Type':'application/json'},
      body: JSON.stringify(body),
    });
    const d = await r.json();
    if(!d.ok){
      $('calstatus').textContent = d.error || 'Save failed';
      return;
    }
    paintManualGeometry(d);
    $('calstatus').textContent = 'Measured geometry saved.';
    await refreshCalibrate();
  }catch(e){
    $('calstatus').textContent = 'Save failed';
  }
}
$('calrun').onclick = async ()=>{
  $('calstatus').textContent = 'Starting…';
  $('calrun').disabled = true;
  try{
    const r = await fetch('/api/calibrate', {
      method:'POST',
      headers:{'Content-Type':'application/json'},
      body: JSON.stringify({
        mode: 'checkup',
        clearance_mm: 25,
      }),
    });
    const d = await r.json();
    if(!d.ok){
      $('calstatus').textContent = d.error || 'failed to start';
      $('calrun').disabled = false;
      return;
    }
    startCalPoll();
    refreshCalibrate();
  }catch(e){
    $('calstatus').textContent = 'Start failed';
    $('calrun').disabled = false;
  }
};
$('calstop').onclick = async ()=>{
  await fetch('/api/calibrate/stop', {method:'POST'});
  $('calstatus').textContent = 'Stopping…';
  refreshCalibrate();
};
$('calrefresh').onclick = ()=> refreshCalibrate();
$('calsavegeom').onclick = ()=> saveManualGeometry();
let rlTimer = null;
function startRlPoll(){
  if(rlTimer) clearInterval(rlTimer);
  rlTimer = setInterval(async ()=>{
    try{
      const r = await fetch('/api/calibrate?t='+Date.now(), {cache:'no-store'});
      const d = await r.json();
      const p = d.progress || {};
      if(d.running){
        $('rlstatus').textContent = p.msg || 'running…';
      } else {
        clearInterval(rlTimer); rlTimer = null;
        rlButtons(false);
        drvLockRlControls(false);
        const res = d.result || {};
        const peak = res.max_current_a ?? res.peak_a ?? '?';
        $('rlstatus').textContent = res.ok
          ? `Done · max ${peak} A`
             + (res.limped ? ' · limped' : ' · holding')
          : (res.error || 'done');
      }
    }catch(e){ /* keep polling */ }
  }, 500);
}
function drvClearHeartbeat(){
  if(drvHb){ clearInterval(drvHb); drvHb = null; }
  if(drvPadReleaseTimer){
    clearTimeout(drvPadReleaseTimer);
    drvPadReleaseTimer = null;
  }
}
function drvResetLocalInput(){
  drvKeys.clear();
  drvPad = null;
}
function rlButtons(disabled){
  for(const id of ['rlstand','rllower','rlwalkfwd',
                   'rlwalkleft','rlwalkright','rlwalkback',
                   'rlwalkfl','rlwalkfr','rlwalkbl','rlwalkbr',
                   'rldrivestart','rldriveend'])
    $(id).disabled = disabled;
}
async function rlMove(mode, body){
  // No confirms anywhere (operator 08-11: no warning modals);
  // the server preflight refuses bad start poses.
  if(drvActive){
    $('rlstatus').textContent = 'End the drive session before starting another RL move.';
    return;
  }
  if(mode!=='stand' && mode!=='lower' && body)
    delete body.heading;   // UI-only label, not an API field
  $('rlstatus').textContent = 'Request sent…';
  rlButtons(true);
  try{
    const r = await fetch('/api/rl/'+mode, {method:'POST',
      body: body ? JSON.stringify(body) : undefined});
    const d = await r.json();
    if(!d.ok){
      $('rlstatus').textContent = requestReceiptLine(d, 'RL '+mode)
        + '; refused: '+(d.error || 'unknown');
      showErr('RL: '+requestReceiptLine(d, mode)+'; '
        +(d.error || 'refused'));
      rlButtons(false);
      return;
    }
    $('rlstatus').textContent = requestReceiptLine(d, 'RL '+mode)
      + '; running…';
    startRlPoll();
  }catch(e){
    $('rlstatus').textContent = 'Start failed (link?)';
    rlButtons(false);
  }
}
async function rlScriptedStand(mode, label, direction='up'){
  if(drvActive){
    $('rlstatus').textContent = 'End the drive session before stand/lower.';
    return;
  }
  $('rlstatus').textContent = label+' request sent…';
  rlButtons(true);
  try{
    const r = await fetch('/api/standup', {method:'POST',
      body: JSON.stringify({mode, speed:10, direction})});
    const d = await r.json();
    if(!d.ok){
      $('rlstatus').textContent = requestReceiptLine(d, label)
        + '; refused: '+(d.error || 'unknown');
      showErr(label+': '+requestReceiptLine(d, '')+'; '
        +(d.error || 'refused'));
      rlButtons(false);
      return;
    }
    $('rlstatus').textContent = requestReceiptLine(d, label)+'; moving…';
    startRlPoll();
  }catch(e){
    $('rlstatus').textContent = 'Start failed (link?)';
    rlButtons(false);
  }
}
$('rlstand').onclick = ()=> rlMove('stand');
$('rllower').onclick = ()=> rlScriptedStand('step', 'Lower', 'down');
function rlWalk(dx, dy, heading){
  const s = parseFloat($('rlwalkspeed').value);
  const n = Math.hypot(dx, dy) || 1;   // unit heading so speed = |v|
  rlMove('walk', {vx: dx/n*s, vy: dy/n*s, heading,
                  duration_s: parseFloat($('rlwalkdur').value)});
}
// Body frame: +vx forward, +vy left. Off-wedge headings are folded onto
// the trained wedge by the rot-60 canonicalizer in rl_policy.py.
$('rlwalkfwd').onclick   = ()=> rlWalk( 1,  0, 'FORWARD');
$('rlwalkback').onclick  = ()=> rlWalk(-1,  0, 'BACKWARD');
$('rlwalkleft').onclick  = ()=> rlWalk( 0,  1, 'strafe LEFT');
$('rlwalkright').onclick = ()=> rlWalk( 0, -1, 'strafe RIGHT');
$('rlwalkfl').onclick    = ()=> rlWalk( 1,  1, 'diagonal FWD-LEFT');
$('rlwalkfr').onclick    = ()=> rlWalk( 1, -1, 'diagonal FWD-RIGHT');
$('rlwalkbl').onclick    = ()=> rlWalk(-1,  1, 'diagonal BACK-LEFT');
$('rlwalkbr').onclick    = ()=> rlWalk(-1, -1, 'diagonal BACK-RIGHT');
$('rlstop').onclick = async ()=>{
  await fetch('/api/rl/stop', {method:'POST'});
  $('rlstatus').textContent = 'Stopping (holds pose; X to limp)…';
};

// ---- Drive session (hold arrow keys — MuJoCo-viewer-style, 08-11) ---------
// The browser streams (vx, vy, wz) heartbeats at 5 Hz while the session is
// active; the robot's 25 Hz loop slews toward them and treats anything
// older than 0.6 s as "keys released". So: keydown = walk, keyup = stop
// and hold, dead tab = stop and hold.
let drvActive = false, drvHb = null;
const drvKeys = new Set();
let drvPad = null;   // on-screen pad vector [dx, dy] while held
let drvPadDownAt = 0;
let drvPadReleaseTimer = null;
const DRV_TAP_PULSE_MS = 650;
const DRV_KEYMAP = {
  arrowup:'fwd', w:'fwd', i:'fwd',
  arrowdown:'back', s:'back', k:'back',
  arrowleft:'left', a:'left', j:'left',
  arrowright:'right', d:'right', l:'right',
  q:'yawL', u:'yawL',
  e:'yawR', o:'yawR',
};
function drvLockRlControls(active){
  for(const id of ['rlstand','rllower','rlwalkfwd',
                   'rlwalkleft','rlwalkright','rlwalkback',
                   'rlwalkfl','rlwalkfr','rlwalkbl','rlwalkbr']){
    $(id).disabled = active;
  }
  const start = $('rldrivestart');
  start.disabled = active;
  start.textContent = active ? 'Driving active' : '▶ Start driving';
  $('rldriveend').disabled = !active;
}
function drvBlockStart(label, detail){
  const start = $('rldrivestart');
  start.disabled = true;
  start.textContent = label;
  $('rldriveend').disabled = true;
  if(detail) $('rldrivestatus').textContent = detail;
}
function drvStartReady(detail){
  const start = $('rldrivestart');
  start.disabled = false;
  start.textContent = '▶ Start driving';
  $('rldriveend').disabled = true;
  if(detail) $('rldrivestatus').textContent = detail;
}
function uiEvent(event, data={}){
  try{
    fetch('/api/ui_event', {
      method:'POST', cache:'no-store', keepalive:true,
      headers:{'Content-Type':'application/json'},
      body: JSON.stringify(Object.assign({event, view:activeView}, data)),
    }).catch(()=>{});
  }catch(e){}
}
function drvClearPad(){
  if(drvPadReleaseTimer){
    clearTimeout(drvPadReleaseTimer);
    drvPadReleaseTimer = null;
  }
  if(drvPad){
    drvPad = null;
    drvSend();
  }
}
function drvVec(){
  let dx = (drvKeys.has('fwd')?1:0) - (drvKeys.has('back')?1:0);
  let dy = (drvKeys.has('left')?1:0) - (drvKeys.has('right')?1:0);
  let dz = (drvKeys.has('yawL')?1:0) - (drvKeys.has('yawR')?1:0);
  if(!dx && !dy && drvPad){ dx = drvPad[0]; dy = drvPad[1]; }
  const n = Math.hypot(dx, dy);
  const s = parseFloat($('rlwalkspeed').value);
  const wz = dz ? dz * 0.18 : 0;
  return n ? [dx/n*s, dy/n*s, wz] : [0, 0, wz];
}
function drvPaint(d){
  const live = (d && d.live) || {};
  const bits = [];
  if(live.model) bits.push(`model <b>${live.model}</b>`);
  if(live.vx_ref!=null)
    bits.push(`v (${Math.round(live.vx_ref*1000)}, `
              + `${Math.round(live.vy_ref*1000)}) mm/s`);
  if(live.wz_ref) bits.push(`ω ${live.wz_ref} rad/s`);
  if(live.roll_deg!=null)
    bits.push(`tilt ${live.roll_deg}/${live.pitch_deg}°`);
  if(live.max_current_a!=null) bits.push(`maxI ${live.max_current_a} A`);
  if(live.rot60_k) bits.push(`sec ${live.rot60_k>0?'+':''}${live.rot60_k}`);
  if(live.t_s!=null) bits.push(`${live.t_s}s`);
  $('rldrivestatus').innerHTML =
    `<b style="color:#5fd08a">DRIVING</b> — hold arrows/WASD · `
    + (bits.length ? bits.join(' · ') : (d && d.status) || 'starting…');
}
async function drvSend(){
  if(!drvActive) return;
  const [vx, vy, wz] = drvVec();
  try{
    const r = await fetch('/api/rl/drive/cmd', {method:'POST',
      body: JSON.stringify({vx, vy, wz})});
    const d = await r.json();
    if(!d.active){ drvEnded(); return; }
    drvPaint(d);
  }catch(e){ /* link blip — watchdog on the robot handles it */ }
}
async function drvEnded(){
  drvActive = false;
  drvClearHeartbeat();
  drvResetLocalInput();
  drvLockRlControls(false);
  try{
    const d = await (await fetch('/api/rl/drive', {cache:'no-store'})).json();
    const res = d.result || {};
    $('rldrivestatus').textContent = 'Session ended'
      + (res.ended ? ` — ${res.ended}` : res.error ? ` — ${res.error}` : '')
      + (res.max_current_a!=null ? ` · maxI ${res.max_current_a} A` : '')
      + ' · holding (X to limp).';
  }catch(e){ $('rldrivestatus').textContent = 'Session ended — holding.'; }
}
$('rldrivestart').onclick = async ()=>{
  // Recover from stale browser state after a service restart/deploy.
  uiEvent('rl_drive_start_click', {
    button:'rldrivestart',
    disabled:$('rldrivestart').disabled,
    active:drvActive,
    status:$('rldrivestatus').textContent,
  });
  drvActive = false;
  drvClearHeartbeat();
  drvResetLocalInput();
  drvLockRlControls(true);
  $('rldrivestart').textContent = 'Starting...';
  $('rldrivestatus').textContent = 'Starting session (preflight'
    + ' — no auto-stance move)…';
  try{
    const r = await fetch('/api/rl/drive/start', {
      method:'POST', cache:'no-store'});
    const d = await r.json();
    if(!d.ok){
      $('rldrivestatus').textContent = 'Refused: '+(d.error || 'unknown');
      showErr('Drive: '+(d.error || 'refused'));
      drvLockRlControls(false);
      $('rldrivestart').textContent = 'Refused';
      return;
    }
    drvActive = true;
    drvKeys.clear(); drvPad = null;
    if(drvHb) clearInterval(drvHb);
    drvHb = setInterval(drvSend, 200);
    drvSend();  // immediate zero-command heartbeat; no 200 ms blind spot
    drvPaint(d);
  }catch(e){
    $('rldrivestatus').textContent = 'Start failed (link?)';
    drvLockRlControls(false);
    $('rldrivestart').textContent = 'Start failed';
  }
};
$('rldriveend').onclick = async ()=>{
  $('rldriveend').disabled = true;
  $('rldrivestatus').textContent = 'Ending session (rolls to a stop, holds)…';
  try{ await fetch('/api/rl/drive/stop', {method:'POST'}); }catch(e){}
  // Heartbeats keep flowing until the server reports inactive, so the
  // decel + wind-down is visible in the status line.
};
drvLockRlControls(false);

async function refreshRlRuntimeState(){
  let rlMoveRunning = false;
  try{
    const r = await fetch('/api/calibrate?t='+Date.now(), {cache:'no-store'});
    const d = await r.json();
    rlMoveRunning = !!(d.running && (d.name||'').startsWith('rl_policy'));
    if(rlMoveRunning){
      $('rlstatus').textContent = (d.progress||{}).msg || 'running…';
      rlButtons(true);
      if(!rlTimer) startRlPoll();
      return true;
    }
    if(rlTimer){ clearInterval(rlTimer); rlTimer = null; }
  }catch(e){
    return false;
  }

  try{
    const d = await (await fetch('/api/rl/drive', {cache:'no-store'})).json();
    if(d.active){
      drvActive = true;
      drvLockRlControls(true);
      if(!drvHb) drvHb = setInterval(drvSend, 200);
      drvPaint(d);
      return true;
    }
  }catch(e){
    return false;
  }

  drvActive = false;
  drvClearHeartbeat();
  drvResetLocalInput();
  drvLockRlControls(false);
  try{
    const rb = await (await fetch('/api/robot', {cache:'no-store'})).json();
    if(rb && !rb.armed){
      drvBlockStart('Stand up first',
        'Blocked: robot is limp/disarmed. Use RL Stand Up / Walk Ready first.');
      return true;
    }
  }catch(e){}
  drvStartReady('Ready — Start driving, then hold arrows/WASD or the pad.');
  return true;
}

setInterval(()=>{ if(activeView === 'rl') refreshRlRuntimeState(); }, 2000);
// Keys drive ONLY from the RL tab (the Drive tab's own key loop streams
// scripted-gait J commands — never both at once). Leaving the tab or
// window counts as releasing everything.
window.addEventListener('keydown', (e)=>{
  if(!drvActive || activeView !== 'rl') return;
  const tag = (e.target && e.target.tagName || '').toLowerCase();
  if(tag === 'input' || tag === 'select' || tag === 'textarea') return;
  const dir = DRV_KEYMAP[e.key.toLowerCase()];
  if(!dir) return;
  e.preventDefault();
  if(!drvKeys.has(dir)){ drvKeys.add(dir); drvSend(); }
});
window.addEventListener('keyup', (e)=>{
  if(!drvActive) return;
  const dir = DRV_KEYMAP[e.key.toLowerCase()];
  if(!dir) return;
  e.preventDefault();
  if(drvKeys.delete(dir)) drvSend();
});
// Lost focus = treat every key as released (missed keyup otherwise).
window.addEventListener('blur', ()=>{
  if(drvActive && drvKeys.size){ drvKeys.clear(); drvSend(); }
});
for(const b of document.querySelectorAll('#rldrivepad button[data-dv]')){
  const dv = b.dataset.dv.split(',').map(Number);
  if(!dv[0] && !dv[1]) continue;          // center "hold" cell
  const down = (e)=>{ e.preventDefault();
    if(!drvActive) return;
    if(drvPadReleaseTimer){
      clearTimeout(drvPadReleaseTimer);
      drvPadReleaseTimer = null;
    }
    drvPad = dv;
    drvPadDownAt = performance.now();
    drvSend();
  };
  const up = ()=>{
    if(drvPad !== dv) return;
    const heldMs = performance.now() - drvPadDownAt;
    const delayMs = Math.max(0, DRV_TAP_PULSE_MS - heldMs);
    if(drvPadReleaseTimer) clearTimeout(drvPadReleaseTimer);
    drvPadReleaseTimer = setTimeout(drvClearPad, delayMs);
  };
  b.addEventListener('pointerdown', down);
  b.addEventListener('pointerup', up);
  b.addEventListener('pointerleave', up);
  b.addEventListener('pointercancel', up);
}

// ---- MuJoCo backend panel --------------------------------------------------
function stopSimPoll(){
  if(simTimer){ clearInterval(simTimer); simTimer = null; }
  simFrameBusy = false;
}
function simPollMaybe(){
  if(!targetHasSim || activeView !== 'rl'){ stopSimPoll(); return; }
  if(!simTimer) simTimer = setInterval(refreshSimPanel, 500);
  refreshSimPanel();
}
async function refreshSimPanel(){
  if(!targetHasSim || activeView !== 'rl' || simBusy) return;
  simBusy = true;
  try{
    const img = $('simframe');
    const now = Date.now();
    if(img && simFrames && !simFrameBusy && now - simFrameLastAt >= 1000){
      simFrameBusy = true;
      simFrameLastAt = now;
      img.onload = img.onerror = ()=>{ simFrameBusy = false; };
      img.src = '/api/sim/frame.jpg?t='+now;
    }
    const d = await (await fetch('/api/sim/state?t='+Date.now(),
      {cache:'no-store'})).json();
    const live = d.live || {};
    const bits = [];
    if(live.mode) bits.push(`<b>${live.mode}</b>`);
    if(live.height_mm!=null) bits.push(`h ${live.height_mm} mm`);
    if(live.vx_ref!=null) bits.push(`v ${Math.round(live.vx_ref*1000)},`
      + `${Math.round(live.vy_ref*1000)} mm/s`);
    if(live.roll_deg!=null) bits.push(`tilt ${live.roll_deg}/`
      + `${live.pitch_deg}°`);
    if(live.t_s!=null) bits.push(`${live.t_s}s`);
    if(d.viewer) bits.push('native viewer');
    $('simstatus').innerHTML = (live.status || d.status || 'ready')
      + (bits.length ? ' · '+bits.join(' · ') : '');
  }catch(e){
    $('simstatus').textContent = 'sim state unavailable';
  } finally {
    simBusy = false;
  }
}
async function simPost(path, body){
  try{
    const r = await fetch(path, {method:'POST',
      body: JSON.stringify(body || {})});
    const d = await r.json();
    $('simstatus').textContent = d.ok
      ? (d.status || 'ok') : (d.error || 'failed');
    refreshSimPanel();
  }catch(e){ $('simstatus').textContent = 'sim command failed'; }
}
function setSyncPoseBusy(busy){
  ['hubsyncpose', 'simsyncpose'].forEach(id=>{
    const b = $(id);
    if(!b) return;
    b.disabled = busy;
    if(id === 'hubsyncpose') b.textContent = busy ? 'Matching…' : 'Match pose';
  });
}
async function syncRobotPoseToSim(){
  if(!(robotTargetAvailable && simTargetAvailable)){
    showSent('connect Robot + MuJoCo before matching pose', true);
    return;
  }
  setSyncPoseBusy(true);
  setTargetLineMsg('robot', 'reading pose…', 'warn');
  setTargetLineMsg('sim', 'waiting…', 'warn');
  if($('simstatus')) $('simstatus').textContent = 'reading robot pose…';
  showSent('matching MuJoCo pose to robot…');
  try{
    const r = await fetch('/api/sim/sync_robot_pose', {method:'POST'});
    const d = await r.json();
    const line = d.ok
      ? 'matched pose · '+(d.live_joints || 0)+'/18 joints'
      : (d.error || 'match pose failed');
    if(d.ok){
      clearTargetLineMsg('robot');
      setTargetLineMsg('sim', line, 'ok');
      setTimeout(()=> clearTargetLineMsg('sim'), 2500);
    } else {
      const which = classifyTargetError(line, 'robot');
      setTargetLineMsg(which, line, 'bad');
      if(which === 'robot') clearTargetLineMsg('sim');
      if(which === 'sim') clearTargetLineMsg('robot');
    }
    if($('simstatus')) $('simstatus').textContent = d.ok
      ? (d.status || line) : line;
    showSent(line, !d.ok);
    refreshSimPanel();
  }catch(e){
    setTargetLineMsg('sim', e.message || 'match pose failed', 'bad');
    clearTargetLineMsg('robot');
    if($('simstatus')) $('simstatus').textContent = 'match pose failed';
    showSent('match pose failed', true);
  }finally{
    setSyncPoseBusy(false);
  }
}
if($('simresetstand'))
  $('simresetstand').onclick = ()=> simPost('/api/sim/reset',
    {start:'plant'});
if($('simresetbelly'))
  $('simresetbelly').onclick = ()=> simPost('/api/sim/reset',
    {start:'belly'});
if($('hubsyncpose'))
  $('hubsyncpose').onclick = syncRobotPoseToSim;
if($('simsyncpose'))
  $('simsyncpose').onclick = syncRobotPoseToSim;
if($('simfall')) $('simfall').onclick = ()=> simPost('/api/sim/fall');
if($('simrecover')) $('simrecover').onclick =
  ()=> simPost('/api/sim/recover');
if($('simpush')) $('simpush').onclick = ()=> simPost('/api/sim/push',
  {x:4, y:0});

// ---- Stand-up lab (baked strategies from rl_move/sim/compare_standup.py) --
let suModes = [], suSel = null, suTimer = null;
async function suLoadModes(){
  try{
    const r = await fetch('/api/standup/modes', {cache:'no-store'});
    const d = await r.json();
    if(!d.ok){ $('sulab-desc').textContent = d.error || 'modes unavailable'; return; }
    suModes = d.modes || [];
    const box = $('sulab-modes'); box.innerHTML = '';
    for(const m of suModes){
      const b = document.createElement('button');
      b.textContent = m.name; b.dataset.mode = m.name;
      b.onclick = ()=> suSelect(m.name);
      box.appendChild(b);
    }
    if(suModes.length)
      suSelect(suModes.some(m=>m.name==='tuck') ? 'tuck' : suModes[0].name);
  }catch(e){ $('sulab-desc').textContent = 'modes unavailable (link?)'; }
}
function suSelect(name){
  suSel = name;
  for(const b of $('sulab-modes').children)
    b.classList.toggle('on', b.dataset.mode===name);
  const m = suModes.find(x=>x.name===name);
  $('sulab-desc').textContent = m
    ? `${m.description} (~${m.total_s}s, ${m.keyframes} keyframes)` : '—';
}
function suNameMode(name){
  if(!name) return 'standup';
  return String(name).replace(/^standup_/, '').replace(/_down$/, '');
}
function suDoneLine(robotState, calState){
  const demo = (robotState && robotState.demo)
    || (calState && calState.demo) || {};
  const res = (calState && calState.result) || {};
  const tel = demo.telemetry || {};
  const params = demo.params || {};
  const name = demo.name || calState?.name || '';
  const mode = res.mode || params.mode || suNameMode(name);
  const direction = res.direction || params.direction
    || (String(name).endsWith('_down') ? 'down' : 'up');
  const verb = direction === 'down' ? 'sit' : 'stand';
  const ok = tel.ok != null ? !!tel.ok : res.ok !== false;
  const bits = [(ok ? 'Done' : 'Failed'), mode, verb];
  if(res.peak_a != null) bits.push(`peak ${res.peak_a} A`);
  if(tel.height_mm != null) bits.push(`height ${tel.height_mm} mm`);
  if(tel.max_lag_deg != null) bits.push(`lag ${tel.max_lag_deg}°`);
  const msg = ok ? bits.join(' · ') : (res.error || robotState?.detail
    || bits.join(' · '));
  return msg + ' — holding (X to limp)';
}
function suPoll(){
  if(suTimer) clearInterval(suTimer);
  suTimer = setInterval(async ()=>{
    try{
      const [rr, cr] = await Promise.all([
        fetch('/api/demo/status?t='+Date.now(), {cache:'no-store'}),
        fetch('/api/calibrate?t='+Date.now(), {cache:'no-store'}),
      ]);
      const robotState = await rr.json();
      const d = await cr.json();
      const demo = robotState.demo || d.demo || {};
      const name = demo.name || d.name || '';
      const running = !!(demo.running || d.running);
      if(running && String(name).startsWith('standup_')){
        const p = demo.progress || d.progress || {};
        $('sulab-status').textContent = p.msg || demo.status || 'running…';
      } else if(!running){
        clearInterval(suTimer); suTimer = null;
        $('sulab-go').disabled = false; $('sulab-sit').disabled = false;
        $('sulab-status').textContent = String(name).startsWith('standup_')
          ? suDoneLine(robotState, d)
          : 'Stopped — holding (X to limp)';
      }
    }catch(e){ /* keep polling */ }
  }, 500);
}
async function suRun(direction){
  if(!suSel) return;
  // No confirm dialog (operator request 08-10): the server refuses a
  // bad start pose and that lands in the status line + error bar.
  $('sulab-go').disabled = true; $('sulab-sit').disabled = true;
  $('sulab-status').textContent = 'Request sent…';
  try{
    const r = await fetch('/api/standup', {method:'POST',
      body: JSON.stringify({mode: suSel, direction,
                            speed: parseFloat($('sulab-speed').value)})});
    const d = await r.json();
    if(!d.ok){
      $('sulab-status').textContent = requestReceiptLine(d, 'Stand-up lab')
        + '; refused: '+(d.error || 'unknown');
      showErr('Stand-up lab: '+requestReceiptLine(d, '')+'; '
        +(d.error || 'refused'));
      $('sulab-go').disabled = false; $('sulab-sit').disabled = false;
      return;
    }
    $('sulab-status').textContent = requestReceiptLine(d, 'Stand-up lab')
      + '; running…';
    suPoll();
  }catch(e){
    $('sulab-status').textContent = 'Start failed (link?)';
    $('sulab-go').disabled = false; $('sulab-sit').disabled = false;
  }
}
$('sulab-go').onclick = ()=> suRun('up');
$('sulab-sit').onclick = ()=> suRun('down');
$('sulab-stop').onclick = async ()=>{
  $('sulab-status').textContent = 'Stop request sent…';
  const r = await fetch('/api/standup/stop', {method:'POST'});
  const d = await r.json().catch(()=>({ok:r.ok}));
  $('sulab-status').textContent = requestReceiptLine(d, 'Stop')
    + '; holding pose';
};
suLoadModes();

// ---- Measure tab (operator data collection -> logs/measurements.jsonl) ----
let muTimer = null;
function muDescribePending(p){
  if(!p) return null;
  const bits = [`<b>${p.kind}</b> (${p.stamp})`];
  if(p.kind === 'hold_current') bits.push(`label ${p.label}`);
  if(p.commanded_mm != null)
    bits.push(`commanded ~${Math.round(p.commanded_mm)} mm`);
  if(p.commanded_rot_deg)
    bits.push(`commanded rot ${p.commanded_rot_deg}° (+ = CW)`);
  if(p.walked_s != null) bits.push(`ran ${p.walked_s}s`);
  if(p.stopped && p.stopped !== 'duration')
    bits.push(`<b style="color:#ff7b72">stopped: ${p.stopped}</b>`);
  if(p.bus_a_mean != null) bits.push(`bus mean ${p.bus_a_mean} A`);
  return bits.join(' · ');
}
function muDescribeRecord(r){
  const bits = [`<b>${r.kind}</b> ${r.stamp || ''}`];
  if(r.label) bits.push(r.label);
  if(r.commanded_mm != null) bits.push(`cmd ${Math.round(r.commanded_mm)}mm`);
  if(r.measured_mm != null) bits.push(`meas ${Math.round(r.measured_mm)}mm`);
  if(r.slip_ratio_measured_over_commanded != null)
    bits.push(`ratio ${r.slip_ratio_measured_over_commanded}`);
  if(r.kind === 'onboard_slip' && r.grade) bits.push(`grade ${r.grade}`);
  if(r.summary) bits.push(r.summary);
  if(r.observed_turn) bits.push(`turned ${r.observed_turn}`);
  if(r.measured_rot_deg != null) bits.push(`${r.measured_rot_deg}°`);
  if(r.bus_a_mean != null) bits.push(`bus ${r.bus_a_mean}A mean`);
  if(r.notes) bits.push(`“${r.notes}”`);
  return bits.join(' · ');
}
async function muRefresh(){
  try{
    const r = await fetch('/api/measure/list', {cache:'no-store'});
    const d = await r.json();
    if(!d.ok) throw new Error(d.error || 'list failed');
    const p = d.pending;
    $('mu-pending-form').style.display = p ? '' : 'none';
    $('mu-pending-desc').innerHTML = p
      ? muDescribePending(p)
      : 'No run waiting for a reading. Start one below; when it '
        + 'finishes, enter the physical measurement here.';
    const recs = d.records || [];
    $('mu-list').innerHTML = recs.length
      ? recs.map(muDescribeRecord).join('<br>')
      : 'No saved measurements yet.';
  }catch(e){ $('mu-list').textContent = 'list unavailable (link?)'; }
}
function muPollMaybe(){
  if(muTimer) clearInterval(muTimer);
  muTimer = setInterval(async ()=>{
    if(activeView !== 'measure'){ clearInterval(muTimer); muTimer = null; return; }
    try{
      const r = await fetch('/api/calibrate?t='+Date.now(), {cache:'no-store'});
      const d = await r.json();
      if(d.running && (d.name||'').startsWith('measure_')){
        $('mu-status').textContent = (d.progress||{}).msg || 'running…';
      } else {
        $('mu-status').textContent =
          ((d.result && !d.result.ok && d.result.error)
           ? 'Failed: '+d.result.error
           : (d.progress||{}).msg || 'Idle.');
        clearInterval(muTimer); muTimer = null;
        muRefresh();
      }
    }catch(e){ /* keep polling */ }
  }, 700);
}
async function muStart(path, body, _confirmText){
  // confirmText retired (operator 08-11: no warning modals)
  $('mu-status').textContent = 'Starting…';
  try{
    const r = await fetch(path, {method:'POST', body: JSON.stringify(body)});
    const d = await r.json();
    if(!d.ok){
      $('mu-status').textContent = 'Refused: '+(d.error || 'unknown');
      showErr('Measure: '+(d.error || 'refused'));
      return;
    }
    $('mu-status').textContent = 'Running…';
    muPollMaybe();
  }catch(e){ $('mu-status').textContent = 'Start failed (link?)'; }
}
$('mu-walkgo').onclick = ()=>{
  const v = parseFloat($('mu-walkspeed').value);
  const s = parseFloat($('mu-walkdur').value);
  muStart('/api/measure/walk', {vx_mm: v, duration_s: s},
    `Robot will WALK forward ${s}s at ${v} mm/s (scripted gait). `
    + 'Tape in place, robot ARMED + standing, you watching?');
};
$('mu-turnpos').onclick = ()=> muStart('/api/measure/walk',
  {vx_mm: 0, omega: 0.3, duration_s: 6},
  'Robot will TURN IN PLACE 6s at +0.3 rad/s. Watching?');
$('mu-turnneg').onclick = ()=> muStart('/api/measure/walk',
  {vx_mm: 0, omega: -0.3, duration_s: 6},
  'Robot will TURN IN PLACE 6s at -0.3 rad/s. Watching?');
$('mu-holdgo').onclick = ()=>{
  const label = $('mu-holdlabel').value;
  muStart('/api/measure/hold',
    {label, duration_s: parseFloat($('mu-holddur').value)},
    label === 'hover'
      ? 'Robot should be PROPPED UP with feet hanging free. It will '
        + 'torque on and HOLD the present pose (no motion). Ready?'
      : 'Robot will torque on and HOLD the present pose (no motion). '
        + 'Feet planted on the floor?');
};
$('mu-slipgo').onclick = ()=>{
  muStart('/api/measure/slip', {},
    'Robot will run the onboard loaded-vs-hover slip probe. Watching?');
};
$('mu-save').onclick = async ()=>{
  const fields = {
    measured_mm: $('mu-measured').value,
    lateral_drift_mm: $('mu-drift').value,
    measured_rot_deg: $('mu-rot').value,
    observed_turn: $('mu-observed').value,
    notes: $('mu-notes').value,
  };
  try{
    const r = await fetch('/api/measure/annotate',
      {method:'POST', body: JSON.stringify(fields)});
    const d = await r.json();
    $('mu-status').textContent = d.ok
      ? 'Saved.' : 'Save failed: '+(d.error || 'unknown');
    if(d.ok){
      for(const id of ['mu-measured','mu-drift','mu-rot','mu-notes'])
        $(id).value = '';
      $('mu-observed').value = '';
    }
  }catch(e){ $('mu-status').textContent = 'Save failed (link?)'; }
  muRefresh();
};
$('mu-discard').onclick = async ()=>{
  await fetch('/api/measure/discard', {method:'POST'});
  muRefresh();
};
$('mu-rlsave').onclick = async ()=>{
  const body = {kind: 'rl_walk_tape', fields: {
    commanded_mm: $('mu-rlcmd').value,
    measured_mm: $('mu-rlmeas').value,
    notes: $('mu-rlnotes').value,
  }};
  try{
    const r = await fetch('/api/measure/note',
      {method:'POST', body: JSON.stringify(body)});
    const d = await r.json();
    $('mu-status').textContent = d.ok
      ? `Saved (attached ${((d.record||{}).rl_episode_csv) || 'no episode csv'}).`
      : 'Save failed: '+(d.error || 'unknown');
    if(d.ok)
      for(const id of ['mu-rlcmd','mu-rlmeas','mu-rlnotes'])
        $(id).value = '';
  }catch(e){ $('mu-status').textContent = 'Save failed (link?)'; }
  muRefresh();
};
$('mu-stop').onclick = async ()=>{
  await fetch('/api/rl/stop', {method:'POST'});
  $('mu-status').textContent = 'Stopping (gait stops, robot holds; X to limp)…';
};
$('mu-refresh').onclick = ()=> muRefresh();

async function rlCheck(mode){
  $('rlpreflight').dataset.checked = '1';
  $('rlpreflight').textContent = 'Checking '+mode+'…';
  try{
    const r = await fetch('/api/rl/preflight?mode='+mode, {cache:'no-store'});
    const d = await r.json();
    const det = [];
    if(d.roll_deg!=null) det.push(`roll ${d.roll_deg}°`);
    if(d.pitch_deg!=null) det.push(`pitch ${d.pitch_deg}°`);
    if(d.max_pose_delta_deg!=null)
      det.push(`pose Δ ${d.max_pose_delta_deg}° (tol ${d.pose_tol_deg}°)`);
    $('rlpreflight').innerHTML = d.ok
      ? `<b style="color:#5fd08a">READY for ${mode}</b>`
        + (d.sim ? ' (sim — always ready)' : '') + ` · ${det.join(' · ')}`
      : `<b style="color:#ff7b72">NOT ready</b>: ${d.error||'?'}`
        + (det.length ? ` · ${det.join(' · ')}` : '');
  }catch(e){ $('rlpreflight').textContent = 'check failed (link?)'; }
}
$('rlcheckstand').onclick = ()=> rlCheck('stand');
$('rlchecklower').onclick = ()=> rlCheck('lower');
$('rlcheckwalk').onclick = ()=> rlCheck('walk');
// The Readiness checks guard REAL hardware (servo IDs, IMU, tilt, start
// pose); the MuJoCo sim passes them by construction. Say so instead of
// showing an empty box — but never clobber a result the operator asked
// for (dataset.checked). Re-painted when the backend target flips.
function rlPaintReadinessDefault(){
  const pf = $('rlpreflight');
  if(pf && pf.dataset.checked !== '1')
    pf.textContent = (targetHasSim && !targetHasRobot)
      ? 'SIM target — always READY. These checks (servo IDs, IMU, tilt, '
        + 'start pose) guard the real robot.'
      : '—';
}
async function refreshRlTab(){
  rlPaintReadinessDefault();
  try{
    const r = await fetch('/api/rl/policy', {cache:'no-store'});
    const d = await r.json();
    const w = d.walk || {};
    $('rlpolicyinfo').innerHTML = d.ok
      ? `stance: <b>${(d.source||'?').split('/').pop()}</b><br>`
        + `obs ${d.obs_dim} → [${(d.hidden||[]).join(', ')}] → `
        + `${d.act_dim} joints · ${d.activation}<br>`
        + (w.source
           ? `walk: <b>${w.source.split('/').pop()}</b><br>`
             + `obs ${w.obs_dim} → [${(w.hidden||[]).join(', ')}] → `
             + `${w.act_dim} joints · ${w.activation}`
           : `walk: ${w.error || 'not deployed'}`)
      : (d.error || 'no policy');
  }catch(e){ $('rlpolicyinfo').textContent = 'policy info unavailable'; }
  await rlLoadPicker();
  // Reconcile controls from live robot truth. This fixes stale browser state
  // after stand/lower completion, service restarts, or a missed poll tick.
  await refreshRlRuntimeState();
}

// ---- Policy picker (linux_control/policies/ registry) ---------------------
// Rendered as a table (name / description / role chips). Operator
// requests: 08-11 "the descriptions ARE the interface" (table, not
// dropdowns); 08-22 roles are picked HERE, per row — no separate
// "Model roles" dropdown section. One model can hold several roles.
// Walk/Stand chips load the live slot (the default every role follows);
// Sit/Hold chips set per-role overrides and toggle back to default.
let rlPolicies = [];
let rlRoles = {};        // role -> {file, resolved} from /api/rl/roles
let rlAllowedObs = {};   // role -> [allowed obs widths]
const RL_SLOT_TITLES = {stance: 'Stand / sit / hold', walk: 'Walk'};
const RL_ROLE_CHIPS = [  // [role, chip label, slot whose rows offer it]
  ['stand', 'Stand', 'stance'],
  ['lower', 'Sit',   'stance'],
  ['walk',  'Walk',  'walk'],
  ['hold',  'Hold',  null],          // any row with an allowed obs width
];
const RL_HOLD_ZERO = 'walk policy @ zero command';

function rlRoleHome(role){
  // Which picker row serves `role` right now: a row file name, 'zero'
  // (hold default = the walk policy's trained stop), or null. Trust
  // `resolved` over the raw assignment — each backend reports there
  // what actually RUNS (e.g. the sim ignores stale walk-role
  // overrides and drives the live slot).
  const cur = rlRoles[role] || {};
  const res = cur.resolved || '';
  if(role === 'hold' && res === RL_HOLD_ZERO) return 'zero';
  const hit = rlPolicies.find(p => p.file === res || p.name === res);
  if(hit) return hit.file;                 // sim: file; robot: meta name
  // Anything else (a "live … slot" label, or a dangling override the
  // backend ignored) means the live slot runs — light the slot row.
  const slot = (role === 'walk' || role === 'hold') ? 'walk' : 'stance';
  const act = rlPolicies.find(p => p.slot === slot && p.active);
  return act ? act.file : null;
}

function rlChipTitle(role, on, zero){
  if(zero)
    return 'Default hold: the walk policy holds at zero command (its '
      + 'trained stop). Click Hold on another model to override.';
  const what = {walk: 'walk when drive keys are held',
                hold: 'hold in place when no keys are held',
                stand: 'stand up (also the stance default for Sit/Hold)',
                lower: 'sit / lower'}[role];
  if(on)
    return (role === 'walk' || role === 'stand')
      ? `Current ${role === 'walk' ? 'walk' : 'stand-up'} model (the live `
        + `${role === 'walk' ? 'walk' : 'stance'} slot).`
      : `This model currently does "${what}" — click to reset the role `
        + 'to its default.';
  return `Use this model to ${what}. Applies at the next move/session.`;
}

async function rlLoadPicker(){
  const box = $('rlpicktable');
  try{
    const [pr, rr] = await Promise.all([
      fetch('/api/rl/policies', {cache:'no-store'}),
      fetch('/api/rl/roles', {cache:'no-store'})]);
    const d = await pr.json();
    if(!d.ok) throw new Error(d.error || 'list failed');
    rlPolicies = (d.policies || []).filter(p => !p.error);
    try{
      const dr = await rr.json();
      rlRoles = (dr.ok && dr.roles) || {};
      rlAllowedObs = (dr.ok && dr.allowed_obs) || {};
    }catch(e){ rlRoles = {}; rlAllowedObs = {}; }
    const home = {};
    for(const [role] of RL_ROLE_CHIPS) home[role] = rlRoleHome(role);
    box.innerHTML = '';
    for(const slot of ['stance','walk']){
      const items = rlPolicies.filter(p => p.slot === slot);
      const tbl = document.createElement('table');
      tbl.className = 'policies';
      const hdr = tbl.insertRow();
      const th = document.createElement('th');
      th.colSpan = 3;
      th.textContent = RL_SLOT_TITLES[slot] || slot;
      hdr.appendChild(th);
      if(!items.length){
        const c = tbl.insertRow().insertCell();
        c.colSpan = 3; c.textContent = '(none in policies/)';
      }
      for(const p of items){
        const tr = tbl.insertRow();
        if(p.active) tr.className = 'active';
        const name = tr.insertCell();
        name.className = 'polname';
        name.textContent = p.name;
        const notes = tr.insertCell();
        notes.className = 'polnotes';
        notes.textContent = p.notes || '—';
        const roles = tr.insertCell();
        roles.className = 'polroles';
        for(const [role, label, roleSlot] of RL_ROLE_CHIPS){
          if(roleSlot && roleSlot !== slot) continue;
          if(role === 'hold'){
            const dims = rlAllowedObs.hold || [68, 72, 74];
            if(!dims.includes(p.obs_dim)) continue;
            // Scripted gaits can't serve the RL hold role.
            if((p.file || '').startsWith('scripted:')) continue;
          }
          const zero = role === 'hold' && home.hold === 'zero'
            && slot === 'walk' && p.active;
          const on = zero || home[role] === p.file;
          const b = document.createElement('button');
          b.className = 'rolechip' + (on ? ' on' : '') + (zero ? ' zero' : '');
          b.textContent = zero ? 'Hold @0' : label;
          b.title = rlChipTitle(role, on, zero);
          b.onclick = ()=> rlChipClick(role, slot, p, on, zero);
          roles.appendChild(b);
        }
      }
      box.appendChild(tbl);
    }
  }catch(e){
    box.textContent = 'policy list unavailable (link?)';
  }
}

async function rlChipClick(role, slot, pick, on, zero){
  if(role === 'walk' || role === 'stand'){
    await rlSlotSelect(role, slot, pick);
  } else if(zero){
    $('rlpickmsg').textContent =
      'already the default hold (walk policy @ zero command)';
  } else {
    // Sit / Hold are per-role overrides; clicking the lit chip resets
    // the role to its default.
    const label = role === 'lower' ? 'Sit' : 'Hold';
    const file = on ? '' : pick.file;
    $('rlpickmsg').textContent = on
      ? `resetting ${label} to default…` : `setting ${label} → ${pick.name}…`;
    try{
      const r = await fetch('/api/rl/roles', {method:'POST',
        body: JSON.stringify({role, file})});
      const dd = await r.json();
      $('rlpickmsg').textContent = dd.ok
        ? (file ? `${label} → ${pick.name} ✔ (next move)`
                : `${label} reset to default ✔`)
        : `failed: ${dd.error || 'unknown'}`;
      if(!dd.ok) showErr('Role: '+(dd.error || 'failed'));
    }catch(e){ $('rlpickmsg').textContent = 'role set failed (link?)'; }
  }
  refreshRlTab();
}

async function rlSlotSelect(role, slot, pick){
  // Check the LIVE registry before swapping: a page loaded hours ago
  // has stale active flags, and blind-applying page state silently
  // reverted the stance policy once (08-11). Confirm swaps by name.
  $('rlpickmsg').textContent = 'checking current policy…';
  try{
    const live = (await (await fetch('/api/rl/policies',
      {cache:'no-store'})).json()).policies || [];
    const cur = live.find(q => q.slot === slot && q.active);
    if(!cur || cur.file !== pick.file){
      if(!confirm(`Swap ${RL_SLOT_TITLES[slot] || slot} policy:\n`
                  + `${cur ? cur.name : '(none)'}\n→ ${pick.name}?\n\n`
                  + 'Takes effect at the NEXT stand/lower/walk.')){
        $('rlpickmsg').textContent = `kept ${cur ? cur.name : '(none)'}`;
        return;
      }
      const r = await fetch('/api/rl/policy_select', {
        method:'POST', body: JSON.stringify({file: pick.file})});
      const d = await r.json();
      if(!d.ok){
        $('rlpickmsg').textContent = `swap failed: ${d.error || 'unknown'}`;
        showErr('Policy select: '+(d.error || 'failed'));
        return;
      }
      $('rlpickmsg').textContent = `${RL_SLOT_TITLES[slot] || slot} → `
        + `${d.name} ✔`;
    } else {
      $('rlpickmsg').textContent = `${pick.name} is already active`;
    }
    // Walk/Stand chips mean "the live slot drives this role": drop any
    // explicit override so the slot pick is what actually runs.
    const curRole = rlRoles[role] || {};
    if(curRole.file && curRole.file !== 'walk')
      await fetch('/api/rl/roles', {method:'POST',
        body: JSON.stringify({role, file: ''})});
  }catch(e){
    $('rlpickmsg').textContent = 'policy select failed (link?)';
  }
}
// --- Motors tab -------------------------------------------------------------
function startMotorsPoll(){
  stopMotorsPoll();
  motorsTimer = setInterval(()=>{ if(activeView==='motors') refreshMotors(); }, 1500);
}
function stopMotorsPoll(){ if(motorsTimer){ clearInterval(motorsTimer); motorsTimer=null; } }
async function refreshMotors(opts){
  const manual = !!(opts && opts.manual);
  const btn = $('mscan');
  if(manual){
    btn.disabled = true;
    btn.textContent = '↻ …';
    $('mrefresh').textContent = 'scanning…';
  }
  try{
    const r = await fetch('/api/status'); if(!r.ok) throw 0;
    const s = await r.json();
    setLink(true);
    $('mport').textContent = s.port || (s.dry_run ? 'dry-run' : '—');
    $('mmode').textContent = (s.armed?'ARMED':'limp')+' / '+s.mode;
    $('mmode').className = 'pill '+(s.armed?'ok':'bad');
    const nLive = (s.live_ids||[]).length;
    $('mlive').textContent = nLive;
    $('merr').textContent = s.error || '';
    const tb = $('mbody'); tb.innerHTML = '';
    (s.motors||[]).forEach(m=>{
      const tr = document.createElement('tr');
      if(m.alarm) tr.classList.add('alarm');
      if(m.joint!=null && m.joint===selJoint) tr.classList.add('sel');
      const flags = (m.status_bits||[]).join(',') || (m.ok?'':'ERR');
      tr.innerHTML = '<td>'+m.id+'</td><td>'+(m.name||'')+'</td><td>'+
        (m.ok?Number(m.deg).toFixed(1):'—')+'</td><td>'+
        (m.ok?Number(m.load_pct).toFixed(0)+'%':'—')+'</td><td>'+
        (m.ok?Number(m.current_a).toFixed(2):'—')+'</td><td>'+
        (m.ok?Number(m.volt).toFixed(1):'—')+'</td><td>'+
        (m.ok?m.temp_c:'—')+'</td><td>'+flags+'</td>';
      tr.onclick = ()=>{
        selJoint = (m.joint!=null)? m.joint : null;
        selSid = m.id;
        $('mseljoint').textContent = selJoint!=null ? (selJoint+' ('+m.name+')') : ('id '+m.id);
        $('mselinfo').textContent = m.ok
          ? ('ID '+m.id+'  '+m.deg+'°  load '+m.load_pct+'%  '+m.current_a+' A  '+m.volt+' V')
          : (m.error||'offline');
        refreshMotors();
      };
      tb.appendChild(tr);
    });
    if(!(s.motors||[]).length)
      tb.innerHTML = '<tr><td colspan="8">No servos answering'
        +(s.dry_run?' (dry-run)':' — check URT-2 / power')+'</td></tr>';
    if(s.demo) $('dstatus').textContent = s.demo.status+(s.demo.name?(' · '+s.demo.name):'');
    const now = new Date();
    const stamp = now.toLocaleTimeString();
    $('mrefresh').textContent = 'updated '+stamp+' · auto 1.5s';
    if(manual) showSent('refresh — '+nLive+' live');
  }catch(e){
    setLink(false, 'status failed');
    $('mrefresh').textContent = 'refresh failed';
    if(manual) showSent('refresh failed');
  }finally{
    if(manual){
      btn.disabled = false;
      btn.textContent = '↻ Refresh';
    }
  }
}
$('mscan').onclick = ()=> refreshMotors({manual:true});
$('mwamp').oninput = ()=> $('mwamplab').textContent = $('mwamp').value;
$('mwiggle').onclick = async ()=>{
  if(selJoint==null){ showSent('select a motor row first'); return; }
  const amp = +$('mwamp').value;
  showSent('wiggle j'+selJoint+'…');
  try{
    const r = await fetch('/api/wiggle',{method:'POST',
      headers:{'Content-Type':'application/json'},
      body: JSON.stringify({joint:selJoint, amp:amp})});
    const j = await r.json();
    if(j.ok){
      setArmed(true);
      showSent('wiggle j'+selJoint+' ±'+amp+'°');
    }else{
      showSent('wiggle failed: '+(j.error||'unknown'));
    }
  }catch(e){ showSent('wiggle failed'); }
};
$('mzero').onclick = async ()=>{
  // safe_zero arms itself; plans a collision-aware path and limps on stall.
  await goPoseZero('sit', 'go zero (safe)');
};
$('msetzero').onclick = ()=> setZeroHere(true);
$('mlimp').onclick = ()=>{ cmd('X'); setArmed(false); };
$('mpinned').onclick = async ()=>{
  // Read-only detector: tipped >=12° over a folded knee? (~1.5s when
  // tipped — it re-reads after a settle so a rock doesn't classify.)
  const el = $('mpinnedout');
  el.textContent = 'checking…';
  el.style.color = '';
  showSent('pinned-tip check…');
  try{
    const d = await (await fetch('/api/pinned_tip')).json();
    const s = d.pinned ? 'PINNED' : (d.tipped ? 'tipped (not pinned)' : 'level');
    el.textContent = s + (d.tilt_deg != null ? ` · tilt ${d.tilt_deg}°` : '')
      + (d.why ? ` — ${d.why}` : (d.error ? ` — ${d.error}` : ''));
    if(d.pinned) el.style.color = '#f66';
    showSent('pinned-tip: ' + s);
  }catch(e){
    el.textContent = 'check failed';
    showSent('pinned-tip check failed');
  }
};
$('muntrap').onclick = async ()=>{
  // Motion: low-torque (20%) fold. The server re-runs the detector and
  // refuses when not pinned; offer force=true only then, for bench tests.
  dancePaused = true;
  showSent('untrap…');
  const post = (force)=> fetch('/api/untrap',{method:'POST',
    headers:{'Content-Type':'application/json'},
    body: JSON.stringify(force ? {force:true} : {})});
  try{
    let j = await (await post(false)).json();
    if(!j.ok && /nothing to untrap/.test(j.error||'')){
      if(!confirm('Detector says: '+(j.error||'not pinned')+'\n\n'
                  +'Run the low-torque fold ANYWAY (bench test)? '
                  +'It will limp first, then fold hips+knees at 20% '
                  +'torque. Watch the robot.')){
        showSent('untrap: skipped (not pinned)');
        $('mpinnedout').textContent = j.error || 'not pinned';
        return;
      }
      j = await (await post(true)).json();
    }
    if(j.ok){
      showSent('untrap running — watch the fold');
    }else{
      showSent('untrap refused: '+(j.error||'unknown'));
      $('mpinnedout').textContent = j.error || 'untrap refused';
    }
  }catch(e){ showSent('untrap failed'); }
};

// --- Demos tab + global robot activity --------------------------------------
function demoSpeed(){ return Math.max(0.25, Math.min(3.0, (+$('dspeed').value)/100)); }
function demoDuration(){ return Math.max(5, Math.min(300, +($('ddur').value)||60)); }
function demoSize(){ return Math.max(0.5, Math.min(3.0, (+$('dsize').value)/100)); }
function demoRate(){ return Math.max(0.08, Math.min(0.60, (+$('drate').value)/100)); }
function demoSoft(){ return Math.max(0.5, Math.min(3.0, (+$('dsoft').value)/100)); }
function demoTorque(){ return Math.max(150, Math.min(1000, Math.round((+$('dtorque').value)*10))); }
// LIVE tempo: while a demo runs, dragging the slider retunes it in place
// (streamed demos react next tick; breathe at the next half-breath).
let liveSpeedTimer = null;
$('dspeed').oninput = ()=>{
  $('dspeedlab').textContent = demoSpeed().toFixed(2);
  if(!(lastDemo && lastDemo.running)) return;
  if(liveSpeedTimer) clearTimeout(liveSpeedTimer);
  liveSpeedTimer = setTimeout(async ()=>{
    liveSpeedTimer = null;
    try{
      const r = await fetch('/api/demo/speed',{method:'POST',
        headers:{'Content-Type':'application/json'},
        body: JSON.stringify({speed: demoSpeed()})});
      const j = await r.json();
      if(j.ok && j.running) showSent('live speed → '+j.speed.toFixed(2)+'×');
    }catch(e){}
  }, 150);
};
$('dsize').oninput = ()=> $('dsizelab').textContent = demoSize().toFixed(2);
$('drate').oninput = ()=> $('dratelab').textContent = demoRate().toFixed(2);
$('dsoft').oninput = ()=> $('dsoftlab').textContent = demoSoft().toFixed(2);
$('dtorque').oninput = ()=> $('dtorquelab').textContent = Math.round(demoTorque()/10);
let demoTimer = null;
let lastZero = null;   // {at_zero, max_err_deg, ...}
let lastDemo = null;   // last /api/robot demo payload
let demoPollN = 0;
function startDemoPoll(){
  stopDemoPoll();
  demoPollN = 0;
  // Status every 0.5s; zero probe every 2s (bus read — don't spam while demoing).
  demoTimer = setInterval(()=>{
    if(activeView!=='demos' && activeView!=='dance'
       && activeView!=='rock' && activeView!=='quad') return;
    demoPollN++;
    refreshRobotState(demoPollN % 4 === 0);
  }, 500);
}
function stopDemoPoll(){ if(demoTimer){ clearInterval(demoTimer); demoTimer=null; } }
function paintRobotActivity(robot){
  if(!robot) return;
  lastRobotState = robot;
  const el = $('robotact');
  if(!el) return;
  const act = robot.activity || 'idle';
  const detail = robot.detail || '';
  el.textContent = detail ? (act+' · '+detail) : act;
  const pitchEl = $('robotpitch');
  if(pitchEl) pitchEl.textContent = feedbackPitchLabel(lastFeedback) || '';
  el.className = 'act-'+act;
  el.title = 'activity='+act+(detail?(' — '+detail):'')
    +' · mode='+(robot.mode||'?')
    +(robot.armed?' · armed':' · limp');
}
function paintDemoStatus(d){
  if(!d) return;
  const running = !!d.running;
  let st = d.status || 'idle';
  const el = $('dstatus');
  el.textContent = st;
  if(running || st === 'done') el.className = 'pill ok';
  else if(String(st).startsWith('error') || st === 'aborted' || st === 'stopping')
    el.className = 'pill bad';
  else el.className = 'pill';
  // Mirror onto the Dance tab's status line (same payload, own ids).
  const del = $('dancestatus');
  if(del){ del.textContent = el.textContent; del.className = el.className; }
  const rel = $('rockstatus');
  if(rel){ rel.textContent = el.textContent; rel.className = el.className; }
  const detail = $('dstatusdetail');
  if(detail){
    const bits = [];
    if(d.name) bits.push(d.name);
    const p = d.params || {};
    if(p.size!=null) bits.push('size '+Number(p.size).toFixed(2)+'×');
    if(p.rate!=null) bits.push(Number(p.rate).toFixed(2)+'Hz');
    if(p.softness!=null) bits.push('soft '+Number(p.softness).toFixed(2)+'×');
    if(p.torque!=null) bits.push('τ'+p.torque);
    if(p.balance_trim) bits.push('trim');
    if(p.log) bits.push(p.log);
    if(running) bits.push('running');
    else if(String(st).startsWith('done')) bits.push('finished');
    else if(st === 'aborted') bits.push('stopped');
    detail.textContent = bits.length ? (' · '+bits.join(' · ')) : '';
    const ddet = $('dancestatusdetail');
    if(ddet) ddet.textContent = detail.textContent;
    const rdet = $('rockstatusdetail');
    if(rdet) rdet.textContent = detail.textContent;
  }
  const telemEl = $('dtelem');
  if(telemEl){
    const t = d.telemetry;
    if(t && t.rows && t.rows.length){
      const c = t.counts || {};
      const noisy = (t.noisiest || []).slice(0,3).join(', ') || '—';
      telemEl.innerHTML =
        `Last run: <b>${t.log_name||'log'}</b> · `+
        `<span class="cal-pill green">${c.green||0}g</span> `+
        `<span class="cal-pill yellow">${c.yellow||0}y</span> `+
        `<span class="cal-pill red">${c.red||0}r</span>`+
        ` · noisiest: ${noisy}`;
    } else if(t && t.log_name){
      telemEl.textContent = 'Last run log: '+t.log_name;
    } else if(!running){
      telemEl.innerHTML =
        'Standard event log includes low-rate command telemetry. Enable Full motion CSV for cmd vs encoder logs.';
    }
  }
  // Quad tab mirrors the same demo state with its own pill.
  const qel = $('qstatus');
  if(qel){
    qel.textContent = el.textContent;
    qel.className = el.className;
    const qd = $('qstatusdetail'), dd = $('dstatusdetail');
    if(qd && dd) qd.textContent = dd.textContent;
  }
}
function paintZeroHint(z){
  const el = $('zerohint');
  if(!el) return;
  if(!z){
    el.textContent = 'Click a demo anytime — it homes to sit/stand as needed, then runs.';
    el.className = 'hint';
    return;
  }
  if(z.busy){
    el.textContent = 'Zero check skipped — robot busy.';
    el.className = 'hint';
    return;
  }
  if(z.error && z.max_err_deg==null){
    el.textContent = 'Zero check failed: '+z.error;
    el.className = 'hint bad';
    return;
  }
  const err = (z.max_err_deg!=null) ? Number(z.max_err_deg).toFixed(1) : '?';
  if(z.at_zero){
    el.textContent = 'At sit zero (max |θ|='+err+'° ≤ '+z.tol_deg+'°).';
    el.className = 'hint ok';
  } else {
    el.textContent = 'Not at sit zero (max |θ|='+err+'° > '+z.tol_deg
      +'°) — air demos will home to sit first.';
    el.className = 'hint';
  }
}
async function refreshRobotState(wantZero){
  try{
    const q = wantZero ? '?zero=1' : '';
    const r = await fetch('/api/robot'+q, {cache:'no-store'});
    if(!r.ok) throw 0;
    const j = await r.json();
    paintRobotActivity(j);
    if(j.demo){ lastDemo = j.demo; paintDemoStatus(j.demo); }
    if(j.zero){ lastZero = j.zero; paintZeroHint(j.zero); }
  }catch(e){ /* heartbeat covers link loss */ }
}
async function refreshDemoStatus(){ return refreshRobotState(activeView==='demos'); }

const DEMO_GROUPS = [
  ['air',   'In the air (sitting)', 'homes to sit zero · legs out'],
  ['stand', 'Standing dances — streamed, live speed', 'homes via the 10× stand-up · planted-foot body IK — feet stay glued'],
  ['plant', 'Planted acts & shows', 'scripted glides · own timing'],
  ['walk',  'Walk', 'open-loop tripod gait — floor + slack cord'],
];
function demoGroupOf(item){
  if(item.group) return item.group;
  if(item.air) return 'air';
  return item.name.startsWith('walk') ? 'walk' : 'plant';
}
async function loadDemos(){
  try{
    const r = await fetch('/api/demos'); const d = await r.json();
    const g = $('dgrid'); g.innerHTML='';
    const items = d.demos||[];
    DEMO_GROUPS.forEach(([key, title, sub])=>{
      const inGroup = items.filter(it=>demoGroupOf(it)===key);
      if(!inGroup.length) return;
      const h = document.createElement('div');
      h.className = 'demo-group';
      h.innerHTML = title+' <span class="sub">· '+sub+'</span>';
      g.appendChild(h);
      inGroup.forEach(item=> g.appendChild(demoButton(item)));
    });
  }catch(e){ $('dgrid').innerHTML = '<div class="hint">Failed to load demos</div>'; }
}
function demoButton(item){
      const b = document.createElement('button');
      b.dataset.name = item.name;
      b.innerHTML = '<b>'+item.name+'</b>'
        +(item.live_speed?' <span style="color:#5fd08a;font-size:10px;font-weight:700">LIVE</span>':'')
        +'<br><span style="color:#9aa3b2;font-weight:400;font-size:12px">'
        +item.title+'</span>';
      b.onclick = async ()=>{
        if(!(await ensureDemoTarget(item))) return;
        if(needArm()) return;
        const sp = demoSpeed();
        const body = {name:item.name, speed:sp, torque:demoTorque(),
                      seconds:demoDuration()};
        if(demoMotionLog()) body.motion_log = true;
        if(item.name==='breathe' || item.name==='breathe_v' || item.has_size){
          body.size = demoSize();
          body.rate = demoRate();
          body.softness = demoSoft();
        }
        showSent('demo '+item.name+' request sent…');
        const res = await fetch('/api/demo',{method:'POST',
          headers:{'Content-Type':'application/json'},
          body: JSON.stringify(body)});
        const j = await res.json();
        if(j.ok){
          const p = j.params || body;
          let msg = '';
          if(j.switched) msg += 'switch←'+(j.switched_from||'?')+' · ';
          msg += 'demo '+item.name+' @ '+sp.toFixed(2)+'×';
          if(j.home) msg += ' (via '+j.home+' zero)';
          if(p.size!=null) msg += ' size '+Number(p.size).toFixed(2)+'×';
          if(p.rate!=null) msg += ' '+Number(p.rate).toFixed(2)+'Hz';
          if(p.softness!=null) msg += ' soft '+Number(p.softness).toFixed(2)+'×';
          if(p.torque!=null) msg += ' τ'+p.torque;
          showSent(requestReceiptLine(j, msg));
        } else {
          showSent(requestReceiptLine(j, 'demo '+item.name)+'; failed: '
            +(j.error||'unknown'), true);
          if(j.zero){ lastZero = j.zero; paintZeroHint(j.zero); }
        }
        if(j.demo) paintDemoStatus(j.demo);
        if(j.robot) paintRobotActivity(j.robot);
        startDemoPoll();
        refreshRobotState(true);
      };
      return b;
}
$('dstop').onclick = async ()=>{
  showSent('stop request sent…');
  const r = await fetch('/api/demo/stop',{method:'POST'});
  try{
    const j = await r.json();
    showSent(requestReceiptLine(j, 'Stop'));
    if(j.demo) paintDemoStatus(j.demo);
    if(j.robot) paintRobotActivity(j.robot);
  }catch(e){}
  showSent('demo stop');
  refreshRobotState(true);
};
// --- Dance tab: curated show list, reusing the demo machinery ----------------
const DANCE_SETS = [
  ['SITTING SHOWS', 'chassis stays down — safe on a desk',
   ['dance_swarm', 'air_trident', 'air_weave', 'air_gearbox', 'air_tides',
    'air_meet', 'air_pendulum', 'air_orbits']],
  ['FULL SHOWS', 'stands up mid-routine — clear floor space',
   ['dance_swarm_up', 'dance_swarm_encore', 'dance_encore', 'dance_wild',
    'dance_swarm_stand', 'dance_steeple', 'dance', 'dance_walk',
    'rise_show']],
];
async function loadDance(){
  try{
    const r = await fetch('/api/demos'); const d = await r.json();
    const byName = {};
    (d.demos||[]).forEach(it=>{ byName[it.name] = it; });
    const g = $('dancegrid'); g.innerHTML='';
    let shown = 0;
    DANCE_SETS.forEach(([title, sub, names])=>{
      const items = names.map(n=>byName[n]).filter(Boolean);
      if(!items.length) return;
      shown += items.length;
      const h = document.createElement('div');
      h.className = 'demo-group';
      h.innerHTML = title+' <span class="sub">· '+sub+'</span>';
      g.appendChild(h);
      items.forEach(item=> g.appendChild(demoButton(item)));
    });
    // Uploaded dance scripts (dances-as-data via POST /api/dances).
    const up = (d.demos||[]).filter(it=> it.group === 'uploaded');
    if(up.length){
      shown += up.length;
      const h = document.createElement('div');
      h.className = 'demo-group';
      h.innerHTML = 'UPLOADED <span class="sub">· dance scripts sent '
        +'over the API — survive code deploys</span>';
      g.appendChild(h);
      up.forEach(item=> g.appendChild(demoButton(item)));
    }
    if(!shown){
      const src = d.sources && d.sources.robot;
      const why = src && src.error ? ' ('+src.error+')' : '';
      g.innerHTML = '<div class="hint">No robot dance demos loaded'
        +why+'. Connect the robot target or switch to Robot.</div>';
    }
  }catch(e){ $('dancegrid').innerHTML = '<div class="hint">Failed to load shows</div>'; }
}
$('dancestop').onclick = ()=> $('dstop').onclick();
$('dancespeed').oninput = ()=>{
  $('dspeed').value = $('dancespeed').value;   // one shared speed setting
  $('dancespeedlab').textContent = demoSpeed().toFixed(2);
  $('dspeed').oninput();                       // live-push if a show runs
};

// --- Rock tab: the rocking-chair seesaw, reusing the demo machinery ---------
// Own speed slider (NOT the shared demo speed): the rock's tempo window is
// sim-verified at 0.75x-1.5x — outside it the presses miss the floor (fast)
// or the rock gets violent (slow).
function rockSpeed(){ return Math.max(0.75, Math.min(1.5, (+$('rockspeed').value)/100)); }
let rockSpeedTimer = null;
$('rockspeed').oninput = ()=>{
  $('rockspeedlab').textContent = rockSpeed().toFixed(2);
  if(!(lastDemo && lastDemo.running && lastDemo.name === 'rock')) return;
  if(rockSpeedTimer) clearTimeout(rockSpeedTimer);
  rockSpeedTimer = setTimeout(async ()=>{
    rockSpeedTimer = null;
    try{
      const r = await fetch('/api/demo/speed',{method:'POST',
        headers:{'Content-Type':'application/json'},
        body: JSON.stringify({speed: rockSpeed()})});
      const j = await r.json();
      if(j.ok && j.running) showSent('live speed → '+j.speed.toFixed(2)+'×');
    }catch(e){}
  }, 150);
};
$('rockstart').onclick = async ()=>{
  let item = {name:'rock'};
  try{   // hub mode annotates items with their target (robot vs sim)
    const r = await fetch('/api/demos'); const d = await r.json();
    item = (d.demos||[]).find(it=> it.name==='rock') || item;
  }catch(e){}
  if(!(await ensureDemoTarget(item))) return;
  if(needArm()) return;
  const sp = rockSpeed();
  const body = {name:'rock', speed:sp,
                seconds:Math.max(5, Math.min(300, +($('rockdur').value)||60))};
  showSent('demo rock request sent…');
  const res = await fetch('/api/demo',{method:'POST',
    headers:{'Content-Type':'application/json'},
    body: JSON.stringify(body)});
  const j = await res.json();
  if(j.ok){
    let msg = 'rock @ '+sp.toFixed(2)+'×';
    if(j.switched) msg = 'switch←'+(j.switched_from||'?')+' · '+msg;
    if(j.home) msg += ' (via '+j.home+' zero)';
    showSent(requestReceiptLine(j, msg));
  } else {
    showSent(requestReceiptLine(j, 'rock')+'; failed: '
      +(j.error||'unknown'), true);
    if(j.zero){ lastZero = j.zero; paintZeroHint(j.zero); }
  }
  if(j.demo) paintDemoStatus(j.demo);
  if(j.robot) paintRobotActivity(j.robot);
  startDemoPoll();
  refreshRobotState(true);
};
$('rockstop').onclick = ()=> $('dstop').onclick();

$('dzero').onclick = ()=> goPoseZero('sit', 'sit zero');
$('dstand').onclick = ()=> goPoseZero('stand', 'stand zero');

// --- Quad tab (tip-back four-leg walk) --------------------------------------
function quadSpeed(){ return Math.max(0.25, Math.min(2.0, (+$('qspeed').value)/100)); }
function quadSuffix(){
  const v = $('qstance') ? $('qstance').value : '';
  if(v === 'safe') return '_safe';
  if(v === 'pitch') return '_pitch';
  if(v === 'aft') return '_aft';
  if(v === 'high') return '_high';
  if(v === 'step') return '_step';
  if(v === 'aggressive') return '_aggressive';
  return '';
}
function quadVariantLabel(){
  const v = $('qstance') ? $('qstance').value : '';
  if(v === 'safe') return 'safe';
  if(v === 'pitch') return 'pitched';
  if(v === 'aft') return 'aft-shift';
  if(v === 'high') return 'high-body';
  if(v === 'step') return 'high-step';
  if(v === 'aggressive') return 'aggressive';
  return 'cool';
}
function quadName(action){ return 'quad_'+action+quadSuffix(); }
function isQuadDown(name){ return name.indexOf('quad_down') === 0; }
function quadRobotBlocked(action){
  return targetHasRobot && quadSuffix() === '_aggressive'
    && ['walk', 'walk_back', 'trot', 'trot_back'].includes(action);
}
function quadRunAction(action, label){
  if(quadRobotBlocked(action)){
    showSent(
      'aggressive walk/trot is blocked on the robot after the forward fall; '
      +'use Safe/Cool for hardware or switch to MuJoCo-only to simulate it',
      true);
    return;
  }
  quadRun(quadName(action), label);
}
$('qspeed').oninput = ()=>{
  $('qspeedlab').textContent = quadSpeed().toFixed(2);
  if(!(lastDemo && lastDemo.running)) return;
  if(liveSpeedTimer) clearTimeout(liveSpeedTimer);
  liveSpeedTimer = setTimeout(async ()=>{
    liveSpeedTimer = null;
    try{
      const r = await fetch('/api/demo/speed',{method:'POST',
        headers:{'Content-Type':'application/json'},
        body: JSON.stringify({speed: quadSpeed()})});
      const j = await r.json();
      if(j.demo) paintDemoStatus(j.demo);
    }catch(e){}
  }, 150);
};
async function quadRun(name, label){
  if(needArm()) return;
  const sp = quadSpeed();
  const timeout = Math.max(30, Math.min(300, +($('qdur').value)||300));
  const body = {name, speed:sp};
  if(quadMotionLog()) body.motion_log = true;
  if(!isQuadDown(name)) body.seconds = timeout;
  showSent(label+' request sent…');
  const res = await fetch('/api/demo',{method:'POST',
    headers:{'Content-Type':'application/json'},
    body: JSON.stringify(body)});
  const j = await res.json();
  if(j.ok){
    let msg = label+' @ '+sp.toFixed(2)+'×';
    if(isQuadDown(name)) msg = label;
    else msg += ' · hold limit '+body.seconds+'s';
    const home = responseHomeLabel(j);
    if(home) msg += ' (via '+home+')';
    showSent(requestReceiptLine(j, msg));
  }
  else showSent(requestReceiptLine(j, label)+'; failed: '
    +(j.error||'unknown'), true);
  if(j.demo) paintDemoStatus(j.demo);
  if(j.robot) paintRobotActivity(j.robot);
  startDemoPoll();
  refreshRobotState(true);
}
$('qrear').onclick = ()=> quadRunAction(
  'rear', 'quad '+quadVariantLabel()+' rear up');
$('qfwd').onclick = ()=> quadRunAction(
  'walk', 'quad '+quadVariantLabel()+' walk forward');
$('qback').onclick = ()=> quadRunAction(
  'walk_back', 'quad '+quadVariantLabel()+' walk backward');
$('qtrot').onclick = ()=> quadRunAction(
  'trot', 'quad '+quadVariantLabel()+' trot forward');
$('qtrotback').onclick = ()=> quadRunAction(
  'trot_back', 'quad '+quadVariantLabel()+' trot backward');
$('qdown').onclick = ()=> quadRunAction(
  'down', 'quad '+quadVariantLabel()+' come down');
$('qstop').onclick = ()=> quadRunAction(
  'hold', 'quad '+quadVariantLabel()+' settle hold');
$('qstand').onclick = ()=> goPoseZero('stand', 'stand zero');
$('dcheckz').onclick = async ()=>{
  showSent('checking zero…');
  await refreshRobotState(true);
  if(lastZero && lastZero.at_zero) showSent('at zero');
  else if(lastZero && lastZero.max_err_deg!=null)
    showSent('not at zero ('+Number(lastZero.max_err_deg).toFixed(1)+'°)');
  else showSent('zero check done');
};
// Keep a light global activity poll even off the demos tab.
setInterval(()=>{ if(activeView!=='demos') refreshRobotState(false); }, 2000);
refreshRobotState(false);
startTelem();   // Drive is the default view; showView() stops it on tab change

// deep links — every tab is reachable via #hash (preferred) or the
// path-style routes the server also serves (/motors, /rl, …).
$('subhost').textContent = location.host || 'hexapod.local:8080';
(function(){
  const path = location.pathname.replace(/\/+$/,'');
  const hash = (location.hash||'').replace(/^#/,'');
  let init = 'drive';
  if(VIEWS.includes(hash)) init = hash;
  else { for(const v of VIEWS){ if(path.endsWith('/'+v)){ init = v; break; } } }
  if(init !== 'drive') showView(init);
})();
window.addEventListener('hashchange', ()=>{
  const v = (location.hash||'').replace(/^#/,'');
  if(VIEWS.includes(v) && v !== activeView) showView(v);
});

// --- SERVO ARM / DISARM gate ------------------------------------------------
// The firmware boots DISARMED (all PCA9685 channels forced full-off = no PWM,
// so every servo is limp). This page also defaults to disarmed on EVERY load
// and NEVER auto-arms. "Enable servos" sends ARM; the big red EMERGENCY STOP,
// the drive STOP, and Debug's Relax all send DISARM (firmware `X`), which cuts
// all PWM. needArm() gates every servo-driving send on both pages.
function updateArmUI(){
  const bar = $('armbar');
  const zeroBtn = $('armzero');
  const simOnly = targetHasSim && !targetHasRobot;
  const robotConfigured = robotTargetAvailable || !!robotTargetUrl;
  bar.classList.toggle('sim', simOnly);
  if(simOnly){
    bar.classList.remove('armed', 'disarmed');
    $('armstate').textContent = '● SIM';
    $('armbtn').textContent = 'Stand';
    $('armbtn').title = 'Reset the MuJoCo sim to the standing pose.';
    zeroBtn.disabled = true;
    zeroBtn.title = robotConfigured
      ? 'Switch Robot active to run safe zero on the real robot.'
      : 'Connect the robot to run safe zero.';
    $('estop').textContent = robotConfigured ? '■ E-STOP' : '■ Stop';
    $('estop').title = robotConfigured
      ? 'Global emergency stop: cut robot servo power immediately and stop '
        +'MuJoCo, even while the active target is sim.'
      : 'Stop the sim motion — the stance policy holds.';
    return;
  }
  bar.classList.toggle('armed', servosArmed);
  bar.classList.toggle('disarmed', !servosArmed);
  // Compact header labels (operator 08-19); the long explanation lives in
  // the button tooltips instead of a hint paragraph. Labels + titles are
  // restored here because sim mode rewrites all of them.
  $('armstate').textContent = servosArmed ? '● ON' : '● OFF';
  $('armbtn').textContent   = servosArmed ? 'Disarm' : 'Enable';
  $('armbtn').title = servosArmed
    ? 'Normal power-off (SETTLE): lowers gently to the ground, THEN cuts '
      +'servo power. For an instant cut use EMERGENCY STOP (robot drops).'
    : 'Power the servos on (ARM). Nothing moves until you press Stand.';
  zeroBtn.disabled = !robotTargetAvailable;
  zeroBtn.title = robotTargetAvailable
    ? 'Move to logical sit zero: STEP lower if standing; safe-zero '
      +'recovery if not standing or tangled.'
    : 'Robot target is not connected.';
  $('estop').textContent = '■ E-STOP';
  $('estop').title = 'Cut all power to the servos IMMEDIATELY — the robot '
    +'goes limp NOW and will drop. Use only in an emergency. For a normal, '
    +'gentle power-off use Disarm / Sit & power off.';
}
function setArmed(on){ servosArmed = on; if(!on) armed = false; updateArmUI(); }
function armServos(){
  if(targetHasSim && !targetHasRobot){
    simPost('/api/sim/reset', {start:'plant'});
    showSent('SIM — reset to stand');
    return;
  }
  cmd('ARM'); setArmed(true);
  showSent('ARM — servos enabled (nothing moves; press Stand to stand)');
}
// GRACEFUL power-off: lower to the ground first (firmware SETTLE = SIT then
// DISARM), only THEN cut power. This is the NORMAL disarm/relax/off path so the
// robot settles instead of collapsing. UI shows disarmed once the command is
// sent (the firmware does the lower, then goes limp on its own).
function settleServos(){ dbgTestAbort = true; cmd('SETTLE'); setArmed(false);
  showSent('DISARM — lowering gently, then servos off'); }
async function topSafeZero(){
  if(targetHasSim && !targetHasRobot){
    showSent('switch Robot active before STEP lower', true);
    return;
  }
  if(!robotTargetAvailable){
    showSent('robot target not connected', true);
    return;
  }
  await goPoseZero('sit', 'STEP lower');
}
// INSTANT limp: cut all PWM NOW (true emergency stop; the robot drops). Always
// allowed, even while disarmed, and used for the boot-time safe default.
function disarmServos(){
  dbgTestAbort = true; cmd('X', {globalStop:true}); setArmed(false);
  showSent(targetHasSim && !targetHasRobot
    && !(robotTargetAvailable || !!robotTargetUrl)
    ? 'SIM — stopped, stance policy holds'
    : 'EMERGENCY STOP — servos limp NOW');
}
// Returns true (and warns) when disarmed; every servo-driving action calls it.
function needArm(){
  if(targetHasSim && !targetHasRobot) return false;
  if(servosArmed) return false;
  showSent('⚠ Servos disarmed — press Enable first');
  return true;
}
// The Disarm toggle is a NORMAL power-off -> graceful lower then limp.
$('armbtn').onclick = ()=> servosArmed ? settleServos() : armServos();
// Header Zero uses the same smart lower: STEP-down from standing,
// safe-zero recovery from tangled/not-standing poses.
$('armzero').onclick = topSafeZero;
// EMERGENCY STOP is the ONLY instant-limp control (cuts PWM immediately).
$('estop').onclick  = disarmServos;
// Enforce the safe default on EVERY page load: show disarmed AND tell the
// firmware to disarm now — harmless if it just booted disarmed, and it clears
// any stale ARMED state from a prior session so the page's OFF state is real.
setArmed(false);
cmd('X');
