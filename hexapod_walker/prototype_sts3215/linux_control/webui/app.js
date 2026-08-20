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

// --- link heartbeat --------------------------------------------------------
function setLink(ok, detail){
  linkOk = !!ok;
  document.body.classList.toggle('link-down', !ok);
  if(ok){
    linkFailStreak = 0;
    lastPingOkAt = Date.now();
    conn.textContent = detail || 'connected';
    conn.className = 'ok';
  } else {
    conn.textContent = detail || 'offline';
    conn.className = 'bad';
    const bar = document.getElementById('offlinebar');
    const target = backendKind === 'sim' ? 'MuJoCo sim' : 'Uno Q';
    if(bar) bar.textContent = `● Lost connection to ${target} — retrying…`
      +(detail ? (' ('+detail+')') : '');
  }
}
function applyBackendMeta(meta){
  if(!meta) return;
  const kind = meta.kind || (meta.service === 'hexapod-sim' ? 'sim' : 'robot');
  const frames = meta.frames !== false;
  const nativeViewer = !!meta.viewer;
  const changed = kind !== backendKind || frames !== simFrames
    || nativeViewer !== simNativeViewer;
  backendKind = kind;
  simFrames = frames;
  simNativeViewer = nativeViewer;
  document.body.classList.toggle('sim-backend', backendKind === 'sim');
  document.body.classList.toggle('sim-native-viewer',
    backendKind === 'sim' && simNativeViewer);
  document.body.classList.toggle('sim-browser-frames',
    backendKind === 'sim' && simFrames);
  if(!simFrames){
    const img = document.getElementById('simframe');
    if(img) img.removeAttribute('src');
    simFrameBusy = false;
  }
  if(changed){
    updateArmUI();
    if(activeView === 'rl') simPollMaybe();
  }
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
    if(j && j.ok === false) throw new Error(j.error || 'ping failed');
    applyBackendMeta(j);
    setLink(true, backendKind === 'sim' ? 'sim connected' : undefined);
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
async function cmd(line){
  try {
    const r = await fetch('/cmd', {method:'POST', body:line});
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
function showSent(line, isErr){
  sentEl.textContent = line;
  if(isErr ||
     /refus|fail|error|not ready|missing|timeout|no bus|unknown|denied|abort/i
       .test(String(line)))
    showErr(line);
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
function padBtnDown(gp, i){
  const btn = gp.buttons[i];
  if(!btn) return false;
  // Triggers report .value; .pressed is fine once past the browser threshold.
  return !!(btn.pressed || (typeof btn.value === 'number' && btn.value > 0.35));
}
async function padRunDemo(name, label){
  if(needArm()) return;
  dancePaused = true;
  showSent('pad '+label+' → '+name);
  try{
    const body = {name:name, speed:demoSpeed(), torque:demoTorque()};
    if(name==='breathe' || name==='breathe_v'){
      body.size = demoSize(); body.rate = demoRate(); body.softness = demoSoft();
    }
    const res = await fetch('/api/demo',{method:'POST',
      headers:{'Content-Type':'application/json'},
      body: JSON.stringify(body)});
    const j = await res.json();
    if(j.ok) showSent('demo '+name+(j.home?(' via '+j.home):''));
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
  // SIT uses /api/safe_zero (collision-aware staged plan; LIMPS on any
  // stall / unexpected-force feedback). STAND keeps the verified glide.
  dancePaused = true;
  if(pose === 'stand') armed = false;
  const tag = label || pose;
  const safe = pose !== 'stand';
  showSent(tag + '…');
  try{
    let r = await fetch(safe ? '/api/safe_zero' : '/api/zero',{method:'POST',
      headers:{'Content-Type':'application/json'},
      body: JSON.stringify(safe ? {} : {pose: pose})});
    if(safe && r.status === 404){
      // Older server without safe_zero — legacy glide fallback.
      r = await fetch('/api/zero',{method:'POST',
        headers:{'Content-Type':'application/json'},
        body: JSON.stringify({pose: pose})});
    }
    const j = await r.json();
    if(!j.ok){
      showSent(tag + ' failed: '+(j.error||'unknown'));
      if(j.demo) paintDemoStatus(j.demo);
      if(j.robot) paintRobotActivity(j.robot);
      return;
    }
    setArmed(true);
    if(safe && j.plan && j.plan.stages)
      showSent(tag + ' — safe plan: '+j.plan.stages.length+' stage(s), ~'
        +(j.plan.total_s||'?')+'s (limps on stall)');
    else
      showSent(tag + ' — gliding (full torque)');
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
    if(j.ok) showSent('zero-here OK — '+(j.ok_n||'?')+'/'+(j.count||'?')+' (limp)');
    else showSent('zero-here '+(j.error||'failed'));
  }catch(e){ showSent('zero-here failed'); }
}
async function padStopDemo(){
  showSent('pad B → stop demo');
  try{
    const r = await fetch('/api/demo/stop',{method:'POST'});
    const j = await r.json();
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

// --- Drive bench workflow ----------------------------------------------------
// Mirrors rl_move/scripts/tape_measure_walk.py: the operator limps, hand-poses,
// POST /api/set_zero, ARMs, Stands (P glide), preflights, then the gait gets
// plain `J vx vy omega` and `J 0 0 0` over /cmd — nothing else.
document.getElementById('wlimp').onclick = ()=>{
  dbgTestAbort = true; cmd('X'); setArmed(false);
  showSent('limp — torque off; hand-pose legs, then Set zero HERE');
};
async function setZeroHere(fromMotors){
  // No confirm (operator 08-11: no warning modals). Motors do not
  // move — only the zero point is rewritten.
  showSent('set-here-as-zero…');
  try{
    const r = await fetch('/api/set_zero',{method:'POST'});
    const j = await r.json();
    setArmed(false);
    if(j.ok) showSent('zero-here OK — '+j.ok_n+'/'+j.count+' (limp)');
    else showSent('zero-here '+(j.error || ((j.ok_n||0)+'/'+(j.count||0)+' — check Motors table')));
  }catch(e){ showSent('zero-here failed'); }
  if(fromMotors) refreshMotors();
}
document.getElementById('wsetzero').onclick = ()=> setZeroHere(false);
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

// --- gait picker: tripod (0) vs no-slip (1) + overlap alpha ------------------
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
    const fmt = (v,d)=> (v==null || isNaN(v)) ? '—' : Number(v).toFixed(d);
    if(fb && fb.ok){
      $('tmroll').textContent = fmt(fb.roll_deg, 1);
      $('tmpitch').textContent = fmt(fb.pitch_deg, 1);
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
const VIEWS = ['drive','motors','demos','dance','quad','rl','experiments',
               'measure','calibrate','debug'];
const TAB_TITLES = {drive:'Drive', motors:'Motors', demos:'Demos',
                    dance:'Dance', quad:'Quad', rl:'RL',
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
  else stopDemoPoll();
  if(which === 'calibrate'){
    if(servosArmed){ cmd('HOLD'); forceResend(); }
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
$('tab-quad').onclick = ()=> showView('quad');
$('tab-rl').onclick = ()=> showView('rl');
$('tab-experiments').onclick = ()=> showView('experiments');
$('tab-measure').onclick = ()=> showView('measure');
$('tab-calibrate').onclick = ()=> showView('calibrate');
$('tab-debug').onclick = ()=> showView('debug');
dbgRefresh();

// --- Calibrate (step + shake/hold + plant height + CSV log) -----------------
let calMode = 'step';
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
    : '(default +20 / +80)';
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
function syncCalModeUI(){
  const shake = calMode === 'shake';
  const plant = calMode === 'plant';
  const geometry = calMode === 'geometry';
  const imu = calMode === 'imu';
  $('calhint-step').style.display = (!shake && !plant && !geometry && !imu) ? '' : 'none';
  $('calhint-shake').style.display = shake ? '' : 'none';
  $('calhint-plant').style.display = plant ? '' : 'none';
  const geoEl = $('calhint-geometry');
  if(geoEl) geoEl.style.display = geometry ? '' : 'none';
  $('calhint-imu').style.display = imu ? '' : 'none';
  $('calstepwrap').style.display = (!shake && !plant && !geometry && !imu) ? '' : 'none';
  $('calnudgewrap').style.display = shake ? '' : 'none';
  $('calaxiswrap').style.display = (plant || geometry || imu) ? 'none' : '';
  $('calplantinfo').style.display = (plant || geometry) ? '' : 'none';
  $('calplantreset').style.display = (plant || geometry) ? '' : 'none';
  $('calimuinfo').style.display = imu ? '' : 'none';
  $('calimureset').style.display = imu ? '' : 'none';
  if(imu){
    $('callegend').innerHTML =
      '<span class="cal-pill green">green</span> still · saved · '+
      '<span class="cal-pill yellow">yellow</span> mild shake · saved · '+
      '<span class="cal-pill red">red</span> too much motion · not saved';
  } else if(plant || geometry){
    $('callegend').innerHTML =
      '<span class="cal-pill green">saved</span> contact → new stand home · '+
      '<span class="cal-pill yellow">no contact</span> not saved · '+
      '<span class="cal-pill red">abort</span> stopped';
  } else if(shake){
    $('callegend').innerHTML =
      '<span class="cal-pill green">green</span> quiet hold · '+
      '<span class="cal-pill yellow">yellow</span> mild shake · '+
      '<span class="cal-pill red">red</span> hunt / never settled';
  } else {
    $('callegend').innerHTML =
      '<span class="cal-pill green">green</span> tracked · '+
      '<span class="cal-pill yellow">yellow</span> partial / high load · '+
      '<span class="cal-pill red">red</span> barely moved';
  }
}
function renderCalResult(res){
  const body = $('calbody');
  const head = $('calhead');
  if(!res){
    if(body && !($('calstatus').textContent||'').includes('…'))
      body.innerHTML = '<tr><td colspan="8">No rows yet.</td></tr>';
    return;
  }
  if(res.mode === 'imu'){
    const g = res.grade || (res.saved ? 'green' : 'red');
    $('calcounts').innerHTML =
      (res.saved
        ? `<span class="cal-pill green">saved</span> IMU ${g}`
        : `<span class="cal-pill ${g==='red'?'red':'yellow'}">not saved</span> IMU ${g}`) +
      ` · |g|=${res.accel_mag_g!=null?Number(res.accel_mag_g).toFixed(3):'—'} · `+
      `ω ptp ${res.gyro_ptp_dps!=null?Number(res.gyro_ptp_dps).toFixed(1):'—'} dps`+
      (res.hint ? `<div class="hint" style="margin-top:6px">${res.hint}</div>` : '');
    $('callog').textContent = res.log_name
      ? `Log: logs/${res.log_name} (${res.samples||0} samples)`
      : (res.log ? `Log: ${res.log}` : 'Log: —');
    head.innerHTML = '<tr><th>Vector</th><th>X</th><th>Y</th><th>Z</th>'+
      '<th>Unit</th><th></th><th></th><th></th></tr>';
    const rows = res.rows || [];
    if(rows.length){
      body.innerHTML = rows.map(r=>
        `<tr class="g-${g}"><td>${r.axis}</td>`+
        `<td>${Number(r.x).toFixed(4)}</td>`+
        `<td>${Number(r.y).toFixed(4)}</td>`+
        `<td>${Number(r.z).toFixed(4)}</td>`+
        `<td>${r.unit||''}</td><td colspan="3"></td></tr>`
      ).join('');
    } else {
      body.innerHTML = '<tr><td colspan="8">No IMU vectors.</td></tr>';
    }
    if(res.imu) paintImuInfo(res.imu);
    return;
  }
  if(res.mode === 'plant' || res.mode === 'geometry_plant'){
    const saved = !!(res.saved || (res.ok && res.joints_deg));
    const g = saved ? 'green' : 'yellow';
    $('calcounts').innerHTML =
      (saved
        ? `<span class="cal-pill green">saved</span> hip ${res.hip_deg}° / knee ${res.knee_deg}°`
        : `<span class="cal-pill yellow">not saved</span> `+
          (res.contact_found?'':'no contact · ')+
          `hip ${res.hip_deg!=null?res.hip_deg:'—'}° / knee ${res.knee_deg!=null?res.knee_deg:'—'}`) +
      (res.clearance_mm!=null ? ` · +${res.clearance_mm}mm` : '') +
      (res.msg ? `<div class="hint" style="margin-top:6px">${res.msg}</div>` : '') +
      (res.hint ? `<div class="hint" style="margin-top:6px">${res.hint}</div>` : '') +
      (res.error ? `<div class="hint" style="margin-top:6px;color:#f88">${res.error}</div>` : '');
    $('callog').textContent = res.path
      ? `Plant: ${res.path}`
      : (res.log_name
        ? `Log: logs/${res.log_name} (${res.samples||0} samples)`
        : (res.log ? `Log: ${res.log}` : 'Log: —'));
    head.innerHTML = '<tr><th>Leg</th><th>Hip°</th><th>Knee°</th><th></th>'+
      '<th></th><th></th><th></th><th></th></tr>';
    const legs = res.per_leg || [];
    if(legs.length){
      body.innerHTML = legs.map(r=>
        `<tr class="g-${g}"><td>L${r.leg}</td><td>${r.hip_deg}</td>`+
        `<td>${r.knee_deg}</td><td colspan="5"></td></tr>`
      ).join('');
    } else if(res.joints_deg && res.joints_deg.length===18){
      let rows='';
      for(let leg=0;leg<6;leg++){
        const h=res.joints_deg[leg*3+1], k=res.joints_deg[leg*3+2];
        rows += `<tr class="g-${g}"><td>L${leg}</td><td>${Number(h).toFixed(1)}</td>`+
          `<td>${Number(k).toFixed(1)}</td><td colspan="5"></td></tr>`;
      }
      body.innerHTML = rows;
    } else {
      body.innerHTML = '<tr><td colspan="8">No per-leg snapshot.</td></tr>';
    }
    if(res.plant) paintPlantInfo(res.plant);
    else if(saved) paintPlantInfo(res);
    return;
  }
  if(!res.rows || !res.rows.length){
    if(body && !($('calstatus').textContent||'').includes('…'))
      body.innerHTML = '<tr><td colspan="8">No rows yet.</td></tr>';
    return;
  }
  const shake = (res.mode || 'step') === 'shake';
  const c = res.counts || {};
  const size = shake
    ? `nudge ${res.nudge_deg!=null?res.nudge_deg:res.step_deg}°`
    : `step ${res.step_deg}°`;
  $('calcounts').innerHTML =
    `<span class="cal-pill green">${c.green||0} green</span> `+
    `<span class="cal-pill yellow">${c.yellow||0} yellow</span> `+
    `<span class="cal-pill red">${c.red||0} red</span>`+
    ` · ${res.joints_tested||0} joints · ${shake?'shake':'step'} · ${size}`+
    (res.hint ? `<div class="hint" style="margin-top:6px">${res.hint}</div>` : '');
  $('callog').textContent = res.log_name
    ? `Log: logs/${res.log_name} (${res.samples||0} samples)`
    : (res.log ? `Log: ${res.log}` : 'Log: —');
  if(shake){
    head.innerHTML = '<tr><th>Joint</th><th>Name</th><th>Nudge</th>'+
      '<th>Overshoot</th><th>Hold pp°</th><th>RMS err</th>'+
      '<th>Load pp</th><th>Grade</th></tr>';
    body.innerHTML = res.rows.map(r=>{
      const g = r.grade || 'red';
      return `<tr class="g-${g}"><td>${r.joint}</td><td>${r.name}</td>`+
        `<td>${r.nudge_deg!=null?r.nudge_deg:r.delta_cmd_deg}</td>`+
        `<td>${r.overshoot_deg!=null?r.overshoot_deg:'—'}</td>`+
        `<td>${r.hold_pp_deg!=null?r.hold_pp_deg:'—'}</td>`+
        `<td>${r.hold_rms_err_deg!=null?r.hold_rms_err_deg:'—'}</td>`+
        `<td>${r.hold_load_pp!=null?r.hold_load_pp+'%':'—'}</td>`+
        `<td><span class="cal-pill ${g}">${g}</span></td></tr>`;
    }).join('');
  } else {
    head.innerHTML = '<tr><th>Joint</th><th>Name</th><th>Δcmd</th><th>Δact</th>'+
      '<th>Track%</th><th>Load</th><th>Vmin</th><th>Grade</th></tr>';
    body.innerHTML = res.rows.map(r=>{
      const g = r.grade || 'red';
      return `<tr class="g-${g}"><td>${r.joint}</td><td>${r.name}</td>`+
        `<td>${r.delta_cmd_deg}</td><td>${r.delta_actual_deg}</td>`+
        `<td>${r.tracking_pct}</td><td>${r.peak_load_pct}%</td>`+
        `<td>${r.min_volt==null?'—':r.min_volt}</td>`+
        `<td><span class="cal-pill ${g}">${g}</span></td></tr>`;
    }).join('');
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
    if(d.running){
      $('calstatus').textContent = (p.msg || 'Running…') +
        (p.index!=null ? ` (${(p.index|0)+1}/${p.total||'?'})` : '');
      $('calrun').disabled = true;
    } else {
      $('calrun').disabled = false;
      if(d.result && d.result.ok){
        $('calstatus').textContent = d.result.aborted ? 'Aborted.' : 'Done.';
        renderCalResult(d.result);
      } else if(d.result && d.result.error){
        $('calstatus').textContent = 'Error: '+d.result.error;
      } else if(!($('calstatus').textContent||'').match(/Done|Error|Aborted/)){
        // keep last status
      }
    }
    if(d.result && (d.result.rows || d.result.mode==='plant'
        || d.result.mode==='geometry_plant' || d.result.mode==='imu'))
      renderCalResult(d.result);
  }catch(e){
    $('calstatus').textContent = 'Calibrate status failed (link?)';
  }
}
$('calstep').oninput = ()=>{ $('calsteplab').textContent = $('calstep').value; };
$('calnudge').oninput = ()=>{ $('calnudgelab').textContent = Number($('calnudge').value).toFixed(1); };
document.querySelectorAll('#calmode button').forEach(b=>{
  b.onclick = ()=>{
    document.querySelectorAll('#calmode button').forEach(x=>x.classList.remove('on'));
    b.classList.add('on');
    calMode = b.dataset.mode || 'step';
    syncCalModeUI();
    if(calMode==='plant' || calMode==='geometry' || calMode==='imu') refreshCalibrate();
  };
});
document.querySelectorAll('#calaxis button').forEach(b=>{
  b.onclick = ()=>{
    document.querySelectorAll('#calaxis button').forEach(x=>x.classList.remove('on'));
    b.classList.add('on');
    calAxis = b.dataset.axis || 'all';
  };
});
syncCalModeUI();
$('calrun').onclick = async ()=>{
  if(calMode !== 'imu' && needArm()) return;
  $('calstatus').textContent = 'Starting…';
  $('calrun').disabled = true;
  try{
    const r = await fetch('/api/calibrate', {
      method:'POST',
      headers:{'Content-Type':'application/json'},
      body: JSON.stringify({
        mode: calMode,
        step_deg: parseFloat($('calstep').value)||10,
        nudge_deg: parseFloat($('calnudge').value)||2,
        axis: calAxis,
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
        const res = d.result || {};
        $('rlstatus').textContent = res.ok
          ? `Done · max ${res.max_current_a ?? '?'} A`
             + (res.limped ? ' · limped' : ' · holding')
          : (res.error || 'done');
      }
    }catch(e){ /* keep polling */ }
  }, 500);
}
function rlButtons(disabled){
  for(const id of ['rlstand','rllower','rlglide','rlcapture','rlwalkfwd',
                   'rlwalkleft','rlwalkright','rlwalkback',
                   'rlwalkfl','rlwalkfr','rlwalkbl','rlwalkbr',
                   'rldrivestart'])
    $(id).disabled = disabled;
}
async function rlMove(mode, body){
  // No confirms anywhere (operator 08-11: no warning modals);
  // the server preflight refuses bad start poses.
  if(mode!=='stand' && mode!=='lower' && body)
    delete body.heading;   // UI-only label, not an API field
  $('rlstatus').textContent = 'Preflight…';
  rlButtons(true);
  try{
    const r = await fetch('/api/rl/'+mode, {method:'POST',
      body: body ? JSON.stringify(body) : undefined});
    const d = await r.json();
    if(!d.ok){
      $('rlstatus').textContent = 'Refused: '+(d.error || 'unknown');
      showErr('RL: '+(d.error || 'refused'));
      rlButtons(false);
      return;
    }
    $('rlstatus').textContent = 'Running…';
    startRlPoll();
  }catch(e){
    $('rlstatus').textContent = 'Start failed (link?)';
    rlButtons(false);
  }
}
$('rlstand').onclick = ()=> rlMove('stand');
$('rllower').onclick = ()=> rlMove('lower');
$('rlglide').onclick = async ()=>{
  $('rlstatus').textContent = 'Scripted → plant stance…';
  try{
    const r = await fetch('/api/standup', {method:'POST',
      body: JSON.stringify({mode:'plant', speed:10})});
    const d = await r.json();
    if(!d.ok){
      $('rlstatus').textContent = 'Refused: '+(d.error || 'unknown');
      showErr('Plant stance: '+(d.error || 'refused'));
      return;
    }
    $('rlstatus').textContent = 'Moving to plant stance…';
    startDemoPoll();
  }catch(e){ $('rlstatus').textContent = 'Start failed (link?)'; }
};
$('rlcapture').onclick = async ()=>{
  $('rlstatus').textContent = 'Capturing plant pose (no motion)…';
  try{
    const r = await fetch('/api/rl/capture_plant', {method:'POST'});
    const d = await r.json();
    $('rlstatus').textContent = d.ok
      ? `Plant captured: hip ${d.hip_deg}° knee ${d.knee_deg}° — `
        + 'Lower & Walk now start from this pose.'
      : 'Capture failed: '+(d.error || 'unknown');
  }catch(e){ $('rlstatus').textContent = 'Capture failed (link?)'; }
};
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
// The browser streams (vx, vy) heartbeats at 5 Hz while the session is
// active; the robot's 25 Hz loop slews toward them and treats anything
// older than 0.6 s as "keys released". So: keydown = walk, keyup = stop
// and hold, dead tab = stop and hold.
let drvActive = false, drvHb = null;
const drvKeys = new Set();
let drvPad = null;   // on-screen pad vector [dx, dy] while held
const DRV_KEYMAP = {
  arrowup:'fwd', w:'fwd', i:'fwd',
  arrowdown:'back', s:'back', k:'back',
  arrowleft:'left', a:'left', j:'left',
  arrowright:'right', d:'right', l:'right',
};
function drvVec(){
  let dx = (drvKeys.has('fwd')?1:0) - (drvKeys.has('back')?1:0);
  let dy = (drvKeys.has('left')?1:0) - (drvKeys.has('right')?1:0);
  if(!dx && !dy && drvPad){ dx = drvPad[0]; dy = drvPad[1]; }
  const n = Math.hypot(dx, dy);
  const s = parseFloat($('rlwalkspeed').value);
  return n ? [dx/n*s, dy/n*s] : [0, 0];
}
function drvPaint(d){
  const live = (d && d.live) || {};
  const bits = [];
  if(live.model) bits.push(`model <b>${live.model}</b>`);
  if(live.vx_ref!=null)
    bits.push(`v (${Math.round(live.vx_ref*1000)}, `
              + `${Math.round(live.vy_ref*1000)}) mm/s`);
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
  const [vx, vy] = drvVec();
  try{
    const r = await fetch('/api/rl/drive/cmd', {method:'POST',
      body: JSON.stringify({vx, vy})});
    const d = await r.json();
    if(!d.active){ drvEnded(); return; }
    drvPaint(d);
  }catch(e){ /* link blip — watchdog on the robot handles it */ }
}
async function drvEnded(){
  drvActive = false;
  if(drvHb){ clearInterval(drvHb); drvHb = null; }
  drvKeys.clear(); drvPad = null;
  $('rldrivestart').disabled = false;
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
  $('rldrivestart').disabled = true;
  $('rldrivestatus').textContent = 'Starting session (preflight'
    + ' — may acquire the stand first)…';
  try{
    const r = await fetch('/api/rl/drive/start', {method:'POST'});
    const d = await r.json();
    if(!d.ok){
      $('rldrivestatus').textContent = 'Refused: '+(d.error || 'unknown');
      showErr('Drive: '+(d.error || 'refused'));
      $('rldrivestart').disabled = false;
      return;
    }
    drvActive = true;
    drvKeys.clear(); drvPad = null;
    if(drvHb) clearInterval(drvHb);
    drvHb = setInterval(drvSend, 200);
    drvPaint(d);
  }catch(e){
    $('rldrivestatus').textContent = 'Start failed (link?)';
    $('rldrivestart').disabled = false;
  }
};
$('rldriveend').onclick = async ()=>{
  $('rldrivestatus').textContent = 'Ending session (rolls to a stop, holds)…';
  try{ await fetch('/api/rl/drive/stop', {method:'POST'}); }catch(e){}
  // Heartbeats keep flowing until the server reports inactive, so the
  // decel + wind-down is visible in the status line.
};
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
    if(!drvActive) return; drvPad = dv; drvSend(); };
  const up = ()=>{ if(drvPad === dv){ drvPad = null; drvSend(); } };
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
  if(backendKind !== 'sim' || activeView !== 'rl'){ stopSimPoll(); return; }
  if(!simTimer) simTimer = setInterval(refreshSimPanel, 500);
  refreshSimPanel();
}
async function refreshSimPanel(){
  if(backendKind !== 'sim' || activeView !== 'rl' || simBusy) return;
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
if($('simresetstand'))
  $('simresetstand').onclick = ()=> simPost('/api/sim/reset',
    {start:'plant'});
if($('simresetbelly'))
  $('simresetbelly').onclick = ()=> simPost('/api/sim/reset',
    {start:'belly'});
if($('simfall')) $('simfall').onclick = ()=> simPost('/api/sim/fall');
if($('simrecover')) $('simrecover').onclick =
  ()=> simPost('/api/sim/recover');
if($('simpush')) $('simpush').onclick = ()=> simPost('/api/sim/push',
  {x:4, y:0});

// ---- Model roles (which policy file serves each function) ------------------
const RL_ROLE_DEFS = [
  ['walk',  'Walk (keys held)'],
  ['hold',  'Hold (no keys)'],
  ['stand', 'Stand up'],
  ['lower', 'Sit / lower'],
];
async function rlRolesRefresh(){
  const box = $('rlroles');
  try{
    const d = await (await fetch('/api/rl/roles', {cache:'no-store'})).json();
    if(!d.ok) throw new Error(d.error || 'roles failed');
    const allowed = d.allowed_obs || {};
    box.innerHTML = '';
    for(const [role, label] of RL_ROLE_DEFS){
      const cur = (d.roles[role] || {});
      const row = document.createElement('label');
      row.className = 'hint';
      row.style.cssText = 'display:flex;gap:8px;align-items:center;'
        + 'margin-top:4px';
      const span = document.createElement('span');
      span.style.cssText = 'min-width:120px;display:inline-block';
      span.innerHTML = `<b>${label}</b>`;
      const sel = document.createElement('select');
      const def = document.createElement('option');
      def.value = '';
      def.textContent = role === 'hold'
        ? 'walk policy @ zero command (default)'
        : `live ${role === 'walk' ? 'walk' : 'stance'} slot (default)`;
      sel.appendChild(def);
      const dims = allowed[role] || [];
      for(const p of rlPolicies){
        if(!dims.includes(p.obs_dim)) continue;
        const o = document.createElement('option');
        o.value = p.file;
        o.textContent = `${p.name} (obs ${p.obs_dim})`;
        sel.appendChild(o);
      }
      sel.value = (cur.file && cur.file !== 'walk') ? cur.file : '';
      sel.onchange = async ()=>{
        $('rlrolesmsg').textContent = `setting ${role}…`;
        try{
          const r = await fetch('/api/rl/roles', {method:'POST',
            body: JSON.stringify({role, file: sel.value})});
          const dd = await r.json();
          $('rlrolesmsg').textContent = dd.ok
            ? `${label} → ${(dd.roles[role]||{}).resolved} ✔ `
              + '(next session/move)'
            : `failed: ${dd.error || 'unknown'}`;
          if(!dd.ok) showErr('Role: '+(dd.error || 'failed'));
        }catch(e){ $('rlrolesmsg').textContent = 'role set failed (link?)'; }
        rlRolesRefresh();
      };
      const now = document.createElement('span');
      now.style.color = '#8089a0';
      now.textContent = cur.resolved ? `→ ${cur.resolved}` : '';
      row.appendChild(span); row.appendChild(sel); row.appendChild(now);
      box.appendChild(row);
    }
  }catch(e){ box.textContent = 'roles unavailable (link?)'; }
}

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
function suPoll(){
  if(suTimer) clearInterval(suTimer);
  suTimer = setInterval(async ()=>{
    try{
      const r = await fetch('/api/calibrate?t='+Date.now(), {cache:'no-store'});
      const d = await r.json();
      if(d.running && (d.name||'').startsWith('standup_')){
        $('sulab-status').textContent = (d.progress||{}).msg || 'running…';
      } else if(!d.running){
        clearInterval(suTimer); suTimer = null;
        $('sulab-go').disabled = false; $('sulab-sit').disabled = false;
        const res = d.result || {};
        $('sulab-status').textContent = res.ok
          ? `Done · ${res.mode} ${res.direction==='down'?'sit':'stand'} · `
            + `peak ${res.peak_a ?? '?'} A — holding (X to limp)`
          : (res.error || 'stopped — holding (X to limp)');
      }
    }catch(e){ /* keep polling */ }
  }, 500);
}
async function suRun(direction){
  if(!suSel) return;
  // No confirm dialog (operator request 08-10): the server refuses a
  // bad start pose and that lands in the status line + error bar.
  $('sulab-go').disabled = true; $('sulab-sit').disabled = true;
  $('sulab-status').textContent = 'Starting…';
  try{
    const r = await fetch('/api/standup', {method:'POST',
      body: JSON.stringify({mode: suSel, direction,
                            speed: parseFloat($('sulab-speed').value)})});
    const d = await r.json();
    if(!d.ok){
      $('sulab-status').textContent = 'Refused: '+(d.error || 'unknown');
      showErr('Stand-up lab: '+(d.error || 'refused'));
      $('sulab-go').disabled = false; $('sulab-sit').disabled = false;
      return;
    }
    $('sulab-status').textContent = 'Running…';
    suPoll();
  }catch(e){
    $('sulab-status').textContent = 'Start failed (link?)';
    $('sulab-go').disabled = false; $('sulab-sit').disabled = false;
  }
}
$('sulab-go').onclick = ()=> suRun('up');
$('sulab-sit').onclick = ()=> suRun('down');
$('sulab-stop').onclick = async ()=>{
  await fetch('/api/standup/stop', {method:'POST'});
  $('sulab-status').textContent = 'Stopping (holds pose; X to limp)…';
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
      ? `<b style="color:#5fd08a">READY for ${mode}</b> · ${det.join(' · ')}`
      : `<b style="color:#ff7b72">NOT ready</b>: ${d.error||'?'}`
        + (det.length ? ` · ${det.join(' · ')}` : '');
  }catch(e){ $('rlpreflight').textContent = 'check failed (link?)'; }
}
$('rlcheckstand').onclick = ()=> rlCheck('stand');
$('rlchecklower').onclick = ()=> rlCheck('lower');
$('rlcheckwalk').onclick = ()=> rlCheck('walk');
async function refreshRlTab(){
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
  await rlRolesRefresh();
  try{
    const r = await fetch('/api/calibrate?t='+Date.now(), {cache:'no-store'});
    const d = await r.json();
    if(d.running && (d.name||'').startsWith('rl_policy')){
      $('rlstatus').textContent = (d.progress||{}).msg || 'running…';
      rlButtons(true);
      startRlPoll();
    }
  }catch(e){}
  // Reconnect to a drive session that survived a page reload: resume
  // heartbeats (the robot has been holding since ours went stale).
  try{
    const d = await (await fetch('/api/rl/drive', {cache:'no-store'})).json();
    if(d.active && !drvActive){
      drvActive = true;
      $('rldrivestart').disabled = true;
      if(drvHb) clearInterval(drvHb);
      drvHb = setInterval(drvSend, 200);
      drvPaint(d);
    }
  }catch(e){}
}

// ---- Policy picker (linux_control/policies/ registry) ---------------------
// Rendered as a table (name / description / Use) instead of dropdowns —
// operator request 08-11: the descriptions ARE the interface.
let rlPolicies = [];
const RL_SLOT_TITLES = {stance: 'Stand / sit / hold', walk: 'Walk'};
async function rlLoadPicker(){
  const box = $('rlpicktable');
  try{
    const r = await fetch('/api/rl/policies', {cache:'no-store'});
    const d = await r.json();
    if(!d.ok) throw new Error(d.error || 'list failed');
    rlPolicies = (d.policies || []).filter(p => !p.error);
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
        if(p.active){
          const pill = document.createElement('span');
          pill.className = 'pill ok';
          pill.textContent = 'ACTIVE';
          pill.style.marginLeft = '6px';
          name.appendChild(pill);
        }
        const notes = tr.insertCell();
        notes.className = 'polnotes';
        notes.textContent = p.notes || '—';
        const use = tr.insertCell();
        use.className = 'poluse';
        if(!p.active){
          const b = document.createElement('button');
          b.textContent = 'Use';
          b.onclick = ()=> rlPickUse(slot, p);
          use.appendChild(b);
        }
      }
      box.appendChild(tbl);
    }
  }catch(e){
    box.textContent = 'policy list unavailable (link?)';
  }
}
async function rlPickUse(slot, pick){
  // Check the LIVE registry before swapping: a page loaded hours ago
  // has stale active flags, and blind-applying page state silently
  // reverted the stance policy once (08-11). Confirm swaps by name.
  $('rlpickmsg').textContent = 'checking current policy…';
  try{
    const live = (await (await fetch('/api/rl/policies',
      {cache:'no-store'})).json()).policies || [];
    const cur = live.find(p => p.slot === slot && p.active);
    if(cur && cur.file === pick.file){
      $('rlpickmsg').textContent = `${pick.name} is already active`;
      refreshRlTab();
      return;
    }
    if(!confirm(`Swap ${RL_SLOT_TITLES[slot] || slot} policy:\n`
                + `${cur ? cur.name : '(none)'}\n→ ${pick.name}?\n\n`
                + 'Takes effect at the NEXT stand/lower/walk.')){
      $('rlpickmsg').textContent = `kept ${cur ? cur.name : '(none)'}`;
      return;
    }
    const r = await fetch('/api/rl/policy_select', {
      method:'POST', body: JSON.stringify({file: pick.file})});
    const d = await r.json();
    $('rlpickmsg').textContent = d.ok
      ? `${RL_SLOT_TITLES[slot] || slot} → ${d.name} ✔`
      : `swap failed: ${d.error || 'unknown'}`;
    if(!d.ok) showErr('Policy select: '+(d.error || 'failed'));
  }catch(e){
    $('rlpickmsg').textContent = 'policy select failed (link?)';
  }
  refreshRlTab();
}
$('calplantreset').onclick = async ()=>{
  try{
    const r = await fetch('/api/plant/reset', {method:'POST'});
    const d = await r.json();
    if(d.ok){ paintPlantInfo(d); showSent('plant reset to default'); }
    else showSent(d.error || 'reset failed');
    refreshCalibrate();
  }catch(e){ showSent('plant reset failed'); }
};
$('calimureset').onclick = async ()=>{
  try{
    const r = await fetch('/api/imu/reset', {method:'POST'});
    const d = await r.json();
    if(d.ok){ paintImuInfo(d); showSent('IMU calib cleared'); }
    else showSent(d.error || 'reset failed');
    refreshCalibrate();
  }catch(e){ showSent('IMU reset failed'); }
};

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
       && activeView!=='quad') return;
    demoPollN++;
    refreshRobotState(demoPollN % 4 === 0);
  }, 500);
}
function stopDemoPoll(){ if(demoTimer){ clearInterval(demoTimer); demoTimer=null; } }
function paintRobotActivity(robot){
  if(!robot) return;
  const el = $('robotact');
  if(!el) return;
  const act = robot.activity || 'idle';
  const detail = robot.detail || '';
  el.textContent = detail ? (act+' · '+detail) : act;
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
  const detail = $('dstatusdetail');
  if(detail){
    const bits = [];
    if(d.name) bits.push(d.name);
    const p = d.params || {};
    if(p.size!=null) bits.push('size '+Number(p.size).toFixed(2)+'×');
    if(p.rate!=null) bits.push(Number(p.rate).toFixed(2)+'Hz');
    if(p.softness!=null) bits.push('soft '+Number(p.softness).toFixed(2)+'×');
    if(p.torque!=null) bits.push('τ'+p.torque);
    if(p.log) bits.push(p.log);
    if(running) bits.push('running');
    else if(String(st).startsWith('done')) bits.push('finished');
    else if(st === 'aborted') bits.push('stopped');
    detail.textContent = bits.length ? (' · '+bits.join(' · ')) : '';
    const ddet = $('dancestatusdetail');
    if(ddet) ddet.textContent = detail.textContent;
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
        'Demos auto-log cmd vs encoder → <code>logs/demo_*.csv</code> (+ summary).';
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

// --- demo motion schematics ------------------------------------------------
// Stylised pose fns per demo, rendered by the 3D view below when idle /
// hovering (while a demo RUNS the 3D view shows live encoders instead).
const DEMO_PREVIEW = {
  breathe: { start:'sit', blurb:'From sit zero (legs out): all hips/knees gently flex in/out together like a breath.',
    pose:(t,L)=>{ const b=0.5*(1-Math.cos(t*Math.PI*2*0.35));
      for(let i=0;i<6;i++){ L[i].yaw=0; L[i].hip=-0.35*b; L[i].knee=0.45*b; } } },
  breathe_v: { start:'sit', blurb:'Same as breathe, but velocity/speed-hold (no position hunt). Still starts at sit zero.',
    pose:(t,L)=>DEMO_PREVIEW.breathe.pose(t,L) },
  heartbeat: { start:'sit', blurb:'Sit zero → double knee thump pulse (lub-dub), then rest.',
    pose:(t,L)=>{ const c=(t%1.1)/1.1; let k=0;
      if(c<0.12) k=Math.sin(Math.PI*c/0.12); else if(c>0.18&&c<0.30) k=0.7*Math.sin(Math.PI*(c-0.18)/0.12);
      for(let i=0;i<6;i++){ L[i].yaw=0; L[i].hip=-0.2*k; L[i].knee=0.55*k; } } },
  twinkle: { start:'sit', blurb:'Sit zero → small independent wiggles on each joint (alive fidget).',
    pose:(t,L)=>{ for(let i=0;i<6;i++){ L[i].yaw=0.25*Math.sin(t*2.1+i); L[i].hip=0.15*Math.sin(t*1.7+i*1.3);
      L[i].knee=0.2*Math.sin(t*2.4+i*0.7); } } },
  shimmy: { start:'sit', blurb:'Sit zero → odd/even legs yaw opposite ways (shimmy), hips/knees stay out.',
    pose:(t,L)=>{ const a=0.45*Math.sin(t*Math.PI*2*0.55);
      for(let i=0;i<6;i++){ L[i].yaw=(i%2? -a:a); L[i].hip=0; L[i].knee=0; } } },
  ripple: { start:'sit', blurb:'Sit zero → yaw wave travels around the hex (phase per leg).',
    pose:(t,L)=>{ for(let i=0;i<6;i++){ L[i].yaw=0.5*Math.sin(t*Math.PI*2*0.45 - i*Math.PI/3);
      L[i].hip=0; L[i].knee=0; } } },
  conductor: { start:'sit', blurb:'Sit zero → one leg waves as “pointer”; neighbors hold; pointer walks around.',
    pose:(t,L)=>{ const p=Math.floor((t*0.4)%6);
      for(let i=0;i<6;i++){ if(i===p){ L[i].yaw=0.55*Math.sin(t*4); L[i].hip=-0.25*Math.abs(Math.sin(t*4)); L[i].knee=0.15; }
        else { L[i].yaw=(Math.abs(i-p)%6===1||Math.abs(i-p)%6===5)? -0.15:0; L[i].hip=0; L[i].knee=0; } } } },
  arms_up: { start:'sit', blurb:'Sit zero → fold all six legs way overhead, hold, back to sit zero.',
    pose:(t,L)=>{ const c=t%4; let u=0;
      if(c<1.2) u=c/1.2; else if(c<2.6) u=1; else u=Math.max(0,1-(c-2.6)/1.2);
      for(let i=0;i<6;i++){ L[i].yaw=0; L[i].hip=-1.05*u; L[i].knee=0.4*u; } } },
  stand_sway: { start:'stand', blurb:'Standing (planted-foot IK, LIVE speed): the BODY leans side to side; all six feet stay glued to the floor.',
    pose:(t,L)=>{ const s=Math.sin(t*1.9);
      for(let i=0;i<6;i++){ const a=(i+0.5)*Math.PI/3;
        L[i].yaw=0; L[i].hip=-0.55-0.12*s*Math.sin(a); L[i].knee=0.9+0.08*s*Math.sin(a); } } },
  stand_hula: { start:'stand', blurb:'Standing (planted-foot IK, LIVE speed): the body circles like a hula hoop — the old robot\u2019s wiggle. Feet never move.',
    pose:(t,L)=>{ for(let i=0;i<6;i++){ const a=(i+0.5)*Math.PI/3, c=Math.cos(t*3.1-a);
        L[i].yaw=0; L[i].hip=-0.55-0.12*c; L[i].knee=0.9+0.08*c; } } },
  stand_bounce: { start:'stand', blurb:'Standing (planted-foot IK, LIVE speed): pogo — the body dips straight down and back up, feet planted.',
    pose:(t,L)=>{ const c=0.5*(1-Math.cos(t*5.6));
      for(let i=0;i<6;i++){ L[i].yaw=0; L[i].hip=-0.55+0.16*c; L[i].knee=0.9+0.12*c; } } },
  stand_twist: { start:'stand', blurb:'Standing (planted-foot IK, LIVE speed): twist & dip — the body yaws while the feet stay planted (IK counter-rotates them). Cord-safe.',
    pose:(t,L)=>{ const y=0.2*Math.sin(t*2.8), d=0.5*(1-Math.cos(t*5.6));
      for(let i=0;i<6;i++){ L[i].yaw=y; L[i].hip=-0.55+0.08*d; L[i].knee=0.9+0.06*d; } } },
  stand_wave: { start:'stand', blurb:'Standing (planted-foot IK, LIVE speed): stadium wave — a narrow raised-leg crest circles the body, everyone else stays planted.',
    pose:(t,L)=>{ for(let i=0;i<6;i++){ const a=(i+0.5)*Math.PI/3;
        const e=Math.pow(0.5*(1+Math.cos(t*2.6-a)),3);
        L[i].yaw=0; L[i].hip=-0.55+e*0.03; L[i].knee=0.9-e*0.38; } } },
  stand_march: { start:'stand', blurb:'Standing (planted-foot IK, LIVE speed): tripod march in place — alternating tripods step straight up while the weight sways; never fewer than three feet down.',
    pose:(t,L)=>{ const ph=t*3.9;
      for(let i=0;i<6;i++){ const c=(i%2===0)?0:Math.PI;
        let d=((ph-c)%(2*Math.PI)+2*Math.PI)%(2*Math.PI); if(d>Math.PI)d=2*Math.PI-d;
        const p=(d<0.42*Math.PI)?0.5*(1+Math.cos(d/(0.42*Math.PI)*Math.PI)):0;
        L[i].yaw=0; L[i].hip=-0.55-0.22*p; L[i].knee=0.9+0.1*p; } } },
  stand_hi: { start:'stand', blurb:'Standing (planted-foot IK, LIVE speed): weight eases back and dips, then ONE front paw lifts and waves hello — five feet stay down.',
    pose:(t,L)=>{ const sh=Math.min(t/0.8,1), ar=Math.max(0,Math.min((t-0.7)/1,1));
      for(let i=0;i<6;i++){ const paw=(i===0);
        const w=Math.sin(t*5);
        L[i].yaw=paw?0.21*ar*w:0;
        L[i].hip=-0.55-0.05*sh+(paw?ar*(-0.35):0.0);
        L[i].knee=0.9+(paw?ar*(-0.55+0.2*w):0.05*sh); } } },
  stand_hands: { start:'stand', blurb:'Stand zero → lift three legs (0,2,4) way overhead; other three stay planted; back to stand.',
    pose:(t,L)=>{ const c=t%4; let u=0;
      if(c<1.1) u=c/1.1; else if(c<2.7) u=1; else u=Math.max(0,1-(c-2.7)/1.1);
      for(let i=0;i<6;i++){ const up=(i%2===0)?1:0;
        L[i].yaw=0; L[i].hip=-0.55+0.75*u*up; L[i].knee=0.9-0.6*u*up; } } },
  rise: { start:'stand', blurb:'From stand → reach deeper toward the floor (plant), hold, return to stand zero.',
    pose:(t,L)=>{ const u=0.5*(1-Math.cos(Math.min(1,t%4/2)*Math.PI)); // 0..1..0 over ~4s loop
      const phase=((t%4)<2)? u : 1-u;
      for(let i=0;i<6;i++){ L[i].yaw=0; L[i].hip=-0.55-0.35*phase; L[i].knee=0.85+0.25*phase; } } },
  'rise+': { start:'stand', blurb:'Like rise but higher/faster plant reach, then back to stand zero.',
    pose:(t,L)=>{ const u=0.5*(1-Math.cos(Math.min(1,(t%3)/1.4)*Math.PI));
      const phase=((t%3)<1.4)? u : 1-u;
      for(let i=0;i<6;i++){ L[i].yaw=0; L[i].hip=-0.35+0.55*phase; L[i].knee=0.85+0.2*phase; } } },
  rise_turn: { start:'stand', blurb:'Stand → plant → small yaw twist → untwist → stand zero (cord-safe).',
    pose:(t,L)=>{ const c=t%5; let hip=-0.55, knee=0.9, yaw=0;
      if(c<1.2){ const u=c/1.2; hip=-0.55+0.75*u; knee=0.9+0.15*u; }
      else if(c<2.0){ hip=0.2; knee=1.05; yaw=0.35*((c-1.2)/0.8); }
      else if(c<2.8){ hip=0.2; knee=1.05; yaw=0.35*(1-(c-2.0)/0.8); }
      else { const u=Math.min(1,(c-2.8)/1.2); hip=0.2-0.75*u; knee=1.05-0.15*u; }
      for(let i=0;i<6;i++){ L[i].yaw=yaw; L[i].hip=hip; L[i].knee=knee; } } },
  plant_look: { start:'stand', blurb:'Standing: body look left/right + nod (small yaw/pitch on plant).',
    pose:(t,L)=>{ const yaw=0.35*Math.sin(t*1.2), nod=0.12*Math.sin(t*2.0);
      for(let i=0;i<6;i++){ L[i].yaw=yaw; L[i].hip=-0.55+nod; L[i].knee=0.9; } } },
  walk: { start:'stand', blurb:'Real tripod gait: walk forward a few strides (~45 mm/s), then hold stand.',
    pose:(t,L)=>{ for(let i=0;i<6;i++){ const ph=(t*2.2)+(i%2)*Math.PI; const sw=Math.max(0,Math.sin(ph));
      L[i].yaw=0.12*Math.sin(ph); L[i].hip=0.35-0.15*sw; L[i].knee=1.2-0.2*sw; } } },
  walk_spin: { start:'stand', blurb:'In-place tripod turn (spin), then hold stand. Cord-friendlier than walking away.',
    pose:(t,L)=>{ for(let i=0;i<6;i++){ const ph=(t*2.5)+(i%2)*Math.PI; const sw=Math.max(0,Math.sin(ph));
      L[i].yaw=0.25*Math.sin(t*1.5); L[i].hip=0.35-0.12*sw; L[i].knee=1.2-0.15*sw; } } },
  walk_oval: { start:'stand', blurb:'Tripod: forward → spin → reverse → stand. Keep cord slack / clear floor.',
    pose:(t,L)=>{ DEMO_PREVIEW.walk.pose(t,L); } },
  plant_bounce: { start:'stand', blurb:'Standing boom/pop: chassis bobs by flexing hips/knees together.',
    pose:(t,L)=>{ const b=0.5*(1-Math.cos(t*Math.PI*2*1.1));
      for(let i=0;i<6;i++){ L[i].yaw=0; L[i].hip=-0.55-0.25*b; L[i].knee=0.9+0.2*b; } } },
  plant_ripple: { start:'stand', blurb:'Standing: ripple around the hex — legs lift in a traveling wave.',
    pose:(t,L)=>{ for(let i=0;i<6;i++){ const w=0.5*(1+Math.sin(t*3 - i*Math.PI/3));
      L[i].yaw=0.15*Math.sin(t*2+i); L[i].hip=-0.55+0.45*w; L[i].knee=0.9-0.35*w; } } },
  plant_gallop: { start:'stand', blurb:'Standing gallop: opposite leg pairs lift/plant in antiphase.',
    pose:(t,L)=>{ for(let i=0;i<6;i++){ const s=(i%2)?1:-1; const w=0.5*(1+Math.sin(t*4)*s);
      L[i].yaw=0; L[i].hip=-0.55+0.4*w; L[i].knee=0.9-0.3*w; } } },
  plant_tripod: { start:'stand', blurb:'Tripod flip: three legs up while three stay planted, then swap.',
    pose:(t,L)=>{ const side=Math.sin(t*2.2)>0?1:0;
      for(let i=0;i<6;i++){ const up=((i%2)===side)?1:0;
      L[i].yaw=0; L[i].hip=-0.55+0.5*up; L[i].knee=0.9-0.4*up; } } },
  plant_fan: { start:'stand', blurb:'All six legs dance outward/inward in a fan pattern.',
    pose:(t,L)=>{ const a=0.4*Math.sin(t*2.5);
      for(let i=0;i<6;i++){ L[i].yaw=a*((i%2)?-1:1); L[i].hip=-0.45+0.2*Math.sin(t*3+i);
      L[i].knee=0.85+0.15*Math.cos(t*3+i); } } },
  plant_star: { start:'stand', blurb:'Odds/evens + star snaps — quick planted accents.',
    pose:(t,L)=>{ const snap=Math.pow(Math.max(0,Math.sin(t*5)),8);
      for(let i=0;i<6;i++){ const o=i%2; L[i].yaw=(o?1:-1)*(0.2+0.35*snap);
      L[i].hip=-0.55+0.35*snap*(1-o); L[i].knee=0.9-0.25*snap*o; } } },
  plant_stomp: { start:'stand', blurb:'Stomp barrage then ta-da pose; stays planted / returns to stand.',
    pose:(t,L)=>{ const st=Math.abs(Math.sin(t*6));
      for(let i=0;i<6;i++){ const hit=((Math.floor(t*3)+i)%3===0)?st:0;
      L[i].yaw=0; L[i].hip=-0.55+0.35*hit; L[i].knee=0.9-0.25*hit; } } },
  rise_show: { start:'stand', blurb:'Full planted show: plant, then look/bounce/ripple/gallop/… finale → stand zero.',
    pose:(t,L)=>{ // mash of plant_ripple + bounce
      DEMO_PREVIEW.plant_ripple.pose(t,L);
      const b=0.15*(1-Math.cos(t*2));
      for(let i=0;i<6;i++){ L[i].hip-=b; L[i].knee+=b*0.5; } } },
};
let prevName = 'breathe';
let prevRaf = 0;
let prevT0 = 0;
function demoPreviewInfo(name){
  return DEMO_PREVIEW[name] || {
    start: /^(plant|rise|stand|walk)/.test(name||'') ? 'stand' : 'sit',
    blurb: 'Schematic motion preview.',
    pose:(t,L)=>{ for(let i=0;i<6;i++){ L[i].yaw=0; L[i].hip=-0.4; L[i].knee=0.7; } }
  };
}
function setDemoPreview(name){
  prevName = name || prevName;
  const info = demoPreviewInfo(prevName);
  const title = $('dprevtitle'); if(title) title.textContent = prevName;
  const blurb = $('dprevblurb'); if(blurb) blurb.textContent = info.blurb;
  const tags = $('dprevtags');
  if(tags){
    tags.innerHTML = info.start==='sit'
      ? '<span class="tag sit">starts sit zero</span><span class="tag">air</span>'
      : '<span class="tag stand">starts stand zero</span><span class="tag">planted</span>';
  }
  document.querySelectorAll('#dgrid button').forEach(b=>{
    b.classList.toggle('previewing', b.dataset.name===prevName);
  });
}
// --- 3D demo view --------------------------------------------------------
// Real leg geometry (coxa 12.5 / femur 90 / tibia 128 mm), perspective
// camera with drag-orbit + wheel-zoom. While a demo runs it renders the
// LIVE robot from /api/pose encoder angles; idle/hover shows the demo's
// schematic motion mapped onto the same skeleton.
const HEX3D = {
  az: -38, el: 26, dist: 640, f: 430,
  spin: true, drag: null, bound: false,
  coxa: 12.5, femur: 90, tibia: 128, legR: 55, hexR: 63.5, bodyT: 14,
};
let livePose = { deg: null, ts: 0 };
let livePoseBusy = false, livePollAt = 0;
async function pollLivePose(){
  if(livePoseBusy) return; livePoseBusy = true;
  try{
    const r = await fetch('/api/pose', {cache:'no-store'});
    const j = await r.json();
    if(j && j.ok && Array.isArray(j.degrees))
      livePose = { deg: j.degrees.map(v=>v==null?0:Number(v)),
                   ts: performance.now() };
  }catch(e){ /* keep last */ }
  finally{ livePoseBusy = false; }
}
function hex3dBind(cv){
  if(HEX3D.bound) return; HEX3D.bound = true;
  cv.style.touchAction = 'none';
  cv.style.cursor = 'grab';
  cv.addEventListener('pointerdown', e=>{
    HEX3D.drag = {x:e.clientX, y:e.clientY}; HEX3D.spin = false;
    cv.setPointerCapture(e.pointerId); cv.style.cursor='grabbing'; });
  cv.addEventListener('pointermove', e=>{
    if(!HEX3D.drag) return;
    HEX3D.az += (e.clientX - HEX3D.drag.x)*0.5;
    HEX3D.el = Math.max(8, Math.min(72, HEX3D.el + (e.clientY - HEX3D.drag.y)*0.35));
    HEX3D.drag = {x:e.clientX, y:e.clientY}; });
  const up = ()=>{ HEX3D.drag = null; cv.style.cursor='grab'; };
  cv.addEventListener('pointerup', up);
  cv.addEventListener('pointercancel', up);
  cv.addEventListener('wheel', e=>{ e.preventDefault();
    HEX3D.dist = Math.max(340, Math.min(1300,
      HEX3D.dist * (e.deltaY>0 ? 1.08 : 0.93))); }, {passive:false});
}
// (yaw,hip,knee in DEG, robot convention: +hip/+knee fold down) → 4 pts, mm.
function hex3dLeg(i, yawDeg, hipDeg, kneeDeg){
  const A = (i+0.5)*Math.PI/3;
  const yaw = yawDeg*Math.PI/180, hip = hipDeg*Math.PI/180,
        knee = kneeDeg*Math.PI/180;
  const dphi = A + yaw, dx = Math.cos(dphi), dy = Math.sin(dphi);
  const p0 = [HEX3D.legR*Math.cos(A), HEX3D.legR*Math.sin(A), 0];
  const p1 = [p0[0]+HEX3D.coxa*dx, p0[1]+HEX3D.coxa*dy, 0];
  const ch = Math.cos(hip), sh = Math.sin(hip);
  const p2 = [p1[0]+HEX3D.femur*ch*dx, p1[1]+HEX3D.femur*ch*dy,
              -HEX3D.femur*sh];
  const ck = Math.cos(hip+knee), sk = Math.sin(hip+knee);
  const p3 = [p2[0]+HEX3D.tibia*ck*dx, p2[1]+HEX3D.tibia*ck*dy,
              p2[2]-HEX3D.tibia*sk];
  return [p0, p1, p2, p3];
}
function drawDemoPreview(ts){
  const cv = $('dprevcv'); if(!cv) return;
  if(activeView!=='demos'){ prevRaf = requestAnimationFrame(drawDemoPreview); return; }
  hex3dBind(cv);
  const ctx = cv.getContext('2d');
  const W = cv.width, H = cv.height;
  ctx.clearRect(0,0,W,H);
  ctx.fillStyle = '#12151c'; ctx.fillRect(0,0,W,H);

  // ---- joint angles: LIVE while a demo runs, else the hover schematic ----
  const running = !!(lastDemo && lastDemo.running);
  if(running && ts - livePollAt > 250){ livePollAt = ts; pollLivePose(); }
  const liveFresh = livePose.deg && (performance.now() - livePose.ts) < 1500;
  const useLive = running && liveFresh;
  const deg = new Array(18).fill(0);
  const info = demoPreviewInfo(prevName);
  if(useLive){
    for(let k=0;k<18;k++) deg[k] = livePose.deg[k] || 0;
  } else {
    if(!prevT0) prevT0 = ts;
    const t = (ts - prevT0) / 1000;
    const legs = [];
    for(let i=0;i<6;i++) legs.push(
      info.start==='sit' ? {yaw:0, hip:0, knee:0}
                         : {yaw:0, hip:-0.55, knee:0.9});
    try{ info.pose(t, legs); }catch(e){}
    // Schematic pose fns are stylised radians. Map onto the real robot
    // convention (+hip/+knee = fold down): sit demos already use robot
    // signs; stand demos are anchored so (-0.55, 0.9) = the real plant
    // stance (hip +19.5°, knee +78.9°).
    for(let i=0;i<6;i++){
      const L = legs[i];
      deg[3*i] = L.yaw*57.3;
      if(info.start==='sit'){ deg[3*i+1] = L.hip*57.3; deg[3*i+2] = L.knee*57.3; }
      else { deg[3*i+1] = -L.hip*57.3*0.62; deg[3*i+2] = L.knee*57.3*1.53; }
    }
  }

  // ---- forward kinematics + body height ----
  const legPts = [];
  let minz = 0;
  for(let i=0;i<6;i++){
    const pts = hex3dLeg(i, deg[3*i], deg[3*i+1], deg[3*i+2]);
    legPts.push(pts);
    minz = Math.min(minz, pts[3][2]);
  }
  const bz = Math.max(24, -minz);            // lowest foot on the floor
  if(HEX3D.spin && !useLive) HEX3D.az += 0.15;

  // ---- camera ----
  const az = HEX3D.az*Math.PI/180, el = HEX3D.el*Math.PI/180;
  const ca = Math.cos(az), sa = Math.sin(az);
  const ce = Math.cos(el), se = Math.sin(el);
  const zc = Math.min(110, bz*0.75 + 30);    // look-at height
  const cx = W/2, cy = H/2 + 14;
  function proj(p){
    const x1 = p[0]*ca - p[1]*sa;
    const y1 = p[0]*sa + p[1]*ca;
    const z1 = p[2] - zc;                     // p already in world z
    const y2 = y1*ce + z1*se;
    const z2 = -y1*se + z1*ce;
    const d = HEX3D.dist - y2;
    return {x: cx + HEX3D.f*x1/d, y: cy - HEX3D.f*z2/d, d};
  }
  const world = p => [p[0], p[1], p[2] + bz];

  // ---- ground grid + shadows (always behind) ----
  ctx.lineWidth = 1; ctx.strokeStyle = '#1c2331';
  for(let g=-220; g<=220; g+=55){
    let a = proj([g,-220,0]), b = proj([g,220,0]);
    ctx.beginPath(); ctx.moveTo(a.x,a.y); ctx.lineTo(b.x,b.y); ctx.stroke();
    a = proj([-220,g,0]); b = proj([220,g,0]);
    ctx.beginPath(); ctx.moveTo(a.x,a.y); ctx.lineTo(b.x,b.y); ctx.stroke();
  }
  ctx.fillStyle = 'rgba(0,0,0,0.30)';
  ctx.beginPath();
  for(let i=0;i<6;i++){
    const v = proj([HEX3D.hexR*Math.cos(i*Math.PI/3),
                    HEX3D.hexR*Math.sin(i*Math.PI/3), 0]);
    if(i===0) ctx.moveTo(v.x,v.y); else ctx.lineTo(v.x,v.y);
  }
  ctx.closePath(); ctx.fill();
  for(const pts of legPts){
    const f = proj([pts[3][0], pts[3][1], 0]);
    ctx.beginPath(); ctx.ellipse(f.x, f.y, 5, 5*Math.max(0.25,se), 0, 0, 7);
    ctx.fill();
  }

  // ---- depth-sorted primitives: body + legs ----
  const prims = [];
  // body hexagonal prism
  const topV = [], botV = [];
  for(let i=0;i<6;i++){
    const a = i*Math.PI/3;
    topV.push(world([HEX3D.hexR*Math.cos(a), HEX3D.hexR*Math.sin(a),  HEX3D.bodyT]));
    botV.push(world([HEX3D.hexR*Math.cos(a), HEX3D.hexR*Math.sin(a), -HEX3D.bodyT]));
  }
  prims.push({ d: proj(world([0,0,0])).d, draw(){
    const t = topV.map(proj), b = botV.map(proj);
    ctx.fillStyle = '#141c2c';
    ctx.beginPath(); b.forEach((v,i)=> i?ctx.lineTo(v.x,v.y):ctx.moveTo(v.x,v.y));
    ctx.closePath(); ctx.fill();
    ctx.strokeStyle = '#23304d'; ctx.lineWidth = 1.5;
    for(let i=0;i<6;i++){ ctx.beginPath();
      ctx.moveTo(t[i].x,t[i].y); ctx.lineTo(b[i].x,b[i].y); ctx.stroke(); }
    ctx.fillStyle = '#1b2744';
    ctx.beginPath(); t.forEach((v,i)=> i?ctx.lineTo(v.x,v.y):ctx.moveTo(v.x,v.y));
    ctx.closePath(); ctx.fill();
    ctx.strokeStyle = '#2b6cff'; ctx.lineWidth = 2; ctx.stroke();
    // heading notch: front is +x (between legs 0 and 5)
    const n = proj(world([HEX3D.hexR*0.72, 0, HEX3D.bodyT]));
    ctx.fillStyle = '#2b6cff';
    ctx.beginPath(); ctx.arc(n.x, n.y, 3, 0, 7); ctx.fill();
  }});
  // legs
  const segCol = ['#6c7891', '#9aa3b2', '#c3cad8'];
  for(let i=0;i<6;i++){
    const wp = legPts[i].map(world);
    for(let s=0;s<3;s++){
      const a = wp[s], b = wp[s+1];
      prims.push({ d: proj([(a[0]+b[0])/2,(a[1]+b[1])/2,(a[2]+b[2])/2]).d,
        draw(){
          const pa = proj(a), pb = proj(b);
          ctx.strokeStyle = segCol[s]; ctx.lineWidth = s===0?3:4;
          ctx.lineCap = 'round';
          ctx.beginPath(); ctx.moveTo(pa.x,pa.y); ctx.lineTo(pb.x,pb.y); ctx.stroke();
          if(s===2){                       // foot
            const onGround = b[2] < 6;
            ctx.beginPath(); ctx.arc(pb.x, pb.y, onGround?4.5:3.5, 0, 7);
            ctx.fillStyle = onGround ? '#5fd08a' : '#ffb24d'; ctx.fill();
          } else {                          // joint
            ctx.beginPath(); ctx.arc(pb.x, pb.y, 2.5, 0, 7);
            ctx.fillStyle = '#39445c'; ctx.fill();
          }
        }});
    }
  }
  prims.sort((p,q)=> q.d - p.d);            // far → near
  prims.forEach(p=> p.draw());

  // ---- labels ----
  ctx.font = '11px system-ui,sans-serif';
  if(useLive){
    ctx.fillStyle = '#5fd08a';
    ctx.fillText('● LIVE — robot encoder angles', 10, 17);
  } else {
    ctx.fillStyle = '#5a6478';
    ctx.fillText(running ? 'connecting to live pose…'
                         : 'preview — schematic motion', 10, 17);
  }
  ctx.fillStyle = '#3c455c';
  ctx.fillText('drag = orbit · wheel = zoom', W-158, H-8);

  prevRaf = requestAnimationFrame(drawDemoPreview);
}
function startDemoPreviewLoop(){
  if(prevRaf) cancelAnimationFrame(prevRaf);
  prevT0 = 0;
  prevRaf = requestAnimationFrame(drawDemoPreview);
}

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
    if(items.length) setDemoPreview(items[0].name);
    startDemoPreviewLoop();
  }catch(e){ $('dgrid').innerHTML = '<div class="hint">Failed to load demos</div>'; }
}
function demoButton(item){
      const b = document.createElement('button');
      b.dataset.name = item.name;
      b.innerHTML = '<b>'+item.name+'</b>'
        +(item.live_speed?' <span style="color:#5fd08a;font-size:10px;font-weight:700">LIVE</span>':'')
        +'<br><span style="color:#9aa3b2;font-weight:400;font-size:12px">'
        +item.title+'</span>';
      b.onmouseenter = ()=> setDemoPreview(item.name);
      b.onfocus = ()=> setDemoPreview(item.name);
      b.onclick = async ()=>{
        setDemoPreview(item.name);
        if(needArm()) return;
        const sp = demoSpeed();
        const body = {name:item.name, speed:sp, torque:demoTorque(),
                      seconds:demoDuration()};
        if(item.name==='breathe' || item.name==='breathe_v' || item.has_size){
          body.size = demoSize();
          body.rate = demoRate();
          body.softness = demoSoft();
        }
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
          showSent(msg);
        } else {
          showSent(j.error||'failed', true);
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
  showSent('stopping…');
  const r = await fetch('/api/demo/stop',{method:'POST'});
  try{
    const j = await r.json();
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
   ['dance_wild', 'dance_swarm_stand', 'dance_steeple', 'dance',
    'dance_walk', 'rise_show']],
];
async function loadDance(){
  try{
    const r = await fetch('/api/demos'); const d = await r.json();
    const byName = {};
    (d.demos||[]).forEach(it=>{ byName[it.name] = it; });
    const g = $('dancegrid'); g.innerHTML='';
    DANCE_SETS.forEach(([title, sub, names])=>{
      const items = names.map(n=>byName[n]).filter(Boolean);
      if(!items.length) return;
      const h = document.createElement('div');
      h.className = 'demo-group';
      h.innerHTML = title+' <span class="sub">· '+sub+'</span>';
      g.appendChild(h);
      items.forEach(item=> g.appendChild(demoButton(item)));
    });
  }catch(e){ $('dancegrid').innerHTML = '<div class="hint">Failed to load shows</div>'; }
}
$('dancestop').onclick = ()=> $('dstop').onclick();
$('dancespeed').oninput = ()=>{
  $('dspeed').value = $('dancespeed').value;   // one shared speed setting
  $('dancespeedlab').textContent = demoSpeed().toFixed(2);
  $('dspeed').oninput();                       // live-push if a show runs
};

$('dzero').onclick = ()=> goPoseZero('sit', 'sit zero');
$('dstand').onclick = ()=> goPoseZero('stand', 'stand zero');

// --- Quad tab (tip-back four-leg walk) --------------------------------------
function quadSpeed(){ return Math.max(0.25, Math.min(2.0, (+$('qspeed').value)/100)); }
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
  const body = {name, speed:sp,
                seconds:Math.max(20, Math.min(300, +($('qdur').value)||40))};
  showSent(label+' @ '+sp.toFixed(2)+'×…');
  const res = await fetch('/api/demo',{method:'POST',
    headers:{'Content-Type':'application/json'},
    body: JSON.stringify(body)});
  const j = await res.json();
  if(j.ok) showSent(label+' @ '+sp.toFixed(2)+'× for '+body.seconds+'s'
                    +(j.home?' (via '+j.home+' zero)':''));
  else showSent(j.error||'failed', true);
  if(j.demo) paintDemoStatus(j.demo);
  if(j.robot) paintRobotActivity(j.robot);
  startDemoPoll();
  refreshRobotState(true);
}
$('qstart').onclick = ()=> quadRun('quad_walk', 'quad walk');
$('qtrot').onclick = ()=> quadRun('quad_trot', 'quad trot');
$('qstop').onclick = async ()=>{
  showSent('stopping…');
  const r = await fetch('/api/demo/stop',{method:'POST'});
  try{
    const j = await r.json();
    if(j.demo) paintDemoStatus(j.demo);
    if(j.robot) paintRobotActivity(j.robot);
  }catch(e){}
  showSent('quad stop');
  refreshRobotState(true);
};
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
  bar.classList.toggle('sim', backendKind === 'sim');
  if(backendKind === 'sim'){
    bar.classList.remove('armed', 'disarmed');
    $('armstate').textContent = '● SIM MODE — MuJoCo backend';
    $('armbtn').textContent = 'Reset sim stand';
    $('estop').textContent = '■ Stop sim';
    return;
  }
  bar.classList.toggle('armed', servosArmed);
  bar.classList.toggle('disarmed', !servosArmed);
  $('armstate').textContent = servosArmed ? '● ARMED — servos live'
                                          : '● SERVOS OFF (disarmed)';
  $('armbtn').textContent   = servosArmed ? 'Disarm (servos off)'
                                          : 'Enable servos (power on)';
}
function setArmed(on){ servosArmed = on; if(!on) armed = false; updateArmUI(); }
function armServos(){
  if(backendKind === 'sim'){
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
// INSTANT limp: cut all PWM NOW (true emergency stop; the robot drops). Always
// allowed, even while disarmed, and used for the boot-time safe default.
function disarmServos(){
  dbgTestAbort = true; cmd('X'); setArmed(false);
  showSent(backendKind === 'sim'
    ? 'SIM — stopped, stance policy holds'
    : 'EMERGENCY STOP — servos limp NOW');
}
// Returns true (and warns) when disarmed; every servo-driving action calls it.
function needArm(){
  if(backendKind === 'sim') return false;
  if(servosArmed) return false;
  showSent('⚠ Servos disarmed — press “Enable servos” first');
  return true;
}
// The Disarm toggle is a NORMAL power-off -> graceful lower then limp.
$('armbtn').onclick = ()=> servosArmed ? settleServos() : armServos();
// EMERGENCY STOP is the ONLY instant-limp control (cuts PWM immediately).
$('estop').onclick  = disarmServos;
// Enforce the safe default on EVERY page load: show disarmed AND tell the
// firmware to disarm now — harmless if it just booted disarmed, and it clears
// any stale ARMED state from a prior session so the page's OFF state is real.
setArmed(false);
cmd('X');
