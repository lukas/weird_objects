#!/usr/bin/env python3
"""
Web control panel for the hexapod.

Runs on the robot's onboard Linux side (the Arduino UNO Q's Qualcomm
processor) and serves a single-page control UI over HTTP.  Open it from any
computer/phone on the same network (e.g. http://<board-ip>:8080) and drive the
robot with on-screen joysticks, your keyboard, OR an Xbox controller plugged
into YOUR computer (via the browser Gamepad API -- no Bluetooth pairing to the
board needed).

The page sends firmware command lines to this server, which forwards them to
the STM32 sketch through the on-board arduino-router monitor port
(tcp:127.0.0.1:7500) -- the same pipe xbox_drive.py and the interactive
`socat tcp:127.0.0.1:7500 -` session use.

Pure standard library (no Flask/pip).  Needs the firmware build that has the
live-drive `J vx vy omega [gait]` command.

An HTTPS server is also started (default :8443) using a self-signed cert.
Browsers only expose the Gamepad API in a "secure context" (HTTPS or
localhost), so an Xbox controller plugged into your computer is ONLY visible
to the page over the https:// URL -- the plain http:// page still works for
on-screen + keyboard control.

  python3 web_drive.py                 # serve on 0.0.0.0:8080 (+ https :8443)
  python3 web_drive.py --http-port 9000
"""

import argparse
import json
import os
import socket
import ssl
import subprocess
import threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

# Self-signed TLS material for the HTTPS listener (generated on first run).
CERT_FILE = os.path.expanduser("~/.hexapod_cert.pem")
KEY_FILE = os.path.expanduser("~/.hexapod_key.pem")


def ensure_cert():
    """Make a long-lived self-signed cert if we don't have one. Returns True
    when both files are present afterwards."""
    if os.path.exists(CERT_FILE) and os.path.exists(KEY_FILE):
        return True
    try:
        subprocess.run(
            ["openssl", "req", "-x509", "-newkey", "rsa:2048", "-nodes",
             "-keyout", KEY_FILE, "-out", CERT_FILE, "-days", "3650",
             "-subj", "/CN=hexapod.local",
             "-addext", "subjectAltName=DNS:hexapod.local,DNS:localhost"],
            check=True, capture_output=True)
        print(f"[https] generated self-signed cert at {CERT_FILE}")
        return True
    except (OSError, subprocess.CalledProcessError) as e:
        print(f"[https] cert generation failed ({e}); HTTPS disabled")
        return False

# Persisted calibration (the UNO Q has no EEPROM, so the standing foot height
# the user dials in is remembered HERE, on the board's Linux side, and pushed
# to the firmware with `Z <mm>` on startup / page load).
CAL_FILE = os.path.expanduser("~/.hexapod_cal.json")


def load_cal():
    try:
        with open(CAL_FILE) as f:
            return json.load(f)
    except (OSError, ValueError):
        return {}


def save_cal(d):
    try:
        tmp = CAL_FILE + ".tmp"
        with open(tmp, "w") as f:
            json.dump(d, f)
        os.replace(tmp, CAL_FILE)
        return True
    except OSError as e:
        print(f"[cal] save failed: {e}")
        return False


CAL = load_cal()


# --------------------------------------------------------------- serial link
class Link:
    """Thread-safe line sender to the firmware over the router monitor port."""

    def __init__(self, host="127.0.0.1", port=7500):
        self.host, self.port = host, port
        self.sock = None
        self.lock = threading.Lock()

    def _connect(self):
        try:
            s = socket.create_connection((self.host, self.port), timeout=3.0)
            s.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            self.sock = s
            # Drain (and discard) anything the firmware emits -- we only send,
            # but if its replies aren't read the socket buffer fills and the
            # link eventually stalls.
            threading.Thread(target=self._drain, args=(s,), daemon=True).start()
            print(f"[link] connected to {self.host}:{self.port}")
        except OSError as e:
            self.sock = None
            print(f"[link] connect failed: {e}")

    def _drain(self, s):
        try:
            while True:
                if not s.recv(4096):
                    break
        except OSError:
            pass

    def send(self, line):
        with self.lock:
            if self.sock is None:
                self._connect()
                if self.sock is None:
                    return False
            try:
                self.sock.sendall((line.rstrip("\n") + "\n").encode("ascii", "ignore"))
                return True
            except OSError as e:
                print(f"[link] send failed ({e}); will reconnect")
                try:
                    self.sock.close()
                finally:
                    self.sock = None
                return False


LINK = None   # set in main()
HTTPS_PORT = None   # actual HTTPS port that bound (443 if privileged, else 8443)


PAGE = r"""<!doctype html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1, maximum-scale=1, user-scalable=no">
<title>Hexapod control</title>
<style>
  :root { color-scheme: dark; }
  * { box-sizing: border-box; -webkit-tap-highlight-color: transparent; }
  body { margin:0; font-family: system-ui, -apple-system, Segoe UI, Roboto, sans-serif;
         background:#0f1115; color:#e7eaf0; }
  header { padding:10px 16px; display:flex; align-items:center; gap:12px;
           border-bottom:1px solid #232733; }
  header h1 { font-size:16px; margin:0; font-weight:600; }
  #conn { font-size:12px; color:#9aa3b2; }
  #conn.bad { color:#ff6b6b; } #conn.ok { color:#5fd08a; }
  .wrap { max-width:880px; margin:0 auto; padding:16px; }
  .row { display:flex; gap:16px; flex-wrap:wrap; }
  .pad { flex:1 1 260px; background:#161a22; border:1px solid #232733; border-radius:14px;
         padding:14px; }
  .pad h2 { margin:0 0 8px; font-size:13px; color:#9aa3b2; font-weight:600; text-transform:uppercase;
            letter-spacing:.5px; }
  canvas { touch-action:none; width:100%; height:200px; display:block; border-radius:10px;
           background:#0c0e13; }
  .turn canvas { height:90px; }
  .btns { display:flex; gap:8px; flex-wrap:wrap; margin-top:10px; }
  button { flex:1 1 auto; min-width:64px; padding:12px 10px; border-radius:10px; border:1px solid #2c3140;
           background:#1b2030; color:#e7eaf0; font-size:14px; font-weight:600; cursor:pointer; }
  button:active { background:#27304a; }
  button.on { background:#2b6cff; border-color:#2b6cff; }
  button.danger { border-color:#7a2b2b; } button.danger:active { background:#5a2020; }
  .seg button { flex:1; }
  label.slab { display:block; font-size:12px; color:#9aa3b2; margin:10px 0 4px; }
  input[type=range] { width:100%; }
  .hint { font-size:12px; color:#8089a0; line-height:1.5; margin-top:8px; }
  kbd { background:#222838; border:1px solid #333b52; border-radius:4px; padding:1px 5px; font-size:11px; }
  #sent { font-family: ui-monospace, monospace; font-size:12px; color:#5fd08a; }
  #gp { font-size:12px; color:#8089a0; }
</style>
</head>
<body>
<header>
  <h1>🕷 Hexapod control</h1>
  <span id="conn" class="ok">ready</span>
  <span id="gp"></span>
  <span style="flex:1"></span>
  <span id="sent"></span>
</header>
<div class="wrap">
  <div class="row">
    <div class="pad">
      <h2>Drive (left stick)</h2>
      <canvas id="drive"></canvas>
      <div class="turn"><h2 style="margin-top:12px">Turn</h2><canvas id="turn"></canvas></div>
    </div>
    <div class="pad">
      <h2>Gait <span style="color:#8089a0;font-weight:400">(controller: right-stick click)</span></h2>
      <div class="btns seg" id="gaits">
        <button data-gait="0" class="on">Tripod</button>
        <button data-gait="1">Ripple</button>
        <button data-gait="2">Wave</button>
        <button data-gait="3">Tetrapod</button>
      </div>
      <h2 style="margin-top:14px">Dances <span style="color:#8089a0;font-weight:400">(hover for controller combo)</span></h2>
      <div class="btns" id="dances">
        <button data-cmd="M1">Wave 👋</button>
        <button data-cmd="M2">Say hi</button>
        <button data-cmd="M3">Hula</button>
        <button data-cmd="M4">Twist</button>
        <button data-cmd="M5">March</button>
        <button data-cmd="M6">Boogie</button>
        <button data-cmd="M7">Pinwheel</button>
        <button data-cmd="M8">Freakout 🤪</button>
        <button data-cmd="M9">Pogo</button>
        <button data-cmd="M10">Cancan</button>
        <button data-cmd="M11">Corkscrew</button>
        <button data-cmd="M12">Shimmy</button>
        <button data-cmd="M13">Twist-stomp</button>
        <button data-cmd="M14">Tippy-taps</button>
        <button data-cmd="M15">Disco</button>
        <button data-cmd="M16">Rave 🎉</button>
        <button data-cmd="M17">Breathe</button>
        <button data-cmd="M18">Sway</button>
        <button data-cmd="M19">Nod</button>
        <button data-cmd="M20">Slow wave</button>
      </div>
      <h2 style="margin-top:14px">Pose</h2>
      <div class="btns">
        <button data-cmd="P" title="Controller: A">Stand</button>
        <button data-cmd="U" title="Controller: Y">Legs up</button>
        <button data-cmd="C" title="Controller: X">Center</button>
        <button data-cmd="X" class="danger" title="Controller: B">Relax</button>
      </div>
      <button id="stop" class="danger" style="margin-top:10px">■ STOP / disarm</button>

      <h2 style="margin-top:16px">Stand height (calibrate)</h2>
      <label class="slab">Body height: <span id="standlab">120</span> mm
        &nbsp;<span style="color:#8089a0">(left = lower / sit, right = taller / lift)</span></label>
      <input id="standh" type="range" min="50" max="160" value="120">
      <div class="btns">
        <button id="standdn">▼ lower 2mm</button>
        <button id="standup">▲ taller 2mm</button>
        <button id="savestand" class="on">✓ Save height</button>
      </div>
      <div class="hint">Drag until the feet plant firmly and the body sits at
        the height you want (it eases to each new height when you let go), then
        <b>Save</b>. It's remembered on the robot and re-applied on boot.</div>

      <label class="slab">Max speed: <span id="vlab">55</span> mm/s</label>
      <input id="vmax" type="range" min="15" max="110" value="55">

      <label class="slab" title="How high each foot lifts off the ground on every step.">
        Swing lift: <span id="klab">18</span> mm</label>
      <input id="lift" type="range" min="4" max="40" value="18"
        title="How high each foot lifts during a step. Lower = smoother, steadier (less body rock per step) but clears smaller bumps; higher = steps over rough ground/obstacles but rocks more.">
      <div class="hint"><b>Swing lift</b> = how high a foot lifts each step.
        <b>Lower</b> (e.g. 8–12 mm) = the body bobs/rocks less per step, so it's
        steadier and smoother on flat ground. <b>Higher</b> (25–40 mm) = it picks
        its feet up more to clear bumps or carpet, at the cost of more body sway.</div>

      <label class="slab" title="Shifts the body's resting position a few mm to keep its weight balanced over its feet.">
        COM lean trim X / Y (mm)
        &nbsp;<span style="color:#8089a0">(controller: D-pad, no trigger)</span></label>
      <input id="comx" type="range" min="-25" max="25" value="0"
        title="COM trim X: shift the body forward (+) / back (-) in mm. Use it to re-center the weight if the robot tips or drifts front/back.">
      <input id="comy" type="range" min="-25" max="25" value="0"
        title="COM trim Y: shift the body left (+) / right (-) in mm. Use it to re-center the weight if a side-heavy load (wires/battery) makes it lean or drift to one side.">
      <div class="hint"><b>COM trim</b> (center-of-mass lean) nudges the whole
        body a few mm so its weight sits centered over its feet. The legs, wire
        bundle and battery aren't perfectly balanced, so the robot can lean or
        drift to one side and walk less steadily. If it leans/drifts, trim it the
        <i>opposite</i> way (<b>X</b> = forward/back, <b>Y</b> = left/right) until
        it stands square — that steadies both the stance and the gait.</div>

      <div class="hint">
        <b>Keyboard:</b> <kbd>W</kbd><kbd>A</kbd><kbd>S</kbd><kbd>D</kbd> drive,
        <kbd>Q</kbd>/<kbd>E</kbd> turn, <kbd>1</kbd>/<kbd>2</kbd>/<kbd>3</kbd>/<kbd>4</kbd> gait,
        <kbd>Space</kbd> stand, <kbd>V/B/O/T</kbd> + <kbd>5/6/7/8</kbd> dances.<br>
        <b>Gamepad:</b> plug an Xbox pad into THIS computer — it auto-drives
        (left stick + right stick turn). <b>D-pad alone = COM trim.</b>
        <b>Right-stick click = cycle gait.</b> Dances: hold a shoulder/trigger +
        D-pad — <kbd>LB</kbd>=1-4, <kbd>RB</kbd>=5-8, <kbd>LT</kbd>=9-12,
        <kbd>RT</kbd>=13-16, <kbd>LB+RB</kbd>=17-20 (D-pad U/R/D/L picks within).
        A/B/X/Y = Stand/Relax/Center/Legs-up.
      </div>
    </div>
  </div>
</div>
<script>
const conn = document.getElementById('conn');
const sentEl = document.getElementById('sent');
const gpEl = document.getElementById('gp');
let gait = 0, armed = false, dancePaused = false, lastInput = 0;
let maxVx = 55, maxVy = 40, maxOmega = 0.8;

// --- command transport -----------------------------------------------------
async function cmd(line){
  try {
    const r = await fetch('/cmd', {method:'POST', body:line});
    if(!r.ok) throw 0;
    conn.textContent='connected'; conn.className='ok';
  } catch(e){ conn.textContent='link error'; conn.className='bad'; }
}
function showSent(line){ sentEl.textContent = line; }

// --- on-screen joysticks ----------------------------------------------------
function makeStick(canvas, horizontalOnly){
  const ctx = canvas.getContext('2d');
  let active=false, nx=0, ny=0;          // normalized -1..1, ny up = +
  function resize(){ const r=canvas.getBoundingClientRect();
    canvas.width=r.width*devicePixelRatio; canvas.height=r.height*devicePixelRatio;
    ctx.setTransform(devicePixelRatio,0,0,devicePixelRatio,0,0); draw(); }
  function draw(){
    const w=canvas.width/devicePixelRatio, h=canvas.height/devicePixelRatio;
    ctx.clearRect(0,0,w,h);
    const cx=w/2, cy=h/2, R=Math.min(w,h)/2-12;
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
    const cx=r.width/2, cy=r.height/2, R=Math.min(r.width,r.height)/2-12;
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
  if(k==='1'){setGait(0);return;} if(k==='2'){setGait(1);return;}
  if(k==='3'){setGait(2);return;} if(k==='4'){setGait(3);return;}
  if(k==='5'||k==='6'||k==='7'||k==='8'){ doDance('M'+k); return; }
  if(k===' '){ disc('P'); return; }
  if(k==='v'||k==='b'||k==='o'||k==='t'){ doDance(k.toUpperCase()); return; }
  if(k==='c'){ disc('C'); return; } if(k==='u'){ disc('U'); return; }
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
let gpPrev = [];
function httpsUrl(){
  const p = __HTTPS_PORT__;
  return 'https://'+location.hostname+(p===443?'':(':'+p))+location.pathname;
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
  gpEl.textContent='🎮 '+gp.id.slice(0,18);
  const b=gp.buttons.map(x=>x.pressed);
  const press=i=> b[i] && !gpPrev[i];
  const val=i=> (gp.buttons[i] ? gp.buttons[i].value : 0);
  // A held shoulder/trigger picks the dance "bank"; the D-pad then picks one of
  // four within it.  With NO modifier the D-pad nudges COM trim instead.
  //   LB=1-4  RB=5-8  LT=9-12  RT=13-16  LB+RB=17-20   (U/R/D/L = +1/2/3/4)
  const lb=b[4], rb=b[5], lt=val(6)>0.4, rt=val(7)>0.4;
  let bank=-1;
  if(lb&&rb) bank=4; else if(lb) bank=0; else if(rb) bank=1;
  else if(lt) bank=2; else if(rt) bank=3;
  // D-pad: dance (if a modifier is held) or trim nudge (if not).
  if(press(12)){ bank>=0 ? doDance('M'+(bank*4+1)) : trimNudge( 2, 0); } // up: lean fwd
  if(press(13)){ bank>=0 ? doDance('M'+(bank*4+3)) : trimNudge(-2, 0); } // down: lean back
  if(press(14)){ bank>=0 ? doDance('M'+(bank*4+4)) : trimNudge( 0, 2); } // left: lean left
  if(press(15)){ bank>=0 ? doDance('M'+(bank*4+2)) : trimNudge( 0,-2); } // right: lean right
  if(press(11)) setGait((gait+1)%4);          // right-stick click: cycle gait
  if(press(0)) disc('P');                     // A
  if(press(1)) disc('X');                     // B
  if(press(2)) disc('C');                     // X
  if(press(3)) disc('U');                     // Y
  gpPrev=b;
  const dz=v=>Math.abs(v)<0.12?0:v;
  const ax0=dz(gp.axes[0]||0), ax1=dz(gp.axes[1]||0), ax2=dz(gp.axes[2]||0);
  if(ax0||ax1||ax2){ lastInput=performance.now(); armed=true; dancePaused=false; }
  return {x:ax0, y:-ax1, t:ax2};
}

// --- gait / dance helpers ---------------------------------------------------
function setGait(g){ gait=g;
  document.querySelectorAll('#gaits button').forEach(btn=>
    btn.classList.toggle('on', +btn.dataset.gait===g)); }
function doDance(c){ cmd(c); dancePaused=true; armed=true; forceResend();
  showSent('dance '+c+' (move to resume)'); }
function disc(c){ cmd(c); forceResend(); if('PUX'.includes(c)) armed=false; }

// Nudge the COM lean trim sliders (also what the bare D-pad does on the pad).
function trimNudge(dx, dy){
  comx.value = clamp(+comx.value + dx, -25, 25);
  comy.value = clamp(+comy.value + dy, -25, 25);
  sendCom();
  showSent('trim X='+comx.value+' Y='+comy.value+' mm');
}

document.querySelectorAll('#gaits button').forEach(btn=>
  btn.onclick=()=>setGait(+btn.dataset.gait));
function isDance(c){ return 'VBOT'.includes(c) || c[0]==='M'; }
document.querySelectorAll('button[data-cmd]').forEach(btn=>
  btn.onclick=()=>{ const c=btn.dataset.cmd;
    if(isDance(c)) doDance(c); else disc(c); });
document.getElementById('stop').onclick=()=>{ cmd('P'); forceResend(); armed=false; showSent('stopped'); };

// Tooltip each dance button with the equivalent controller combo.
(function(){
  const banks=['LB','RB','LT','RT','LB+RB'], dirs=['Up','Right','Down','Left'];
  document.querySelectorAll('#dances button').forEach(btn=>{
    const n=+btn.dataset.cmd.slice(1);          // M<n>
    const bank=Math.floor((n-1)/4), dir=(n-1)%4;
    btn.title='Controller: hold '+banks[bank]+' + D-pad '+dirs[dir];
  });
})();

// sliders
const vmax=document.getElementById('vmax'), lift=document.getElementById('lift');
const comx=document.getElementById('comx'), comy=document.getElementById('comy');
vmax.oninput=()=>{ maxVx=+vmax.value; maxVy=Math.round(maxVx*0.73);
  document.getElementById('vlab').textContent=vmax.value; };
lift.oninput=()=>{ document.getElementById('klab').textContent=lift.value; };
lift.onchange=()=>cmd('K '+lift.value);
function sendCom(){ cmd('E '+comx.value+' '+comy.value); }
comx.onchange=sendCom; comy.onchange=sendCom;

// --- stand-height calibration ----------------------------------------------
// Slider value = body height (mm); firmware foot Z = -height (more negative
// = legs reach further down = body taller).  On release we set Z then ease
// into the stance (P) so you can see/feel the new height; Save persists it.
const standh=document.getElementById('standh'), standlab=document.getElementById('standlab');
function standZ(){ return -(+standh.value); }
async function applyStand(){ await cmd('Z '+standZ().toFixed(1)); cmd('P'); armed=false;
  forceResend(); showSent('stand height '+standh.value+' mm'); }
standh.oninput=()=>{ standlab.textContent=standh.value; };
standh.onchange=applyStand;
document.getElementById('standup').onclick=()=>{
  standh.value=Math.min(+standh.max,+standh.value+2); standlab.textContent=standh.value; applyStand(); };
document.getElementById('standdn').onclick=()=>{
  standh.value=Math.max(+standh.min,+standh.value-2); standlab.textContent=standh.value; applyStand(); };
document.getElementById('savestand').onclick=async ()=>{
  try{ await fetch('/cal',{method:'POST',body:standZ().toFixed(1)});
       showSent('✓ saved '+standh.value+' mm'); }
  catch(e){ showSent('save failed'); } };
// load the remembered height + sync it to the firmware on page open
fetch('/cal').then(r=>r.json()).then(d=>{
  if(d && d.stand_z!=null){ standh.value=Math.round(-d.stand_z);
    standlab.textContent=standh.value; cmd('Z '+(+d.stand_z).toFixed(1)); }
}).catch(()=>{});

// --- drive loop -------------------------------------------------------------
// Runs at 20 Hz for responsive input, but only SENDS a J packet when the
// command actually changes (plus a slow heartbeat) -- streaming identical
// packets at 20 Hz floods and wedges the MCU<->Linux serial bridge.
function clamp(v,a,b){ return Math.max(a,Math.min(b,v)); }
let lastLine='', lastSendT=0;
function forceResend(){ lastLine=''; }   // call after any discrete command
function loop(){
  const gpv = pollGamepad();
  let x=0,y=0,t=0;
  if(gpv && (gpv.x||gpv.y||gpv.t)){ x=gpv.x; y=gpv.y; t=gpv.t; }
  else {
    const kv=keyVec();
    x = driveStick.x || kv.x;
    y = driveStick.y || kv.y;
    t = turnStick.x  || kv.t;
  }
  if(armed && !dancePaused){
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
</script>
</body>
</html>
"""


class Handler(BaseHTTPRequestHandler):
    protocol_version = "HTTP/1.1"

    def log_message(self, *a):
        pass   # quiet

    def _send(self, code, body, ctype="text/plain; charset=utf-8"):
        data = body.encode("utf-8") if isinstance(body, str) else body
        self.send_response(code)
        self.send_header("Content-Type", ctype)
        self.send_header("Content-Length", str(len(data)))
        self.end_headers()
        try:
            self.wfile.write(data)
        except OSError:
            pass

    def do_GET(self):
        if self.path in ("/", "/index.html"):
            page = PAGE.replace("__HTTPS_PORT__", str(HTTPS_PORT or 8443))
            self._send(200, page, "text/html; charset=utf-8")
        elif self.path == "/cal":
            self._send(200, json.dumps({"stand_z": CAL.get("stand_z")}),
                       "application/json")
        else:
            self._send(404, "not found")

    def do_POST(self):
        n = int(self.headers.get("Content-Length", 0) or 0)
        body = self.rfile.read(n).decode("ascii", "ignore") if n else ""
        if self.path == "/cmd":
            ok = LINK.send(body.strip())
            self._send(200 if ok else 502, "ok" if ok else "link down")
        elif self.path == "/cal":
            # Persist the calibrated standing foot Z (mm) and push it to the
            # firmware so it survives reboots/reflashes (no on-board EEPROM).
            try:
                z = float(body.strip())
            except ValueError:
                self._send(400, "bad value")
                return
            CAL["stand_z"] = z
            saved = save_cal(CAL)
            LINK.send(f"Z {z:.1f}")
            self._send(200 if saved else 500, "saved" if saved else "save failed")
        else:
            self._send(404, "not found")


def main():
    global LINK
    ap = argparse.ArgumentParser(description="Hexapod web control panel")
    ap.add_argument("--host", default="127.0.0.1", help="firmware bridge host")
    ap.add_argument("--port", type=int, default=7500, help="router monitor TCP port")
    ap.add_argument("--bind", default="0.0.0.0", help="HTTP bind address")
    ap.add_argument("--http-port", type=int, default=8080, dest="http_port",
                    help="HTTP port to serve the UI on (default 8080)")
    ap.add_argument("--https-port", type=int, default=443, dest="https_port",
                    help="preferred HTTPS port (needed for gamepad; default 443)")
    args = ap.parse_args()

    LINK = Link(args.host, args.port)
    # Re-assert the remembered standing height so the firmware (which forgets
    # it on reset) comes up at the calibrated stance.
    if CAL.get("stand_z") is not None:
        if LINK.send(f"Z {float(CAL['stand_z']):.1f}"):
            print(f"[cal] applied saved stand Z = {CAL['stand_z']} mm")

    # HTTPS (in a background thread) so the Gamepad API is available.  Prefer
    # 443 so the plain `https://hexapod.local` (no port) works; if we lack the
    # privilege to bind it, fall back to 8443 so the panel still comes up.
    global HTTPS_PORT
    if ensure_cert():
        ctx = ssl.SSLContext(ssl.PROTOCOL_TLS_SERVER)
        ctx.load_cert_chain(CERT_FILE, KEY_FILE)
        for hp in (args.https_port, 8443):
            try:
                httpsd = ThreadingHTTPServer((args.bind, hp), Handler)
                httpsd.socket = ctx.wrap_socket(httpsd.socket, server_side=True)
                threading.Thread(target=httpsd.serve_forever, daemon=True).start()
                HTTPS_PORT = hp
                print(f"[web] gamepad-ready HTTPS on https://{args.bind}:{hp}")
                break
            except PermissionError:
                print(f"[https] no privilege to bind {hp}; trying fallback")
            except OSError as e:
                print(f"[https] could not bind {hp} ({e})")
            if hp == 8443:
                print("[https] HTTPS disabled; HTTP only")

    srv = ThreadingHTTPServer((args.bind, args.http_port), Handler)
    print(f"[web] serving control panel on http://{args.bind}:{args.http_port}  "
          f"(forwarding to firmware {args.host}:{args.port})")
    print("[web] open http://<board-ip>:%d from any device on the LAN" % args.http_port)
    if HTTPS_PORT:
        sfx = "" if HTTPS_PORT == 443 else f":{HTTPS_PORT}"
        print(f"[web] for an Xbox controller use https://<board-ip>{sfx}")
    try:
        srv.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        srv.server_close()


if __name__ == "__main__":
    main()
