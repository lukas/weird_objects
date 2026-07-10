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
  nav.tabs { display:flex; gap:6px; }
  .tab { flex:0 0 auto; min-width:0; padding:7px 16px; font-size:13px; font-weight:600;
         border-radius:8px; background:#161a22; border:1px solid #2c3140; color:#e7eaf0;
         cursor:pointer; }
  .tab.on { background:#2b6cff; border-color:#2b6cff; }
  .view { display:none; } .view.active { display:block; }
  code { background:#222838; border:1px solid #333b52; border-radius:4px; padding:0 4px;
         font-family: ui-monospace, monospace; font-size:11px; }
  .armbar { display:flex; align-items:center; gap:10px; flex-wrap:wrap;
            padding:11px 14px; border-radius:12px; margin-bottom:14px;
            border:1px solid #2c3140; }
  .armbar.disarmed { background:#2a1416; border-color:#7a2b2b; }
  .armbar.armed    { background:#12271a; border-color:#2e7d47; }
  .armstate { font-weight:800; font-size:15px; letter-spacing:.3px; }
  .armbar.disarmed .armstate { color:#ff6b6b; }
  .armbar.armed    .armstate { color:#5fd08a; }
  .armbtn, .estop { flex:0 0 auto; min-width:0; padding:11px 16px; font-weight:700;
            font-size:14px; border-radius:10px; cursor:pointer; }
  .armbtn { background:#2e7d47; border:1px solid #3a9a58; color:#fff; }
  .armbar.armed .armbtn { background:#1b2030; border-color:#2c3140; color:#e7eaf0; }
  .estop { background:#7a2b2b; border:1px solid #a33; color:#fff; }
  .estop:active { background:#5a2020; }
</style>
</head>
<body>
<header>
  <h1>🕷 Hexapod control</h1>
  <nav class="tabs">
    <button id="tab-drive" class="tab on">Drive</button>
    <button id="tab-debug" class="tab">Debug</button>
  </nav>
  <span id="conn" class="ok">ready</span>
  <span id="gp"></span>
  <span style="flex:1"></span>
  <span id="sent"></span>
</header>
<div class="wrap">
  <div id="armbar" class="armbar disarmed">
    <span id="armstate" class="armstate">● SERVOS OFF (disarmed)</span>
    <span class="hint" style="margin:0;flex:1 1 160px">Servos receive <b>no signal</b>
      and sit limp. Press Enable to arm, then Stand.
      <b>Disarm</b> lowers gently first, THEN cuts power; <b>EMERGENCY STOP</b>
      cuts power instantly (drops).</span>
    <button id="armbtn" class="armbtn">Enable servos (power on)</button>
    <button id="estop" class="estop" title="Cut all power to the servos IMMEDIATELY — the robot goes limp NOW and will drop. Use only in an emergency. For a normal, gentle power-off use Disarm / Sit &amp; power off.">■ EMERGENCY STOP</button>
  </div>
  <div id="view-drive" class="view active">
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
        <button id="crouch" title="Gradually pull the legs INWARD into the low, feet-tucked crouch — WITHOUT lifting. Body stays low so you can watch it settle, then press Stand to lift. Slew-limited, tripod-staggered, stays stable (firmware CROUCH).">⤵ Tuck / Crouch</button>
        <button data-cmd="P" title="Controller: A — lift into a stand from wherever it is (folds first, then rises).">▲ Stand</button>
      </div>
      <div class="btns">
        <button id="sit" title="Graceful sit-down: slew-limited, tripod-staggered lower from standing to a low resting pose so the robot settles gently instead of smashing down. Stays powered/held at the bottom — then Disarm to go limp (firmware SIT).">▼ Sit / Lower</button>
        <button data-cmd="U" title="Controller: Y">Legs up</button>
        <button data-cmd="C" title="Controller: X">Center</button>
      </div>
      <button id="stop" class="danger" style="margin-top:10px"
        title="Gently lower to the ground FIRST, then cut power (limp). For an instant drop use EMERGENCY STOP above.">▼ Sit &amp; power off (gentle)</button>

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

      <label class="slab" title="How far the feet pull inward under the body during stand-up — smaller = tucked tighter = easier to lift but watch knee limits.">
        Stand tuck-in radius: <span id="tucklab">130</span> mm</label>
      <input id="tuckr" type="range" min="90" max="200" value="130"
        title="How far the feet pull inward under the body during stand-up — smaller = tucked tighter = easier to lift but watch knee limits.">
      <div class="hint"><b>Stand tuck-in radius</b> = how far the feet pull
        inward under the body while it rises during <b>Stand</b>. <b>Smaller</b>
        = feet tucked tighter (shorter leg lever = easier to lift), <b>larger</b>
        = feet more sprawled. Too tight can hit the knee limit — the firmware
        then relaxes the tuck automatically. Remembered on the robot and
        re-applied on boot.</div>

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
  </div><!-- /view-drive -->

  <div id="view-debug" class="view">
  <div class="row">
    <div class="pad">
      <h2>Per-servo control</h2>
      <label class="slab">Leg</label>
      <div class="btns seg" id="dbgleg">
        <button data-leg="0" class="on">0</button>
        <button data-leg="1">1</button>
        <button data-leg="2">2</button>
        <button data-leg="3">3</button>
        <button data-leg="4">4</button>
        <button data-leg="5">5</button>
      </div>
      <label class="slab">Joint</label>
      <div class="btns seg" id="dbgaxis">
        <button data-axis="0" class="on">Yaw</button>
        <button data-axis="1">Hip</button>
        <button data-axis="2">Knee</button>
      </div>
      <div class="hint">Selected: <b>servo index <span id="dbgidx">0</span></b>
        (leg <span id="dbglegn">0</span> <span id="dbgaxisn">yaw</span>),
        safe range <span id="dbglo">-35</span>..<span id="dbghi">35</span>°.</div>

      <label class="slab">Angle: <span id="dbganglelab">0</span>°</label>
      <input id="dbgangle" type="range" min="-35" max="35" step="1" value="0">
      <div class="btns">
        <button id="dbgneg">◀ −lim</button>
        <button id="dbgdn">− 5°</button>
        <button id="dbgctr" class="on">● center</button>
        <button id="dbgup">+ 5°</button>
        <button id="dbgpos">+lim ▶</button>
      </div>
      <div class="hint">Moves ONLY the selected servo (firmware
        <code>#&lt;index&gt; &lt;deg&gt;</code>), clamped to its safe per-axis
        range. Sends live as you drag (throttled &amp; de-duped like the drive
        joystick).</div>
    </div>

    <div class="pad">
      <h2>Movement test</h2>
      <label class="slab">Wiggle amplitude: <span id="dbgamplab">8</span>°
        &nbsp;<span style="color:#8089a0">(small = safe)</span></label>
      <input id="dbgamp" type="range" min="3" max="20" step="1" value="8">
      <div class="btns">
        <button id="dbgtestone">Test this servo</button>
        <button id="dbgtestall" class="on">Test ALL in sequence</button>
        <button id="dbgteststop" class="danger">■ Stop test</button>
      </div>
      <div class="hint" id="dbgteststatus">Idle. <b>Test this servo</b> wiggles
        the selected joint ±amplitude around its current angle (firmware
        <code>Q&nbsp;&lt;j&gt;&nbsp;&lt;deg&gt;</code>). <b>Test ALL</b> centers
        every joint, then wiggles each one in turn (0→17) with a pause between,
        so you can watch each servo respond. Press <b>Stop test</b> to abort.</div>

      <h2 style="margin-top:18px">Safety</h2>
      <div class="btns">
        <button id="dbgstand" class="on">Stand / Park</button>
        <button id="dbgcenter">Center all</button>
        <button id="dbgrelax" class="danger">Relax (limp)</button>
      </div>
      <div class="hint"><b>Relax</b> cuts PWM <i>instantly</i> so every servo goes
        limp NOW (firmware <code>X</code>) — use it to recover a stalled/buzzing
        servo. For a <i>gentle</i> power-off that lowers the robot to the ground
        first, use <b>Sit &amp; power off</b> / <b>Disarm</b> on the Drive tab.
        <b>Stand / Park</b> eases into the planted stance (<code>P</code>);
        <b>Center all</b> sends every joint to 0° (<code>C</code>).</div>
    </div>
  </div>
  </div><!-- /view-debug -->
</div>
<script>
const conn = document.getElementById('conn');
const sentEl = document.getElementById('sent');
const gpEl = document.getElementById('gp');
let gait = 0, armed = false, dancePaused = false, lastInput = 0, debugMode = false;
// SERVO ARM GATE (separate from `armed`, which just means "a stick is pushed").
// Defaults OFF on every page load and the firmware boots DISARMED, so nothing
// drives a servo until the human presses Enable. All servo-driving sends are
// gated on this flag; ARM/DISARM/E-stop are the only power controls.
let servosArmed = false;
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
function doDance(c){ if(needArm()) return; cmd(c); dancePaused=true; armed=true; forceResend();
  showSent('dance '+c+' (move to resume)'); }
// A plain "Relax/off" (X) means "come down gracefully then go limp" -- route it
// through settleServos(). The true instant limp is EMERGENCY STOP (disarmServos).
function disc(c){ if(c==='X'){ settleServos(); return; }
  if(needArm()) return; cmd(c); forceResend(); if('PU'.includes(c)) armed=false; }

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
// Pre-lift tuck: gradually pull the legs into the low crouch (no lift).
document.getElementById('crouch').onclick=()=>{ if(needArm()) return;
  cmd('CROUCH'); forceResend(); armed=false;
  showSent('CROUCH — tucking legs in low (then press Stand to lift)'); };
// Graceful sit-down: slew-limited lower to a low resting pose (stays held).
document.getElementById('sit').onclick=()=>{ if(needArm()) return;
  cmd('SIT'); forceResend(); armed=false;
  showSent('SIT — lowering gently to a low rest'); };
// "Sit & power off": graceful lower THEN cut power (NOT the instant e-stop).
document.getElementById('stop').onclick=settleServos;

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
async function applyStand(){ await cmd('Z '+standZ().toFixed(1));   // Z = calibration only, no motion
  if(!servosArmed){ showSent('height stored — enable servos to move'); return; }
  cmd('P'); armed=false; forceResend(); showSent('stand height '+standh.value+' mm'); }
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
// --- stand-up tuck-in radius ------------------------------------------------
// Rise tuck radius (mm) sent to the firmware with `$ <mm>` (config only, no
// motion). Low-frequency, so sent + persisted on change (mirrors stand height).
const tuckr=document.getElementById('tuckr'), tucklab=document.getElementById('tucklab');
tuckr.oninput=()=>{ tucklab.textContent=tuckr.value; };
tuckr.onchange=async ()=>{ cmd('$ '+(+tuckr.value).toFixed(1));   // $ = tuck radius, no motion
  try{ await fetch('/cal_tuck',{method:'POST',body:(+tuckr.value).toFixed(1)}); }catch(e){}
  showSent('stand tuck-in '+tuckr.value+' mm'); };

// load the remembered height + tuck radius and sync them to the firmware on open
fetch('/cal').then(r=>r.json()).then(d=>{
  if(d && d.stand_z!=null){ standh.value=Math.round(-d.stand_z);
    standlab.textContent=standh.value; cmd('Z '+(+d.stand_z).toFixed(1)); }
  if(d && d.tuck_r!=null){ tuckr.value=Math.round(d.tuck_r);
    tucklab.textContent=tuckr.value; cmd('$ '+(+d.tuck_r).toFixed(1)); }
}).catch(()=>{});

// --- drive loop -------------------------------------------------------------
// Runs at 20 Hz for responsive input, but only SENDS a J packet when the
// command actually changes (plus a slow heartbeat) -- streaming identical
// packets at 20 Hz floods and wedges the MCU<->Linux serial bridge.
function clamp(v,a,b){ return Math.max(a,Math.min(b,v)); }
let lastLine='', lastSendT=0;
function forceResend(){ lastLine=''; }   // call after any discrete command
function loop(){
  if(debugMode) return;   // Debug tab active: never stream drive/J packets
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

$('dbgstand').onclick  = ()=>{ if(needArm()) return; cmd('P'); showSent('P'); armed=false; };
$('dbgcenter').onclick = ()=>{ if(needArm()) return; cmd('C'); showSent('C'); armed=false; };
$('dbgrelax').onclick  = disarmServos;   // Relax (limp) = disarm / e-stop, always allowed

// --- tab switching ----------------------------------------------------------
function showView(which){
  debugMode = (which === 'debug');
  $('view-drive').classList.toggle('active', !debugMode);
  $('view-debug').classList.toggle('active', debugMode);
  $('tab-drive').classList.toggle('on', !debugMode);
  $('tab-debug').classList.toggle('on', debugMode);
  if(debugMode) armed = false;   // stop the drive loop streaming J packets
}
$('tab-drive').onclick = ()=> showView('drive');
$('tab-debug').onclick = ()=> showView('debug');
dbgRefresh();
// deep link: /debug (or #debug) opens straight to the Debug tab.
if(location.pathname.replace(/\/+$/,'').endsWith('/debug') || location.hash === '#debug')
  showView('debug');

// --- SERVO ARM / DISARM gate ------------------------------------------------
// The firmware boots DISARMED (all PCA9685 channels forced full-off = no PWM,
// so every servo is limp). This page also defaults to disarmed on EVERY load
// and NEVER auto-arms. "Enable servos" sends ARM; the big red EMERGENCY STOP,
// the drive STOP, and Debug's Relax all send DISARM (firmware `X`), which cuts
// all PWM. needArm() gates every servo-driving send on both pages.
function updateArmUI(){
  const bar = $('armbar');
  bar.classList.toggle('armed', servosArmed);
  bar.classList.toggle('disarmed', !servosArmed);
  $('armstate').textContent = servosArmed ? '● ARMED — servos live'
                                          : '● SERVOS OFF (disarmed)';
  $('armbtn').textContent   = servosArmed ? 'Disarm (servos off)'
                                          : 'Enable servos (power on)';
}
function setArmed(on){ servosArmed = on; if(!on) armed = false; updateArmUI(); }
function armServos(){ cmd('ARM'); setArmed(true);
  showSent('ARM — servos enabled (nothing moves; press Stand to stand)'); }
// GRACEFUL power-off: lower to the ground first (firmware SETTLE = SIT then
// DISARM), only THEN cut power. This is the NORMAL disarm/relax/off path so the
// robot settles instead of collapsing. UI shows disarmed once the command is
// sent (the firmware does the lower, then goes limp on its own).
function settleServos(){ dbgTestAbort = true; cmd('SETTLE'); setArmed(false);
  showSent('DISARM — lowering gently, then servos off'); }
// INSTANT limp: cut all PWM NOW (true emergency stop; the robot drops). Always
// allowed, even while disarmed, and used for the boot-time safe default.
function disarmServos(){ dbgTestAbort = true; cmd('X'); setArmed(false);
  showSent('EMERGENCY STOP — servos limp NOW'); }
// Returns true (and warns) when disarmed; every servo-driving action calls it.
function needArm(){
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
        if self.path in ("/", "/index.html", "/debug"):
            page = PAGE.replace("__HTTPS_PORT__", str(HTTPS_PORT or 8443))
            self._send(200, page, "text/html; charset=utf-8")
        elif self.path == "/cal":
            self._send(200, json.dumps({"stand_z": CAL.get("stand_z"),
                                        "tuck_r": CAL.get("tuck_r")}),
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
        elif self.path == "/cal_tuck":
            # Persist the stand-up rise tuck-in radius (mm) and push it to the
            # firmware so it survives reboots/reflashes (no on-board EEPROM).
            try:
                r = float(body.strip())
            except ValueError:
                self._send(400, "bad value")
                return
            CAL["tuck_r"] = r
            saved = save_cal(CAL)
            LINK.send(f"$ {r:.1f}")
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
    if CAL.get("tuck_r") is not None:
        if LINK.send(f"$ {float(CAL['tuck_r']):.1f}"):
            print(f"[cal] applied saved stand tuck-in radius = {CAL['tuck_r']} mm")

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
