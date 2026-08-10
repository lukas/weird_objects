#!/usr/bin/env python3
"""
Web control panel for the STS3215 hexapod.

Runs on the Uno Q Linux side (or a laptop) and serves a drive UI over
HTTP/HTTPS.  Open from your laptop — on-screen sticks, keyboard, or an Xbox
controller plugged into the laptop (browser Gamepad API needs HTTPS).

Commands go to ``DriveController`` (TripodGait → Feetech bus), not the old
v1 STM32 ``J`` bridge.

  python3 web_drive.py                 # :8080 + https :8443
  python3 web_drive.py --dry-run       # UI only (no bus)
  python3 web_drive.py --port /dev/ttyUSB0
"""

from __future__ import annotations

import argparse
import json
import os
import ssl
import subprocess
import sys
import threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

HERE = Path(__file__).resolve().parent
if str(HERE) not in sys.path:
    sys.path.insert(0, str(HERE))

from bench_api import BenchAPI  # noqa: E402
from drive_controller import DriveController  # noqa: E402

# Self-signed TLS material for the HTTPS listener (generated on first run).
CERT_FILE = os.path.expanduser("~/.hexapod_sts_cert.pem")
KEY_FILE = os.path.expanduser("~/.hexapod_sts_key.pem")


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


class Link:
    """Thread-safe bridge from HTTP /cmd into DriveController.handle()."""

    def __init__(self, drive: DriveController):
        self.drive = drive
        self.lock = threading.Lock()

    def send(self, line: str) -> bool:
        with self.lock:
            try:
                try:
                    from event_log import emit
                    emit("cmd", line.strip(), src="drive",
                         data={"line": line.strip()})
                except Exception:
                    pass
                self.drive.handle(line)
                return True
            except Exception as e:
                print(f"[link] handle failed: {e}")
                return False


LINK = None   # set in main()
DRIVE = None
BENCH = None
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
  #conn { font-size:12px; color:#9aa3b2; font-weight:600; }
  #conn.bad { color:#ff6b6b; } #conn.ok { color:#5fd08a; }
  #conn.warn { color:#e6b35a; }
  #robotact { font-size:12px; font-weight:700; padding:3px 10px; border-radius:999px;
              border:1px solid #333b52; background:#222838; color:#c5cce0;
              max-width:280px; overflow:hidden; text-overflow:ellipsis; white-space:nowrap; }
  #robotact.act-demo { color:#7eb6ff; border-color:#2b6cff; }
  #robotact.act-calibrating { color:#c9a0ff; border-color:#6b4aa0; }
  #robotact.act-zeroing { color:#e6b35a; border-color:#8a6a2a; }
  #robotact.act-stopping { color:#ff6b6b; border-color:#7a2b2b; }
  #robotact.act-limp { color:#9aa3b2; }
  #robotact.act-armed { color:#5fd08a; border-color:#2e7d47; }
  #zerohint.bad { color:#ff8a8a; } #zerohint.ok { color:#5fd08a; }
  #offlinebar { display:none; margin:0 0 14px; padding:12px 14px; border-radius:12px;
                background:#3a1818; border:1px solid #a33; color:#ffb4b4;
                font-size:14px; font-weight:700; letter-spacing:.2px; }
  body.link-down #offlinebar { display:block; }
  body.link-down header { border-bottom-color:#7a2b2b; }
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
  .btns.nowrap { flex-wrap:nowrap; gap:6px; }
  .btns.nowrap button { flex:1 1 0; min-width:0; padding:10px 6px; font-size:12px;
                        white-space:nowrap; overflow:hidden; text-overflow:ellipsis; }
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
  table.motors { width:100%; border-collapse:collapse; font-size:12px; }
  table.motors th, table.motors td { padding:6px 8px; border-bottom:1px solid #232733;
    text-align:left; }
  table.motors th { color:#9aa3b2; font-weight:600; }
  table.motors tr.alarm td { color:#ff8a8a; }
  table.motors tr.sel td { background:#1b2744; }
  table.motors tr { cursor:pointer; }
  .pill { display:inline-block; padding:2px 8px; border-radius:999px; font-size:11px;
    background:#222838; border:1px solid #333b52; }
  .pill.ok { color:#5fd08a; border-color:#2e7d47; }
  .pill.bad { color:#ff6b6b; border-color:#7a2b2b; }
  .demo-grid { display:grid; grid-template-columns:repeat(auto-fill,minmax(180px,1fr)); gap:8px; }
  .demo-grid button { text-align:left; }
  .demo-grid button.previewing { outline:1px solid #2b6cff; background:#1b2744; }
  .demo-prev { display:flex; gap:14px; align-items:stretch; flex-wrap:wrap;
               margin:12px 0; padding:12px; border-radius:12px;
               background:#0c0e13; border:1px solid #232733; }
  .demo-prev canvas { width:220px; height:160px; border-radius:8px;
                      background:#0a0c10; border:1px solid #1e2433; }
  .demo-prev .prev-meta { flex:1 1 180px; min-width:160px; }
  .demo-prev .prev-meta h3 { margin:0 0 6px; font-size:14px; color:#e7eaf0; }
  .demo-prev .prev-meta .tag { display:inline-block; font-size:11px; font-weight:700;
    padding:2px 8px; border-radius:999px; margin-right:6px; margin-bottom:6px;
    border:1px solid #333b52; color:#9aa3b2; }
  .demo-prev .prev-meta .tag.sit { color:#7eb6ff; border-color:#2b6cff; }
  .demo-prev .prev-meta .tag.stand { color:#5fd08a; border-color:#2e7d47; }
  .demo-prev .prev-meta p { margin:0; font-size:12px; color:#9aa3b2; line-height:1.45; }
  .live-wrap { display:flex; gap:14px; flex-wrap:wrap; align-items:stretch; }
  .live-wrap .pad.live-main { flex:2 1 420px; }
  .live-wrap .pad.live-side { flex:1 1 220px; }
  #livecv { width:100%; height:420px; display:block; border-radius:10px;
            background:#0a0c10; cursor:grab; touch-action:none; }
  #livecv.dragging { cursor:grabbing; }
  .live-angles { width:100%; border-collapse:collapse; font-size:11px; font-variant-numeric:tabular-nums; }
  .live-angles th, .live-angles td { padding:4px 6px; border-bottom:1px solid #232733; text-align:right; }
  .live-angles th:first-child, .live-angles td:first-child { text-align:left; }
  .live-angles th { color:#9aa3b2; font-weight:600; }
  .live-angles tr.off td { color:#5a6478; }
  .live-angles tr.hi td { color:#7eb6ff; }
  .cal-table { width:100%; border-collapse:collapse; font-size:12px;
    font-variant-numeric:tabular-nums; }
  .cal-table th, .cal-table td { padding:5px 7px; border-bottom:1px solid #232733;
    text-align:right; }
  .cal-table th:first-child, .cal-table td:first-child,
  .cal-table th:nth-child(2), .cal-table td:nth-child(2) { text-align:left; }
  .cal-table th { color:#9aa3b2; font-weight:600; }
  .cal-table tr.g-green td { color:#5fd08a; }
  .cal-table tr.g-yellow td { color:#e0b34d; }
  .cal-table tr.g-red td { color:#ff7b72; }
  .cal-pill { display:inline-block; padding:2px 8px; border-radius:999px;
    font-size:11px; font-weight:700; border:1px solid #333; }
  .cal-pill.green { color:#5fd08a; border-color:#2e7d47; }
  .cal-pill.yellow { color:#e0b34d; border-color:#8a6a1a; }
  .cal-pill.red { color:#ff7b72; border-color:#8a3030; }
</style>
</head>
<body>
<header>
  <h1>Hexapod STS3215</h1>
  <nav class="tabs">
    <button id="tab-drive" class="tab on">Drive</button>
    <button id="tab-live" class="tab">Live</button>
    <button id="tab-motors" class="tab">Motors</button>
    <button id="tab-demos" class="tab">Demos</button>
    <button id="tab-rl" class="tab">RL</button>
    <button id="tab-calibrate" class="tab">Calibrate</button>
    <button id="tab-debug" class="tab">Debug</button>
  </nav>
  <span id="conn" class="warn">checking…</span>
  <span id="robotact" class="act-limp" title="What the robot is trying to do">limp</span>
  <span id="gp"></span>
  <span style="flex:1"></span>
  <span id="sent"></span>
</header>
<div class="wrap">
  <div id="offlinebar">● Lost connection to Uno Q — retrying…</div>
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
      <h2>Gait</h2>
      <div class="btns seg" id="gaits">
        <button data-gait="0" class="on">Tripod</button>
      </div>
      <h2 style="margin-top:14px">Pose</h2>
      <div class="btns">
        <button data-cmd="P" title="Controller: Y — planted stand (hip +20°, knee +80°, or learned plant).">▲ Stand</button>
        <button data-cmd="C" title="Controller: X — sit zero (legs out).">Center / Sit</button>
      </div>
      <button id="stop" class="danger" style="margin-top:10px"
        title="Gently lower, then limp. Instant drop = EMERGENCY STOP.">▼ Sit &amp; power off (gentle)</button>

      <label class="slab">Max speed: <span id="vlab">40</span> mm/s</label>
      <input id="vmax" type="range" min="10" max="100" value="40">

      <label class="slab">Swing lift: <span id="klab">18</span> mm</label>
      <input id="lift" type="range" min="4" max="40" value="18">

      <div class="hint">
        STS3215 open-loop tripod gait on the Feetech bus.<br>
        <b>Keyboard:</b> <kbd>W</kbd><kbd>A</kbd><kbd>S</kbd><kbd>D</kbd> drive,
        <kbd>Q</kbd>/<kbd>E</kbd> turn, <kbd>Space</kbd> stand.<br>
        <b>Xbox (https URL):</b> left stick walk · right stick turn.
        Alone: <b>X</b>=sit · <b>Y</b>=stand · <b>A</b>=set HERE as zero · <b>B</b>=stop demo.
        Demos: hold <b>LB / LT / RB / RT</b> then tap <b>X Y A B</b> (16 chords;
        LB row easiest → RT row hardest). Enable servos first.
      </div>
    </div>
  </div>
  </div><!-- /view-drive -->

  <div id="view-live" class="view">
    <div class="live-wrap">
      <div class="pad live-main">
        <h2>Live schematic</h2>
        <div class="hint" style="margin-top:0;margin-bottom:8px">
          Forward kinematics from motor feedback (coxa 12.5 / femur 90 / tibia 128 mm).
          Drag to orbit · scroll to zoom · updates ~8 Hz.
          <span id="livestamp" style="color:#5a6478">—</span>
        </div>
        <canvas id="livecv" width="880" height="520"></canvas>
        <div class="btns" style="margin-top:10px">
          <button id="livereset">Reset view</button>
          <button id="livepause">Pause</button>
          <button id="liverefresh">↻ Now</button>
        </div>
        <div class="hint" id="livehint">Waiting for pose…</div>
      </div>
      <div class="pad live-side">
        <h2>Joint angles</h2>
        <div class="hint" style="margin:0 0 8px">
          Live <b id="livencount">0</b>/18 ·
          <span id="livemode" class="pill">—</span>
        </div>
        <table class="live-angles" id="livetab">
          <thead><tr><th>Leg</th><th>Yaw</th><th>Hip</th><th>Knee</th></tr></thead>
          <tbody id="livebody"></tbody>
        </table>
        <div class="hint" style="margin-top:10px">
          Sit zero = all ~0° (legs out). Stand zero ≈ hip +20° / knee +80° (femur down, tibia steep).
          Offline joints shown dashed / grey.
        </div>
      </div>
    </div>
  </div><!-- /view-live -->

  <div id="view-motors" class="view">
  <div class="row">
    <div class="pad" style="flex:2 1 420px">
      <h2>Bus status</h2>
      <div class="hint" style="margin-bottom:8px">
        Port <code id="mport">—</code>
        · <span id="mmode" class="pill">—</span>
        · live <b id="mlive">0</b>
        · <span id="mrefresh" class="hint" style="margin:0">auto 1.5s</span>
      </div>
      <div class="btns nowrap">
        <button id="mscan" class="on">↻ Refresh</button>
        <button id="mwiggle">Wiggle selected</button>
        <button id="mzero">Go to zero (slow)</button>
        <button id="msetzero" class="danger">Set HERE as zero</button>
        <button id="mlimp" class="danger">Limp all</button>
      </div>
      <div class="hint" style="margin-top:6px">
        <b>Set HERE as zero</b> = Feetech middle-calibrate on every live servo
        (IDs 2..19). Horns do <i>not</i> turn — limp, hand-pose first, then click.
        Different from <b>Go to zero</b>, which drives joints to logical 0°.
      </div>
      <div style="overflow:auto; margin-top:10px; max-height:420px">
        <table class="motors" id="mtab">
          <thead><tr>
            <th>ID</th><th>Name</th><th>°</th><th>Load</th><th>I (A)</th>
            <th>V</th><th>°C</th><th>Flags</th>
          </tr></thead>
          <tbody id="mbody"><tr><td colspan="8">Loading…</td></tr></tbody>
        </table>
      </div>
    </div>
    <div class="pad" style="flex:1 1 220px">
      <h2>Selected</h2>
      <div class="hint">Click a row. Wiggle uses a small ±amp around the present angle (same as setup test).</div>
      <label class="slab">Joint index: <span id="mseljoint">—</span></label>
      <label class="slab">Wiggle amp: <span id="mwamplab">6</span>°</label>
      <input id="mwamp" type="range" min="2" max="20" value="6">
      <div class="hint" id="mselinfo">No motor selected.</div>
      <div class="hint" style="margin-top:14px" id="merr"></div>
    </div>
  </div>
  </div><!-- /view-motors -->

  <div id="view-demos" class="view">
  <div class="row">
    <div class="pad">
      <h2>In-place demos</h2>
      <div class="hint">Same list as <code>inplace_demos.py</code> / motor_setup
        <b>f</b>, sorted gentle → spicy. Soft air demos first; then planted
        moves (<b>plant_*</b> / <b>rise_*</b>). Cords OK for planted.<br>
        <b>Pad:</b> hold LB/LT/RB/RT then tap X·Y·A·B —
        LB: breathe heartbeat twinkle shimmy ·
        LT: ripple conductor arms_up <b>walk</b> ·
        RB: <b>walk_spin</b> bounce gallop tripod ·
        RT: <b>walk_oval</b> star stomp rise_show.</div>
      <div class="btns" style="margin:10px 0">
        <button id="dstop" class="danger">■ Stop demo</button>
        <button id="dzero">Sit zero</button>
        <button id="dstand">Stand zero</button>
        <button id="dcheckz">Check sit zero</button>
      </div>
      <div class="hint">Homes: <b>sit zero</b> = legs out (0°) for air demos;
        <b>stand zero</b> = plant height (Calibrate → Plant height, else +20°/+80°).
        Planted demos end at stand zero and stay up.</div>
      <label class="slab">Demo speed: <span id="dspeedlab">1.0</span>×
        <span style="color:#8089a0;font-weight:400">(all demos · 0.25× → 2×)</span></label>
      <input id="dspeed" type="range" min="25" max="200" value="100" step="5">
      <label class="slab">Air torque limit: <span id="dtorquelab">45</span>%
        <span style="color:#8089a0;font-weight:400">(lower = less shake/hunt · air demos)</span></label>
      <input id="dtorque" type="range" min="15" max="100" value="45" step="5">
      <div class="hint" style="margin-top:12px;color:#9aa3b2;font-weight:600;text-transform:uppercase;letter-spacing:.4px;font-size:11px">Breathe tunables <span style="font-weight:400;normal-case;letter-spacing:0;text-transform:none">(breathe + breathe_v)</span></div>
      <label class="slab">Size: <span id="dsizelab">1.0</span>×
        <span style="color:#8089a0;font-weight:400">(amplitude 0.5× → 3×)</span></label>
      <input id="dsize" type="range" min="50" max="300" value="100" step="10">
      <label class="slab">Rate: <span id="dratelab">0.20</span> Hz
        <span style="color:#8089a0;font-weight:400">(breaths/sec · slower often smoother)</span></label>
      <input id="drate" type="range" min="8" max="60" value="20" step="1">
      <label class="slab">Softness: <span id="dsoftlab">1.5</span>×
        <span style="color:#8089a0;font-weight:400">(higher = gentler accel, less bark)</span></label>
      <input id="dsoft" type="range" min="50" max="300" value="150" step="10">
      <div class="hint">Status: <span id="dstatus" class="pill">idle</span>
        <span id="dstatusdetail" style="color:#8089a0"></span></div>
      <div class="hint" id="dtelem" style="margin-top:6px;color:#5a6478">
        Demos auto-log cmd vs encoder → <code>logs/demo_*.csv</code> (+ summary).</div>
      <div class="hint" id="zerohint">Click any demo anytime — it aborts the current one, homes to sit (air) or stand (planted), then starts. Switching mid-demo is OK.</div>
      <div class="demo-prev" id="dprev">
        <canvas id="dprevcv" width="440" height="320"></canvas>
        <div class="prev-meta">
          <h3 id="dprevtitle">Hover a demo</h3>
          <div id="dprevtags"></div>
          <p id="dprevblurb">Schematic preview of the expected start pose and motion
            (not a physics sim). Hover a card to scrub the idea; click to run.</p>
        </div>
      </div>
      <div class="demo-grid" id="dgrid" style="margin-top:12px"></div>
    </div>
  </div>
  </div><!-- /view-demos -->

  <div id="view-rl" class="view">
  <div class="row">
    <div class="pad" style="flex:1 1 340px">
      <h2>Policy moves</h2>
      <div class="hint"><b>Stand up</b>: place the robot <b>belly-down,
        legs straight out</b> (logical zero). It curls its feet under
        for ~5&nbsp;s (this is deliberate, not a stall), then rises over
        ~4&nbsp;s and holds. <b>Lower</b>: from the standing plant
        stance it descends gently over ~5&nbsp;s to the belly, then goes
        limp. <b>Stop</b> holds the pose; <b>X</b> (Drive tab) limps.
        <b>Watch the robot the whole time.</b></div>
      <div class="btns" style="margin-top:12px">
        <button id="rlstand" class="on">⬆ Stand up</button>
        <button id="rllower">⬇ Lower</button>
        <button id="rlglide" title="Scripted 4.5 s glide to the captured
plant stance (not RL — same as Drive tab ▲ Stand). Refuses if any joint
is >25° away. Use it to square up before Capture plant or Walk.">▲ Stand
(scripted)</button>
        <button id="rlcapture" title="Save the current 18-joint pose as
the plant stance (no motion). Lower and Walk start only from this pose —
capture while the robot is standing well.">📌 Capture plant</button>
        <button id="rlstop" class="danger">■ Stop</button>
      </div>
      <div class="hint" id="rlstatus" style="margin-top:10px">Idle.</div>
      <h2 style="margin-top:16px">Walk (experimental)</h2>
      <div class="hint">Sim walk champion — <b>NOT validated on
        hardware</b> (known foot-slide in sim; no body-velocity sensor,
        command runs open-loop). Starts <b>only from the captured plant
        stance</b>. Command ramps in over 1&nbsp;s, walks, ramps out and
        holds. <b>Stop</b> aborts and holds. Keep a hand near the 12 V
        switch the first tries.</div>
      <div class="btns" style="margin-top:12px">
        <button id="rlwalkfwd">▲ Walk fwd</button>
        <button id="rlwalkleft">◀ Strafe L</button>
        <button id="rlwalkright">▶ Strafe R</button>
      </div>
      <div class="btns" style="margin-top:8px">
        <label class="hint">speed
          <select id="rlwalkspeed">
            <option value="0.02">0.02 m/s</option>
            <option value="0.03" selected>0.03 m/s</option>
            <option value="0.04">0.04 m/s</option>
          </select></label>
        <label class="hint">for
          <select id="rlwalkdur">
            <option value="4">4 s</option>
            <option value="6" selected>6 s</option>
            <option value="10">10 s</option>
          </select></label>
      </div>
    </div>
    <div class="pad" style="flex:1 1 340px">
      <h2>Readiness (no motion)</h2>
      <div class="hint">Runs the same read-only preflight the buttons
        use: all 18 servo IDs answering, IMU alive, tilt &lt; 12°, and
        the present pose near the expected start. Refusals show the
        exact reason.</div>
      <div class="btns" style="margin-top:12px">
        <button id="rlcheckstand">Check: stand</button>
        <button id="rlchecklower">Check: lower</button>
        <button id="rlcheckwalk">Check: walk</button>
      </div>
      <div class="hint" id="rlpreflight" style="margin-top:10px">—</div>
    </div>
    <div class="pad" style="flex:1 1 340px">
      <h2>Policy</h2>
      <div class="hint" id="rlpolicyinfo">loading…</div>
      <div class="hint" style="margin-top:10px">Trained in the MuJoCo
        twin (PPO, raw 18-joint actions, domain randomization) and
        exported to plain numpy — no torch on the board. Every command
        passes the safety layer: 1.5°/tick rate clamp, joint limits,
        10° tilt trip, sustained &gt;2.5&nbsp;A trip → instant limp.
        Swap a policy by re-running
        <code>rl_move/sim/export_policy_np.py</code> and pushing
        <code>rl_policy_weights.json</code> (stance) /
        <code>rl_walk_weights.json</code> (walk).</div>
    </div>
  </div>
  </div><!-- /view-rl -->

  <div id="view-calibrate" class="view">
  <div class="row">
    <div class="pad" style="flex:1 1 320px">
      <h2>Calibrate</h2>
      <label class="slab">Mode</label>
      <div class="btns seg" id="calmode">
        <button data-mode="step" class="on">Step</button>
        <button data-mode="shake">Shake / hold</button>
        <button data-mode="plant">Plant height</button>
        <button data-mode="geometry">Geo plant (hip0/knee≈90)</button>
        <button data-mode="imu">IMU rest</button>
      </div>
      <div class="hint" id="calhint-step">Glides to <b>sit zero</b>, then
        <b>+step°</b> from 0° and scores tracking % + peak load.
        Stuck / weak joints show red.</div>
      <div class="hint" id="calhint-shake" style="display:none">Glides to
        <b>sit zero</b>, nudges <b>+N°</b>, then <b>holds</b> and measures
        position wobble (peak-to-peak / RMS). Catches hunt &amp; buzz that
        a big step misses.</div>
      <div class="hint" id="calhint-plant" style="display:none">From sit zero,
        slowly reaches down (rise+ depth). When ≥3 hip/knee joints hit a
        <b>current/load bump</b>, saves median hip/knee as <b>stand home</b>
        for Stand zero + planted demos. Robot should be over its platform.</div>
      <div class="hint" id="calhint-geometry" style="display:none">
        <b>Preferred for RL.</b> Hip stays ~0° (straight), knees deepen toward
        +80° (~90°) until force/current contact, then raises ~40&nbsp;mm and
        saves full 18-joint plant. Same as <code>POST /api/rl/find_plant</code>.
        Watch the robot — Stop if it tips.</div>
      <div class="hint" id="calhint-imu" style="display:none">Hold the chassis
        <b>still</b> (~2.5 s). Samples the MPU-6050, saves gyro bias + accel
        sensor bias to <code>logs/imu_calib.json</code>. Motion logs then use
        calibrated readings. No servo motion — limp is fine.</div>
      <div id="calstepwrap">
        <label class="slab">Step size: <span id="calsteplab">10</span>°</label>
        <input id="calstep" type="range" min="5" max="15" step="1" value="10">
      </div>
      <div id="calnudgewrap" style="display:none">
        <label class="slab">Nudge: <span id="calnudgelab">2.0</span>°</label>
        <input id="calnudge" type="range" min="1" max="4" step="0.5" value="2">
      </div>
      <div id="calaxiswrap">
        <label class="slab">Axis</label>
        <div class="btns seg" id="calaxis">
          <button data-axis="all" class="on">All</button>
          <button data-axis="yaw">Yaw</button>
          <button data-axis="hip">Hip</button>
          <button data-axis="knee">Knee</button>
        </div>
      </div>
      <div class="hint" id="calplantinfo" style="margin-top:10px;display:none">
        Stand home: <b id="calplanthip">+20</b>° hip /
        <b id="calplantknee">+80</b>° knee
        <span id="calplantsrc" style="color:#8089a0">(default)</span>
      </div>
      <div class="hint" id="calimuinfo" style="margin-top:10px;display:none">
        IMU: <b id="calimustatus">not calibrated</b>
        <span id="calimudetail" style="color:#8089a0"></span>
      </div>
      <div class="btns" style="margin-top:12px">
        <button id="calrun" class="on">▶ Run test</button>
        <button id="calstop" class="danger">■ Stop</button>
        <button id="calrefresh">↻ Refresh</button>
        <button id="calplantreset" style="display:none">Reset plant default</button>
        <button id="calimureset" style="display:none">Clear IMU calib</button>
      </div>
      <div class="hint" id="calstatus" style="margin-top:10px">Idle. Enable servos first.</div>
      <div class="hint" id="callog" style="margin-top:6px;color:#5a6478">Log: —</div>
      <div class="hint" id="callegend" style="margin-top:12px">
        <span class="cal-pill green">green</span> tracked ·
        <span class="cal-pill yellow">yellow</span> partial / high load ·
        <span class="cal-pill red">red</span> barely moved
      </div>
    </div>
    <div class="pad" style="flex:2 1 420px">
      <h2>Results</h2>
      <div class="hint" id="calcounts" style="margin:0 0 8px">No run yet.</div>
      <div style="overflow:auto; max-height:480px">
        <table class="cal-table" id="caltab">
          <thead id="calhead"><tr>
            <th>Joint</th><th>Name</th><th>Δcmd</th><th>Δact</th>
            <th>Track%</th><th>Load</th><th>Vmin</th><th>Grade</th>
          </tr></thead>
          <tbody id="calbody"><tr><td colspan="8">Run a test to fill this table.</td></tr></tbody>
        </table>
      </div>
    </div>
  </div>
  </div><!-- /view-calibrate -->

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
let gait = 0, armed = false, dancePaused = false, lastInput = 0;
let activeView = 'drive';  // drive | live | motors | demos | calibrate | debug
let calAxis = 'all';
let calTimer = null;
let selJoint = null, selSid = null;
let motorsTimer = null;
let linkOk = null;           // null=unknown, true/false after first ping
let linkFailStreak = 0;
let lastPingOkAt = 0;
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
    if(bar) bar.textContent = '● Lost connection to Uno Q — retrying…'
      +(detail ? (' ('+detail+')') : '');
  }
}
async function heartbeat(){
  const ac = new AbortController();
  const t = setTimeout(()=> ac.abort(), 2000);
  try{
    const r = await fetch('/api/ping?t='+Date.now(), {
      cache:'no-store', signal: ac.signal});
    clearTimeout(t);
    if(!r.ok) throw new Error('HTTP '+r.status);
    const j = await r.json().catch(()=>({}));
    if(j && j.ok === false) throw new Error(j.error || 'ping failed');
    setLink(true);
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
setInterval(heartbeat, 1500);
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
  const p = __HTTPS_PORT__;
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
    else showSent(j.error||('demo '+name+' failed'));
    if(j.demo) paintDemoStatus(j.demo);
    if(j.robot) paintRobotActivity(j.robot);
    startDemoPoll();
    refreshRobotState(true);
  }catch(e){ showSent('demo '+name+' failed'); }
}
async function goPoseZero(pose, label){
  // Bench /api/zero arms the bus itself and glides (Drive `P` was a silent
  // one-shot that looked like a no-op when disarmed / already near plant).
  dancePaused = true;
  if(pose === 'stand') armed = false;
  const tag = label || pose;
  showSent(tag + '…');
  try{
    const r = await fetch('/api/zero',{method:'POST',
      headers:{'Content-Type':'application/json'},
      body: JSON.stringify({pose: pose})});
    const j = await r.json();
    if(!j.ok){
      showSent(tag + ' failed: '+(j.error||'unknown'));
      if(j.demo) paintDemoStatus(j.demo);
      if(j.robot) paintRobotActivity(j.robot);
      return;
    }
    setArmed(true);
    showSent(tag + ' — gliding (full torque)');
    if(j.demo) paintDemoStatus(j.demo);
    if(j.robot) paintRobotActivity(j.robot);
    startDemoPoll();
    for(let i=0;i<40;i++){
      await new Promise(r=>setTimeout(r,400));
      await refreshRobotState(false);
      const d = lastDemo || {};
      if(d.running) continue;
      const st = String(d.status||'');
      const chk = (d.params||{}).stand_check;
      if(st.startsWith('error')){
        showSent(st.replace(/^error:\s*/,'')); return;
      }
      if(st==='done'){
        if(pose==='stand' && chk && chk.max_err_deg!=null)
          showSent('stand verified · tracking '+Number(chk.max_err_deg).toFixed(1)+'°');
        else
          showSent(tag + ' done');
        return;
      }
      if(st==='aborted'){ showSent(tag + ' aborted'); return; }
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

function setGait(g){ gait=g;
  document.querySelectorAll('#gaits button').forEach(btn=>
    btn.classList.toggle('on', +btn.dataset.gait===g)); }
function disc(c){
  if(c==='X'){ settleServos(); return; }
  // Stand / Center: use verified glide, not the old one-shot /cmd P|C.
  if(c==='P'){ goPoseZero('stand', '▲ Stand'); forceResend(); return; }
  if(c==='C'){ goPoseZero('sit', 'Center / Sit'); forceResend(); return; }
  if(needArm()) return;
  cmd(c); forceResend();
}

document.querySelectorAll('#gaits button').forEach(btn=>
  btn.onclick=()=>setGait(+btn.dataset.gait));
document.querySelectorAll('button[data-cmd]').forEach(btn=>
  btn.onclick=()=> disc(btn.dataset.cmd));
document.getElementById('stop').onclick=settleServos;

const vmax=document.getElementById('vmax'), lift=document.getElementById('lift');
vmax.oninput=()=>{ maxVx=+vmax.value; maxVy=Math.round(maxVx*0.73);
  document.getElementById('vlab').textContent=vmax.value; };
lift.oninput=()=>{ document.getElementById('klab').textContent=lift.value; };
lift.onchange=()=>cmd('K '+lift.value);
maxVx = +vmax.value; maxVy = Math.round(maxVx*0.73);

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

$('dbgstand').onclick  = ()=> goPoseZero('stand', '▲ Stand');
$('dbgcenter').onclick = ()=> goPoseZero('sit', 'Center / Sit');
$('dbgrelax').onclick  = disarmServos;   // Relax (limp) = disarm / e-stop, always allowed

// --- tab switching ----------------------------------------------------------
function showView(which){
  activeView = which;
  ['drive','live','motors','demos','rl','calibrate','debug'].forEach(v=>{
    const el = $('view-'+v); if(el) el.classList.toggle('active', v===which);
    const tab = $('tab-'+v); if(tab) tab.classList.toggle('on', v===which);
  });
  if(which !== 'drive') armed = false;   // stop streaming J
  if(which === 'motors'){
    // Freeze stand/walk re-hold so the Motors tab can wiggle without the
    // background loop yanking the body toward the plant/crouch pose.
    if(servosArmed){ cmd('HOLD'); forceResend(); }
    refreshMotors(); startMotorsPoll();
  }
  else stopMotorsPoll();
  if(which === 'demos'){ loadDemos(); refreshDemoStatus(); startDemoPoll(); }
  else stopDemoPoll();
  if(which === 'calibrate'){
    if(servosArmed){ cmd('HOLD'); forceResend(); }
    refreshCalibrate(); startCalPoll();
  }
  else stopCalPoll();
  if(which === 'live'){ startLivePose(); }
  else stopLivePose();
  if(which === 'rl'){ refreshRlTab(); }
}
$('tab-drive').onclick = ()=> showView('drive');
$('tab-live').onclick = ()=> showView('live');
$('tab-motors').onclick = ()=> showView('motors');
$('tab-demos').onclick = ()=> showView('demos');
$('tab-rl').onclick = ()=> showView('rl');
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
                   'rlwalkleft','rlwalkright'])
    $(id).disabled = disabled;
}
async function rlMove(mode, body){
  const what = mode==='stand'
    ? 'STAND UP (robot must be belly-down, legs straight out)'
    : mode==='lower'
    ? 'LOWER to belly (robot must be in the plant stance)'
    : `WALK ${body.vx>0?'forward':body.vy>0?'strafe LEFT':'strafe RIGHT'} `
      + `at ${Math.hypot(body.vx,body.vy).toFixed(2)} m/s for `
      + `${body.duration_s}s — EXPERIMENTAL, robot must be in the plant `
      + `stance with room to move`;
  if(!confirm('Robot will MOVE: '+what+'. Are you watching it?')) return;
  $('rlstatus').textContent = 'Preflight…';
  rlButtons(true);
  try{
    const r = await fetch('/api/rl/'+mode, {method:'POST',
      body: body ? JSON.stringify(body) : undefined});
    const d = await r.json();
    if(!d.ok){
      $('rlstatus').textContent = 'Refused: '+(d.error || 'unknown');
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
  if(!confirm('Robot will MOVE: scripted glide to the captured plant '
              + 'stance over ~4.5 s. Are you watching it?')) return;
  $('rlstatus').textContent = 'Gliding to plant stance…';
  await goPoseZero('stand', '▲ Stand (scripted)');
  $('rlstatus').textContent = 'Glide finished — see toast for result.';
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
function rlWalk(dx, dy){
  const s = parseFloat($('rlwalkspeed').value);
  rlMove('walk', {vx: dx*s, vy: dy*s,
                  duration_s: parseFloat($('rlwalkdur').value)});
}
$('rlwalkfwd').onclick   = ()=> rlWalk(1, 0);
$('rlwalkleft').onclick  = ()=> rlWalk(0, 1);   // +vy = strafe left
$('rlwalkright').onclick = ()=> rlWalk(0, -1);
$('rlstop').onclick = async ()=>{
  await fetch('/api/rl/stop', {method:'POST'});
  $('rlstatus').textContent = 'Stopping (holds pose; X to limp)…';
};
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
  try{
    const r = await fetch('/api/calibrate?t='+Date.now(), {cache:'no-store'});
    const d = await r.json();
    if(d.running && (d.name||'').startsWith('rl_policy')){
      $('rlstatus').textContent = (d.progress||{}).msg || 'running…';
      rlButtons(true);
      startRlPoll();
    }
  }catch(e){}
}
$('calplantreset').onclick = async ()=>{
  if(!confirm('Reset stand home to default hip +20° / knee +80°?')) return;
  try{
    const r = await fetch('/api/plant/reset', {method:'POST'});
    const d = await r.json();
    if(d.ok){ paintPlantInfo(d); showSent('plant reset to default'); }
    else showSent(d.error || 'reset failed');
    refreshCalibrate();
  }catch(e){ showSent('plant reset failed'); }
};
$('calimureset').onclick = async ()=>{
  if(!confirm('Clear IMU calibration? Motion logs will use raw MPU until you re-run IMU rest.')) return;
  try{
    const r = await fetch('/api/imu/reset', {method:'POST'});
    const d = await r.json();
    if(d.ok){ paintImuInfo(d); showSent('IMU calib cleared'); }
    else showSent(d.error || 'reset failed');
    refreshCalibrate();
  }catch(e){ showSent('IMU reset failed'); }
};

// --- Live schematic (FK from motor feedback) --------------------------------
let liveTimer = null;
let livePaused = false;
let liveBusy = false;
let livePose = null;   // last /api/pose payload
let liveGeom = { coxa_mm:12.5, femur_mm:90, tibia_mm:128, body_r_mm:55 };
let liveView = { yaw:0.55, pitch:0.55, zoom:1.0 };
let liveDrag = null;
const LEG_COLORS = ['#7eb6ff','#5fd08a','#e6b35a','#ff8a8a','#c59bff','#7fd7d0'];

function startLivePose(){
  stopLivePose();
  refreshLivePose(true);
  liveTimer = setInterval(()=>{
    if(activeView==='live' && !livePaused) refreshLivePose(false);
  }, 350);  // was 125ms — too chatty on the MCU bus during stand/rise
  startLiveDraw();
}
function stopLivePose(){
  if(liveTimer){ clearInterval(liveTimer); liveTimer=null; }
}
async function refreshLivePose(force){
  if(liveBusy && !force) return;
  liveBusy = true;
  try{
    const r = await fetch('/api/pose?t='+Date.now(), {cache:'no-store'});
    if(!r.ok) throw new Error('HTTP '+r.status);
    const p = await r.json();
    setLink(true);
    livePose = p;
    if(p.geom) liveGeom = p.geom;
    paintLiveAngles(p);
    const st = $('livestamp');
    if(st) st.textContent = '· updated '+new Date().toLocaleTimeString();
    const hint = $('livehint');
    if(hint){
      if(!p.ok) hint.textContent = p.error || 'pose failed';
      else if(p.dry_run) hint.textContent = 'Dry-run — showing sit-zero placeholder.';
      else if((p.live||0) < 3) hint.textContent = 'Few motors answering ('+(p.live||0)+'/18) — check bus/power.';
      else hint.textContent = 'Drawing from present servo angles ('+p.live+'/18 live).';
    }
  }catch(e){
    setLink(false, 'pose failed');
    const hint = $('livehint');
    if(hint) hint.textContent = 'Pose fetch failed — is the board reachable?';
  }finally{
    liveBusy = false;
  }
}
function paintLiveAngles(p){
  const deg = p.degrees || [];
  $('livencount').textContent = String(p.live||0);
  const mode = $('livemode');
  if(mode){
    mode.textContent = (p.armed?'ARMED':'limp')+' / '+(p.mode||'—');
    mode.className = 'pill '+(p.armed?'ok':'bad');
  }
  const tb = $('livebody'); if(!tb) return;
  let html = '';
  for(let leg=0; leg<6; leg++){
    const y = deg[leg*3], h = deg[leg*3+1], k = deg[leg*3+2];
    const off = (y==null && h==null && k==null);
    const fmt = v => (v==null||v===undefined) ? '—' : Number(v).toFixed(1);
    html += '<tr class="'+(off?'off':'')+'"><td style="color:'+LEG_COLORS[leg]+'">L'+leg+
      '</td><td>'+fmt(y)+'</td><td>'+fmt(h)+'</td><td>'+fmt(k)+'</td></tr>';
  }
  tb.innerHTML = html;
}
function liveFkLegs(degrees){
  const coxa = liveGeom.coxa_mm||12.5;
  const femur = liveGeom.femur_mm||90;
  const tibia = liveGeom.tibia_mm||128;
  const bodyR = liveGeom.body_r_mm||55;
  const legs = [];
  for(let i=0;i<6;i++){
    const yaw = (degrees[i*3]==null)?0:degrees[i*3]*Math.PI/180;
    const hip = (degrees[i*3+1]==null)?0:degrees[i*3+1]*Math.PI/180;
    const knee = (degrees[i*3+2]==null)?0:degrees[i*3+2]*Math.PI/180;
    const live = degrees[i*3]!=null || degrees[i*3+1]!=null || degrees[i*3+2]!=null;
    // Leg root around hex: L0 at +X (east), CCW.
    const baseAz = i * Math.PI/3;
    const az = baseAz + yaw;
    const hx = Math.cos(baseAz)*bodyR;
    const hy = Math.sin(baseAz)*bodyR;
    const hz = 0;
    // Hip after coxa along yawed radial.
    const cx = hx + Math.cos(az)*coxa;
    const cy = hy + Math.sin(az)*coxa;
    const cz = hz;
    // Femur then tibia in the vertical plane of `az`.
    // Matches plant FK: z = -F·sin(hip) - T·sin(hip+knee)
    //                   r =  F·cos(hip) + T·cos(hip+knee)
    const kx = cx + Math.cos(az)*femur*Math.cos(hip);
    const ky = cy + Math.sin(az)*femur*Math.cos(hip);
    const kz = cz - femur*Math.sin(hip);
    const rOut = femur*Math.cos(hip) + tibia*Math.cos(hip+knee);
    const fx = cx + Math.cos(az)*rOut;
    const fy = cy + Math.sin(az)*rOut;
    const fz = cz - femur*Math.sin(hip) - tibia*Math.sin(hip+knee);
    legs.push({live, hip:[hx,hy,hz], coxa:[cx,cy,cz], knee:[kx,ky,kz], foot:[fx,fy,fz]});
  }
  return legs;
}
function liveProject(p, W, H){
  // Orbit camera around origin; +Z up, ground toward bottom of screen.
  const cy = Math.cos(liveView.yaw), sy = Math.sin(liveView.yaw);
  const cp = Math.cos(liveView.pitch), sp = Math.sin(liveView.pitch);
  let x = p[0], y = p[1], z = p[2];
  // yaw about Z
  let x1 = x*cy - y*sy, y1 = x*sy + y*cy, z1 = z;
  // pitch about X' (look down from +Y toward −Z)
  let y2 = y1*cp - z1*sp, z2 = y1*sp + z1*cp;
  const scale = 1.15 * liveView.zoom * Math.min(W,H) / 420;
  // Canvas Y grows downward — add y2 so world-down (feet) lands near the bottom.
  return [W*0.52 + x1*scale, H*0.42 + y2*scale, z2]; // z2 retained for depth sort
}
let liveRaf = 0;
function startLiveDraw(){
  if(liveRaf) cancelAnimationFrame(liveRaf);
  const tick = ()=>{
    drawLiveSchematic();
    if(activeView==='live') liveRaf = requestAnimationFrame(tick);
    else liveRaf = 0;
  };
  liveRaf = requestAnimationFrame(tick);
}
function drawLiveSchematic(){
  const cv = $('livecv'); if(!cv) return;
  const ctx = cv.getContext('2d');
  const W = cv.width, H = cv.height;
  ctx.clearRect(0,0,W,H);
  // backdrop
  const g = ctx.createLinearGradient(0,0,0,H);
  g.addColorStop(0,'#12151c'); g.addColorStop(1,'#0a0c10');
  ctx.fillStyle = g; ctx.fillRect(0,0,W,H);

  const deg = (livePose && livePose.degrees) || Array(18).fill(0);
  const legs = liveFkLegs(deg);
  const bodyR = liveGeom.body_r_mm||55;

  // ground grid (z = min foot or 0)
  let groundZ = 0;
  for(const L of legs){ if(L.live) groundZ = Math.min(groundZ, L.foot[2]); }
  // snap soft ground under feet
  const grid = [];
  for(let i=-4;i<=4;i++){
    for(let j=-4;j<=4;j++){
      grid.push(liveProject([i*40, j*40, groundZ], W, H));
    }
  }
  ctx.strokeStyle = '#1e2433'; ctx.lineWidth = 1;
  for(let i=-4;i<=4;i++){
    const a = liveProject([i*40, -160, groundZ], W, H);
    const b = liveProject([i*40,  160, groundZ], W, H);
    ctx.beginPath(); ctx.moveTo(a[0],a[1]); ctx.lineTo(b[0],b[1]); ctx.stroke();
    const c = liveProject([-160, i*40, groundZ], W, H);
    const d = liveProject([ 160, i*40, groundZ], W, H);
    ctx.beginPath(); ctx.moveTo(c[0],c[1]); ctx.lineTo(d[0],d[1]); ctx.stroke();
  }

  // body hex (top face + slight thickness)
  const bodyTop = [], bodyBot = [];
  for(let i=0;i<6;i++){
    const a = i*Math.PI/3;
    bodyTop.push(liveProject([Math.cos(a)*bodyR, Math.sin(a)*bodyR, 8], W, H));
    bodyBot.push(liveProject([Math.cos(a)*bodyR, Math.sin(a)*bodyR, -6], W, H));
  }
  // bottom
  ctx.beginPath();
  bodyBot.forEach((p,i)=>{ if(i===0) ctx.moveTo(p[0],p[1]); else ctx.lineTo(p[0],p[1]); });
  ctx.closePath(); ctx.fillStyle = '#151a24'; ctx.fill(); ctx.strokeStyle='#2a3348'; ctx.stroke();
  // sides
  for(let i=0;i<6;i++){
    const j=(i+1)%6;
    ctx.beginPath();
    ctx.moveTo(bodyBot[i][0],bodyBot[i][1]);
    ctx.lineTo(bodyBot[j][0],bodyBot[j][1]);
    ctx.lineTo(bodyTop[j][0],bodyTop[j][1]);
    ctx.lineTo(bodyTop[i][0],bodyTop[i][1]);
    ctx.closePath();
    ctx.fillStyle = (i%2)? '#1a2233':'#162033';
    ctx.fill(); ctx.strokeStyle='#2b6cff55'; ctx.stroke();
  }
  // top
  ctx.beginPath();
  bodyTop.forEach((p,i)=>{ if(i===0) ctx.moveTo(p[0],p[1]); else ctx.lineTo(p[0],p[1]); });
  ctx.closePath(); ctx.fillStyle='#1b2744'; ctx.fill();
  ctx.strokeStyle='#2b6cff'; ctx.lineWidth=2; ctx.stroke();
  // forward mark (between L0 and L5 → +X)
  const nose = liveProject([bodyR*0.55, 0, 10], W, H);
  const ctr = liveProject([0,0,10], W, H);
  ctx.fillStyle='#e7eaf0'; ctx.beginPath();
  ctx.arc(nose[0], nose[1], 4, 0, 7); ctx.fill();
  ctx.strokeStyle='#8089a0'; ctx.lineWidth=1;
  ctx.beginPath(); ctx.moveTo(ctr[0],ctr[1]); ctx.lineTo(nose[0],nose[1]); ctx.stroke();

  // legs (depth sort by mid z')
  const order = legs.map((L,i)=>({i, z: liveProject(L.knee, W, H)[2]})).sort((a,b)=>a.z-b.z);
  for(const {i} of order){
    const L = legs[i];
    const col = L.live ? LEG_COLORS[i] : '#3a4256';
    const pts = [L.hip, L.coxa, L.knee, L.foot].map(p=>liveProject(p,W,H));
    // shadow
    const sh = liveProject([L.foot[0], L.foot[1], groundZ], W, H);
    ctx.fillStyle = 'rgba(0,0,0,0.35)';
    ctx.beginPath(); ctx.ellipse(sh[0], sh[1], 10, 4, 0, 0, 7); ctx.fill();
    // segments
    ctx.lineCap = 'round'; ctx.lineJoin = 'round';
    ctx.strokeStyle = col; ctx.globalAlpha = L.live ? 1 : 0.45;
    ctx.lineWidth = 7;
    ctx.beginPath(); ctx.moveTo(pts[0][0],pts[0][1]); ctx.lineTo(pts[1][0],pts[1][1]); ctx.stroke();
    ctx.lineWidth = 6;
    ctx.beginPath(); ctx.moveTo(pts[1][0],pts[1][1]); ctx.lineTo(pts[2][0],pts[2][1]); ctx.stroke();
    ctx.lineWidth = 5;
    ctx.beginPath(); ctx.moveTo(pts[2][0],pts[2][1]); ctx.lineTo(pts[3][0],pts[3][1]); ctx.stroke();
    // joints
    ctx.fillStyle = '#0c0e13';
    for(const p of pts){ ctx.beginPath(); ctx.arc(p[0],p[1],3.2,0,7); ctx.fill();
      ctx.strokeStyle = col; ctx.lineWidth=1.5; ctx.stroke(); }
    // foot
    ctx.fillStyle = L.live ? '#e7eaf0' : '#5a6478';
    ctx.beginPath(); ctx.arc(pts[3][0],pts[3][1],4.5,0,7); ctx.fill();
    ctx.globalAlpha = 1;
    // label
    ctx.fillStyle = col; ctx.font = '11px system-ui,sans-serif';
    ctx.fillText('L'+i, pts[0][0]+6, pts[0][1]-6);
  }

  // HUD
  ctx.fillStyle = '#5a6478'; ctx.font = '11px system-ui,sans-serif';
  ctx.fillText('orbit drag · wheel zoom', 12, H-14);
  ctx.fillText('FWD', nose[0]+8, nose[1]-6);
}
(function bindLiveCanvas(){
  const cv = $('livecv'); if(!cv) return;
  cv.addEventListener('pointerdown', e=>{
    liveDrag = {x:e.clientX, y:e.clientY, yaw:liveView.yaw, pitch:liveView.pitch};
    cv.classList.add('dragging');
    cv.setPointerCapture(e.pointerId);
  });
  cv.addEventListener('pointermove', e=>{
    if(!liveDrag) return;
    const dx = e.clientX - liveDrag.x, dy = e.clientY - liveDrag.y;
    liveView.yaw = liveDrag.yaw + dx*0.01;
    liveView.pitch = Math.max(0.15, Math.min(1.35, liveDrag.pitch + dy*0.01));
  });
  const end = e=>{ liveDrag=null; cv.classList.remove('dragging'); };
  cv.addEventListener('pointerup', end);
  cv.addEventListener('pointercancel', end);
  cv.addEventListener('wheel', e=>{
    e.preventDefault();
    liveView.zoom = Math.max(0.45, Math.min(2.4, liveView.zoom * (e.deltaY>0?0.92:1.08)));
  }, {passive:false});
})();
$('livereset').onclick = ()=>{ liveView={yaw:0.55,pitch:0.55,zoom:1.0}; };
$('livepause').onclick = ()=>{
  livePaused = !livePaused;
  $('livepause').textContent = livePaused ? 'Resume' : 'Pause';
  $('livepause').classList.toggle('on', livePaused);
};
$('liverefresh').onclick = ()=> refreshLivePose(true);

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
  if(needArm()) return;
  await fetch('/api/zero',{method:'POST'}); showSent('go zero');
};
$('msetzero').onclick = async ()=>{
  const msg = [
    'Set CURRENT pose as 0° on all live motors?',
    '',
    'Motors will NOT move — only the zero point is rewritten.',
    'Limp and hand-pose first (usually legs straight out).',
    '',
    'Continue?'
  ].join('\n');
  if(!confirm(msg)) return;
  showSent('set-here-as-zero…');
  try{
    const r = await fetch('/api/set_zero',{method:'POST'});
    const j = await r.json();
    setArmed(false);
    if(j.ok) showSent('zero-here OK — '+j.ok_n+'/'+j.count+' (limp)');
    else showSent('zero-here '+(j.error || ((j.ok_n||0)+'/'+(j.count||0)+' — check Motors table')));
    refreshMotors();
  }catch(e){ showSent('zero-here failed'); }
};
$('mlimp').onclick = ()=>{ cmd('X'); setArmed(false); };

// --- Demos tab + global robot activity --------------------------------------
function demoSpeed(){ return Math.max(0.25, Math.min(2.0, (+$('dspeed').value)/100)); }
function demoSize(){ return Math.max(0.5, Math.min(3.0, (+$('dsize').value)/100)); }
function demoRate(){ return Math.max(0.08, Math.min(0.60, (+$('drate').value)/100)); }
function demoSoft(){ return Math.max(0.5, Math.min(3.0, (+$('dsoft').value)/100)); }
function demoTorque(){ return Math.max(150, Math.min(1000, Math.round((+$('dtorque').value)*10))); }
$('dspeed').oninput = ()=> $('dspeedlab').textContent = demoSpeed().toFixed(2);
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
    if(activeView!=='demos') return;
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

// --- schematic demo preview (top-down stick hexapod) ----------------------
// Not a sim — just shows sit vs stand start + the intended motion idea.
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
    start: (name||'').startsWith('plant') || (name||'').startsWith('rise') ? 'stand' : 'sit',
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
function drawDemoPreview(ts){
  const cv = $('dprevcv'); if(!cv) return;
  if(activeView!=='demos'){ prevRaf = requestAnimationFrame(drawDemoPreview); return; }
  const ctx = cv.getContext('2d');
  const W = cv.width, H = cv.height;
  ctx.clearRect(0,0,W,H);
  // two panels: top-down | side
  const mid = W*0.52;
  ctx.fillStyle = '#12151c';
  ctx.fillRect(0,0,mid-6,H);
  ctx.fillRect(mid+6,0,W-(mid+6),H);
  ctx.fillStyle = '#5a6478';
  ctx.font = '11px system-ui,sans-serif';
  ctx.fillText('TOP', 10, 16);
  ctx.fillText('SIDE', mid+16, 16);

  const info = demoPreviewInfo(prevName);
  if(!prevT0) prevT0 = ts;
  const t = (ts - prevT0) / 1000;
  const legs = [];
  for(let i=0;i<6;i++) legs.push({yaw:0, hip:0, knee:0});
  // base pose
  if(info.start==='sit'){
    for(let i=0;i<6;i++){ legs[i].yaw=0; legs[i].hip=0; legs[i].knee=0; }
  } else {
    for(let i=0;i<6;i++){ legs[i].yaw=0; legs[i].hip=-0.55; legs[i].knee=0.9; }
  }
  try{ info.pose(t, legs); }catch(e){}

  // --- top-down ---
  const cx = (mid-6)/2, cy = H*0.55, R = Math.min(cx, cy)-28;
  ctx.strokeStyle = '#2b6cff'; ctx.lineWidth = 2;
  ctx.beginPath();
  for(let i=0;i<6;i++){
    const a = -Math.PI/2 + i*Math.PI/3;
    const x = cx + Math.cos(a)*R*0.28, y = cy + Math.sin(a)*R*0.28;
    if(i===0) ctx.moveTo(x,y); else ctx.lineTo(x,y);
  }
  ctx.closePath(); ctx.stroke();
  ctx.fillStyle = '#1b2744'; ctx.fill();
  for(let i=0;i<6;i++){
    const a = -Math.PI/2 + i*Math.PI/3 + legs[i].yaw*0.7;
    const hipLen = R*(info.start==='sit' ? 0.55 : 0.38);
    const kneeLen = R*(info.start==='sit' ? 0.45 : 0.34);
    // fold shortens radial reach when hip/knee flex
    const reach = hipLen*Math.cos(legs[i].hip*0.9) + kneeLen*Math.cos(legs[i].hip+legs[i].knee)*0.7;
    const x0 = cx + Math.cos(a)*R*0.28, y0 = cy + Math.sin(a)*R*0.28;
    const x1 = x0 + Math.cos(a)*hipLen*0.55;
    const y1 = y0 + Math.sin(a)*hipLen*0.55;
    const x2 = x0 + Math.cos(a)*Math.max(0.12*R, reach);
    const y2 = y0 + Math.sin(a)*Math.max(0.12*R, reach);
    ctx.strokeStyle = '#9aa3b2'; ctx.lineWidth = 3;
    ctx.beginPath(); ctx.moveTo(x0,y0); ctx.lineTo(x1,y1); ctx.lineTo(x2,y2); ctx.stroke();
    ctx.fillStyle = '#5fd08a';
    ctx.beginPath(); ctx.arc(x2,y2,3.5,0,7); ctx.fill();
  }

  // --- side (one representative leg + body) ---
  const sx0 = mid + 36, sy0 = H*0.72;
  const bodyW = 54, bodyH = 18;
  ctx.fillStyle = '#1b2744'; ctx.strokeStyle = '#2b6cff'; ctx.lineWidth = 2;
  ctx.fillRect(sx0, sy0-bodyH, bodyW, bodyH); ctx.strokeRect(sx0, sy0-bodyH, bodyW, bodyH);
  // ground
  ctx.strokeStyle = '#333b52'; ctx.beginPath();
  ctx.moveTo(mid+16, sy0+52); ctx.lineTo(W-12, sy0+52); ctx.stroke();
  if(info.start==='sit'){
    ctx.fillStyle = '#5a6478'; ctx.fillText('legs out / air', mid+16, 36);
  } else {
    ctx.fillStyle = '#5a6478'; ctx.fillText('planted / stand', mid+16, 36);
  }
  // average hip/knee for side view
  let ah=0, ak=0; for(const L of legs){ ah+=L.hip; ak+=L.knee; } ah/=6; ak/=6;
  const femur = 46, tibia = 40;
  const hx = sx0 + bodyW*0.65, hy = sy0;
  const kx = hx + Math.sin(ah)*femur, ky = hy + Math.cos(ah)*femur;
  const fx = kx + Math.sin(ah+ak)*tibia, fy = ky + Math.cos(ah+ak)*tibia;
  ctx.strokeStyle = '#e7eaf0'; ctx.lineWidth = 3;
  ctx.beginPath(); ctx.moveTo(hx,hy); ctx.lineTo(kx,ky); ctx.lineTo(fx,fy); ctx.stroke();
  ctx.fillStyle = '#5fd08a'; ctx.beginPath(); ctx.arc(fx,fy,4,0,7); ctx.fill();
  ctx.fillStyle = '#8089a0'; ctx.font = '10px system-ui,sans-serif';
  ctx.fillText('hip '+ (ah*57.3).toFixed(0)+'°  knee '+(ak*57.3).toFixed(0)+'°', mid+16, H-12);

  prevRaf = requestAnimationFrame(drawDemoPreview);
}
function startDemoPreviewLoop(){
  if(prevRaf) cancelAnimationFrame(prevRaf);
  prevT0 = 0;
  prevRaf = requestAnimationFrame(drawDemoPreview);
}

async function loadDemos(){
  try{
    const r = await fetch('/api/demos'); const d = await r.json();
    const g = $('dgrid'); g.innerHTML='';
    (d.demos||[]).forEach(item=>{
      const b = document.createElement('button');
      b.dataset.name = item.name;
      b.innerHTML = '<b>'+item.name+'</b><br><span style="color:#9aa3b2;font-weight:400;font-size:12px">'
        +item.title+'</span>';
      b.onmouseenter = ()=> setDemoPreview(item.name);
      b.onfocus = ()=> setDemoPreview(item.name);
      b.onclick = async ()=>{
        setDemoPreview(item.name);
        if(needArm()) return;
        const sp = demoSpeed();
        const body = {name:item.name, speed:sp, torque:demoTorque()};
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
          showSent(j.error||'failed');
          if(j.zero){ lastZero = j.zero; paintZeroHint(j.zero); }
        }
        if(j.demo) paintDemoStatus(j.demo);
        if(j.robot) paintRobotActivity(j.robot);
        startDemoPoll();
        refreshRobotState(true);
      };
      g.appendChild(b);
    });
    if((d.demos||[]).length) setDemoPreview(d.demos[0].name);
    startDemoPreviewLoop();
  }catch(e){ $('dgrid').innerHTML = '<div class="hint">Failed to load demos</div>'; }
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
$('dzero').onclick = ()=> goPoseZero('sit', 'sit zero');
$('dstand').onclick = ()=> goPoseZero('stand', 'stand zero');
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

// deep links
const path = location.pathname.replace(/\/+$/,'');
if(path.endsWith('/live') || location.hash==='#live') showView('live');
else if(path.endsWith('/motors') || location.hash==='#motors') showView('motors');
else if(path.endsWith('/demos') || location.hash==='#demos') showView('demos');
else if(path.endsWith('/debug') || location.hash==='#debug') showView('debug');

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
        pass  # access noise; meaningful traffic goes through emit_http

    def _send(self, code, body, ctype="text/plain; charset=utf-8"):
        data = body.encode("utf-8") if isinstance(body, str) else body
        self.send_response(code)
        self.send_header("Content-Type", ctype)
        self.send_header("Content-Length", str(len(data)))
        if "text/html" in ctype:
            self.send_header("Cache-Control", "no-store")
        self.end_headers()
        try:
            self.wfile.write(data)
        except OSError:
            pass

    def _json(self, code, obj):
        # Any error response the website shows also lands in the event
        # log + logs/errors.jsonl (refusals, ok:false, 4xx/5xx).
        try:
            err = None
            if isinstance(obj, dict):
                if obj.get("error"):
                    err = str(obj["error"])
                elif obj.get("ok") is False:
                    err = "request failed (no error message)"
            if err is not None or code >= 400:
                from event_log import emit_api_error
                emit_api_error(self.command, self.path, code=code,
                               error=err, peer=self._peer())
        except Exception:
            pass
        self._send(code, json.dumps(obj), "application/json")

    def _peer(self) -> str:
        try:
            return self.client_address[0]
        except Exception:
            return ""

    def do_GET(self):
        path = self.path.split("?", 1)[0]
        try:
            from event_log import emit_http
            emit_http("GET", self.path, peer=self._peer())
        except Exception:
            pass
        if path in ("/", "/index.html", "/debug", "/motors", "/demos",
                    "/live", "/rl", "/calibrate"):
            page = PAGE.replace("__HTTPS_PORT__", str(HTTPS_PORT or 8443))
            self._send(200, page, "text/html; charset=utf-8")
        elif path == "/cal":
            self._json(200, {"stand_z": CAL.get("stand_z"),
                             "tuck_r": CAL.get("tuck_r")})
        elif path == "/api/ping":
            # Lightweight heartbeat — no bus traffic.
            self._json(200, {"ok": True, "service": "hexapod-web"})
        elif path == "/api/rl/preflight":
            mode = "stand"
            qs = self.path.split("?", 1)
            if len(qs) == 2 and "mode=" in qs[1]:
                mode = qs[1].split("mode=")[1].split("&")[0]
            self._json(200, BENCH.rl_preflight(mode=mode) if BENCH
                       else {"ok": False, "error": "no bench"})
        elif path == "/api/rl/policy":
            self._json(200, BENCH.rl_policy_info() if BENCH
                       else {"ok": False, "error": "no bench"})
        elif path == "/api/events":
            try:
                from event_log import recent, events_path
                n = 100
                qs = self.path.split("?", 1)
                if len(qs) == 2 and "n=" in qs[1]:
                    try:
                        n = int(qs[1].split("n=")[1].split("&")[0])
                    except ValueError:
                        n = 100
                self._json(200, {
                    "ok": True,
                    "path": str(events_path()),
                    "events": recent(n),
                })
            except Exception as e:
                self._json(500, {"ok": False, "error": str(e)})
        elif path == "/api/errors":
            try:
                from event_log import errors_path, recent
                n = 100
                qs = self.path.split("?", 1)
                if len(qs) == 2 and "n=" in qs[1]:
                    try:
                        n = int(qs[1].split("n=")[1].split("&")[0])
                    except ValueError:
                        n = 100
                errs = [e for e in recent(500)
                        if e.get("level") == "error"][-max(1, n):]
                self._json(200, {
                    "ok": True,
                    "path": str(errors_path()),
                    "errors": errs,
                })
            except Exception as e:
                self._json(500, {"ok": False, "error": str(e)})
        elif path == "/api/demo/status":
            # Back-compat; prefer /api/robot.
            if BENCH:
                self._json(200, BENCH.robot_state(check_zero=False))
            else:
                self._json(200, {
                    "activity": "idle", "demo": {
                        "name": None, "status": "idle", "running": False,
                    }})
        elif path == "/api/robot":
            qs = self.path.split("?", 1)
            want_zero = False
            if len(qs) == 2:
                want_zero = "zero=1" in qs[1] or "zero=true" in qs[1]
            if BENCH:
                self._json(200, BENCH.robot_state(check_zero=want_zero))
            else:
                self._json(200, {"activity": "idle", "error": "no bench"})
        elif path == "/api/status":
            self._json(200, BENCH.status() if BENCH else {"error": "no bench"})
        elif path == "/api/pose":
            self._json(200, BENCH.pose() if BENCH else {"ok": False, "error": "no bench"})
        elif path == "/api/demos":
            self._json(200, {"demos": BENCH.list_demos() if BENCH else []})
        elif path == "/api/calibrate":
            self._json(200, BENCH.calibrate_state() if BENCH
                       else {"running": False, "error": "no bench"})
        elif path == "/api/plant":
            self._json(200, BENCH.plant_state() if BENCH
                       else {"ok": False, "error": "no bench"})
        elif path == "/api/imu":
            self._json(200, BENCH.imu_state() if BENCH
                       else {"ok": False, "error": "no bench"})
        elif path == "/api/rl/state" or path == "/api/rl":
            self._json(200, BENCH.rl_state() if BENCH
                       else {"ok": False, "error": "no bench"})
        elif path == "/api/feedback":
            # Fast read-only bulk telemetry (one MCU round-trip + IMU) for
            # external loggers — /api/status's full scan takes seconds.
            self._json(200, BENCH.rl_feedback() if BENCH
                       else {"ok": False, "error": "no bench"})
        else:
            self._send(404, "not found")

    def do_POST(self):
        n = int(self.headers.get("Content-Length", 0) or 0)
        raw = self.rfile.read(n) if n else b""
        body = raw.decode("utf-8", "ignore") if raw else ""
        path = self.path.split("?", 1)[0]
        body_obj = None
        if body:
            try:
                body_obj = json.loads(body)
            except ValueError:
                body_obj = body.strip()[:500]
        try:
            from event_log import emit_http
            emit_http("POST", self.path, body=body_obj, peer=self._peer())
        except Exception:
            pass
        if path == "/cmd":
            ok = LINK.send(body.strip())
            self._send(200 if ok else 502, "ok" if ok else "link down")
        elif path == "/api/wiggle":
            try:
                data = json.loads(body or "{}")
                self._json(200, BENCH.wiggle(int(data.get("joint", -1)),
                                            float(data.get("amp", 6))))
            except Exception as e:
                self._json(400, {"ok": False, "error": str(e)})
        elif path == "/api/demo":
            try:
                data = json.loads(body or "{}")
                kw = dict(
                    speed=float(data.get("speed", 1.0)),
                    size=float(data.get("size", 1.0)),
                    softness=float(data.get("softness", 1.0)),
                )
                if "rate" in data and data.get("rate") is not None:
                    kw["rate"] = float(data["rate"])
                if "torque" in data and data.get("torque") is not None:
                    kw["torque"] = int(float(data["torque"]))
                self._json(200, BENCH.run_demo(str(data.get("name", "")), **kw))
            except Exception as e:
                self._json(400, {"ok": False, "error": str(e)})
        elif path == "/api/demo/stop":
            self._json(200, BENCH.stop_demo())
        elif path == "/api/zero":
            try:
                data = json.loads(body or "{}") if body else {}
            except ValueError:
                data = {}
            pose = "sit"
            force = False
            if isinstance(data, dict):
                if data.get("pose"):
                    pose = str(data.get("pose"))
                force = bool(data.get("force", False))
            elif body and body.strip() in ("sit", "stand"):
                pose = body.strip()
            self._json(200, BENCH.go_zero(pose=pose, force=force))
        elif path == "/api/set_zero":
            self._json(200, BENCH.set_zero_here() if BENCH
                       else {"ok": False, "error": "no bench"})
        elif path == "/api/calibrate":
            try:
                data = json.loads(body or "{}")
            except ValueError:
                data = {}
            if not BENCH:
                self._json(400, {"ok": False, "error": "no bench"})
            else:
                self._json(200, BENCH.run_calibrate(
                    mode=str(data.get("mode", "step")),
                    step_deg=float(data.get("step_deg", 10)),
                    nudge_deg=float(data.get("nudge_deg", 2)),
                    axis=str(data.get("axis", "all")),
                    clearance_mm=float(data.get("clearance_mm", 40)),
                    force=bool(data.get("force", False)),
                ))
        elif path == "/api/calibrate/stop":
            self._json(200, BENCH.stop_calibrate() if BENCH
                       else {"ok": False, "error": "no bench"})
        elif path == "/api/plant/reset":
            self._json(200, BENCH.reset_plant() if BENCH
                       else {"ok": False, "error": "no bench"})
        elif path == "/api/imu/reset":
            self._json(200, BENCH.reset_imu() if BENCH
                       else {"ok": False, "error": "no bench"})
        elif path == "/api/rl/find_plant":
            try:
                data = json.loads(body or "{}") if body else {}
            except ValueError:
                data = {}
            if not BENCH:
                self._json(400, {"ok": False, "error": "no bench"})
            else:
                self._json(200, BENCH.rl_find_plant(
                    clearance_mm=float(data.get("clearance_mm", 40)),
                    force=bool(data.get("force", False))))
        elif path == "/api/rl/capture_plant":
            self._json(200, BENCH.rl_capture_plant() if BENCH
                       else {"ok": False, "error": "no bench"})
        elif path == "/api/rl/stop":
            self._json(200, BENCH.rl_stop() if BENCH
                       else {"ok": False, "error": "no bench"})
        elif path in ("/api/rl/stand", "/api/rl/lower"):
            if not BENCH:
                self._json(400, {"ok": False, "error": "no bench"})
            else:
                self._json(200, BENCH.rl_policy_move(
                    mode=path.rsplit("/", 1)[-1]))
        elif path == "/api/rl/walk":
            try:
                data = json.loads(body or "{}") if body else {}
            except ValueError:
                data = {}
            if not BENCH:
                self._json(400, {"ok": False, "error": "no bench"})
            else:
                self._json(200, BENCH.rl_policy_move(
                    mode="walk",
                    vx=float(data.get("vx", 0.03)),
                    vy=float(data.get("vy", 0.0)),
                    duration_s=float(data.get("duration_s", 6.0))))
        elif path == "/api/rl/set_stance":
            try:
                data = json.loads(body or "{}") if body else {}
            except ValueError:
                data = {}
            if not BENCH:
                self._json(400, {"ok": False, "error": "no bench"})
            else:
                self._json(200, BENCH.rl_set_stance(
                    hip_deg=float(data.get("hip_deg", -20)),
                    knee_deg=float(data.get("knee_deg", 55)),
                    seconds=float(data.get("seconds", 10)),
                    yaw_deg=float(data.get("yaw_deg", 0)),
                    force=bool(data.get("force", False)),
                ))
        elif path == "/api/rl/probe_dynamics":
            try:
                data = json.loads(body or "{}") if body else {}
            except ValueError:
                data = {}
            if not BENCH:
                self._json(400, {"ok": False, "error": "no bench"})
            else:
                self._json(200, BENCH.rl_probe_dynamics(
                    amp_deg=float(data.get("amp_deg", 10)),
                    axis=str(data.get("axis", "all")),
                    soft_torque=int(data.get("soft_torque", 450)),
                ))
        elif path == "/cal":
            try:
                z = float(body.strip())
            except ValueError:
                self._send(400, "bad value")
                return
            CAL["stand_z"] = z
            saved = save_cal(CAL)
            LINK.send(f"Z {z:.1f}")
            self._send(200 if saved else 500, "saved" if saved else "save failed")
        elif path == "/cal_tuck":
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
    global LINK, DRIVE, BENCH, HTTPS_PORT
    ap = argparse.ArgumentParser(description="STS3215 hexapod web control panel")
    ap.add_argument("--bind", default="0.0.0.0", help="HTTP bind address")
    ap.add_argument("--http-port", type=int, default=8080, dest="http_port")
    ap.add_argument("--https-port", type=int, default=8443, dest="https_port",
                    help="HTTPS port for browser Xbox/Gamepad API (default 8443)")
    ap.add_argument("--port", default=None,
                    help="Feetech bus serial port (auto-detect if omitted)")
    ap.add_argument("--baud", type=int, default=1_000_000)
    ap.add_argument("--dry-run", action="store_true",
                    help="serve UI without opening the servo bus")
    ap.add_argument("--log-host", default=None,
                    help="UDP host for timestamped event stream "
                         "(default: HEXAPOD_LOG_HOST env)")
    ap.add_argument("--log-port", type=int, default=None,
                    help="UDP port for event stream (default 9377)")
    args = ap.parse_args()

    try:
        from event_log import configure, emit, install_print_hook
        cfg = configure(host=args.log_host, port=args.log_port)
        install_print_hook()
        emit("boot", "web_drive start", src="web", data=cfg)
        sinks = cfg.get("udp") or []
        print(f"[log] events → {cfg['path']}  udp→{sinks or 'auto'}  "
              f"beacon:{cfg.get('beacon_port')}")
    except Exception as e:
        print(f"[log] event_log unavailable: {e}")

    DRIVE = DriveController(port=args.port, baud=args.baud, dry_run=args.dry_run)
    DRIVE.start()
    try:
        _main_after_bus(args)
    except Exception as e:
        # Bus is up but the web stack died — put the error on the TFT so a
        # headless robot isn't just silently stuck (service will restart).
        _show_fatal_on_tft(e)
        raise


def _show_fatal_on_tft(exc: BaseException) -> None:
    try:
        from status_display import _wrap
        bus = DRIVE.bus if DRIVE else None
        if bus is None or not hasattr(bus, "display_job"):
            return
        body = _wrap(f"{type(exc).__name__}: {exc}", 26, 4)
        while len(body) < 4:
            body.append("")
        bus.display_job(["WEB ERROR"] + body + ["restarting..."],
                        pct=-1, timeout=6.0)
    except Exception:
        pass


def _main_after_bus(args) -> None:
    global LINK, BENCH, HTTPS_PORT
    BENCH = BenchAPI(DRIVE)
    DRIVE.bench = BENCH
    LINK = Link(DRIVE)
    if not args.dry_run:
        # StatusDisplay hard-reinits the ST7789 at start (covers ribbon
        # reseat while MCU still thinks the panel is up).
        BENCH.start_status_display()
        print("[web] TFT status display started (MCU ST7789)")
        BENCH.start_servo_watch()
        print("[web] servo watch started (liveness + 65C cutoff)")

    if ensure_cert():
        ctx = ssl.SSLContext(ssl.PROTOCOL_TLS_SERVER)
        ctx.load_cert_chain(CERT_FILE, KEY_FILE)
        for hp in (args.https_port, 8443, 9443):
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

    srv = ThreadingHTTPServer((args.bind, args.http_port), Handler)
    print(f"[web] serving on http://{args.bind}:{args.http_port}")
    print("[web] open via adb:  adb forward tcp:8080 tcp:8080")
    print("[web]              then http://127.0.0.1:8080")
    if HTTPS_PORT:
        print(f"[web] Xbox/gamepad: adb forward tcp:{HTTPS_PORT} tcp:{HTTPS_PORT}")
        print(f"[web]              then https://127.0.0.1:{HTTPS_PORT}  (accept cert)")
    try:
        srv.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        if BENCH:
            BENCH.stop_status_display()
        srv.server_close()
        DRIVE.close()


if __name__ == "__main__":
    main()
