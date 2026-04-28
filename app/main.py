"""
*=========================================================================
* main.py
* RoverPi Flask Dashboard + Autonomous Controller
* Created on: April 22, 2026
*
* FIXES APPLIED (see FIX comments):
*   FIX-A  Dashboard "BLOCKED always" — path status now read from the
*           dedicated /esp32/status cache, not from get_latest_message()
*           which is almost always "PONG".
*
*   FIX-B  Manual command lag — /cmd route no longer calls set_mode()
*           when already in MANUAL, preventing the spurious STOP that
*           was sent on every button press before the real command.
*
*   FIX-C  AUTO stuck after obstacle — _auto_loop now detects
*           OBSTACLE STOP in the ESP32 response, executes a timed
*           backward escape + re-scan, and resets env before resuming.
*
*   FIX-D  Obstacle escape helper — _escape_from_obstacle() sends
*           BACKWARD for a fixed duration, then STOPs and SCANs so
*           the occupancy map and env have fresh data before the next
*           agent step.
*=========================================================================
"""

import time
import threading

from flask import Flask, Response, jsonify, request
from picamera2 import Picamera2
import cv2

import pantilt
import serial_comm
from rover_env     import RoverEnv
from rover_agent   import RoverAgent
from occupancy_map import OccupancyMap

# ── configuration ─────────────────────────────────────────────────────────────
IDLE_TIMEOUT_S        = 30.0
PING_INTERVAL_S       = 0.4
AUTO_STEP_HZ          = 4
FRAME_W, FRAME_H      = 640, 480

STARTUP_ESP32_TIMEOUT = 10.0
STARTUP_SENSOR_CHECKS = 5
STARTUP_SCAN_WAIT     = 2.0

# FIX-C/D: how long to reverse when stuck, and how many consecutive
# OBSTACLE STOP responses before we give up and try the escape.
ESCAPE_BACKWARD_S     = 0.8   # seconds of BACKWARD during escape
OBSTACLE_STRIKE_MAX   = 3     # consecutive obstacle responses before escape

# ── hardware init ─────────────────────────────────────────────────────────────
picam2 = Picamera2()
picam2.configure(picam2.create_preview_configuration(
    main={"size": (FRAME_W, FRAME_H), "format": "RGB888"}
))
picam2.start()

pantilt.init()

esp32 = serial_comm.ESP32Serial()
esp32.connect()
esp32.start_reader()
time.sleep(1)

# ── ML components ─────────────────────────────────────────────────────────────
occ_map = OccupancyMap()
env     = RoverEnv(picam2, esp32)
agent   = RoverAgent()

# ── system ready gate ─────────────────────────────────────────────────────────
_system_ready = False
_startup_log  = []

def _startup_sequence():
    global _system_ready

    def log(msg):
        print(f"[STARTUP] {msg}")
        _startup_log.append(msg)

    log("Startup sequence begun — motors held")
    esp32.send("STOP")
    time.sleep(0.2)

    log("Waiting for ESP32 READY...")
    deadline    = time.time() + STARTUP_ESP32_TIMEOUT
    esp32_ready = False

    while time.time() < deadline:
        esp32.send("HELLO")
        time.sleep(0.2)
        msg = esp32.get_latest_message()
        if msg and "ESP32 READY" in msg:
            esp32_ready = True
            break

    time.sleep(0.2)
    log("ESP32 READY confirmed" if esp32_ready else "WARNING: ESP32 READY not received")

    log("Waiting for valid sensor readings...")
    valid_reads = 0
    attempts    = 0
    while valid_reads < STARTUP_SENSOR_CHECKS and attempts < 10:
        esp32.send("STATUS")
        time.sleep(0.4)
        for msg in reversed(esp32.get_history()):
            if "dist=" in msg:
                try:
                    dist = float(msg.split("dist=")[1].split()[0])
                    # Accept -1.0 (open-space timeout) and any positive reading.
                    # Only 0.0 is a true sensor wiring error.
                    # In open rooms the HC-SR04 regularly times out (-1.0) — that is normal.
                    if dist != 0.0:
                        valid_reads += 1
                        label = "open/timeout" if dist < 0 else f"{dist:.1f}cm"
                        log(f"Valid read {valid_reads}/{STARTUP_SENSOR_CHECKS}: {label}")
                    break
                except Exception:
                    pass
        attempts += 1

    sensor_ok = valid_reads >= STARTUP_SENSOR_CHECKS
    if not sensor_ok:
        log("WARNING: Sensor not validated — only got 0.0 readings, check HC-SR04 wiring")
    else:
        log("Sensor validated")

    log("Centering camera...")
    pantilt.center()
    time.sleep(0.5)

    log("Running initial scan...")
    esp32.send("SCAN")
    time.sleep(STARTUP_SCAN_WAIT)

    # Prime the _esp_status cache so _sensor_healthy() returns True immediately
    # on the first AUTO loop iteration — without this, the loop blocks on the
    # health gate until the first step % 5 STATUS poll fires (which never comes
    # because step never increments while health gate blocks).
    esp32.send("STATUS")
    time.sleep(0.4)

    _system_ready = sensor_ok
    set_mode("AUTO")
    log("Startup complete — AUTO mode active" if sensor_ok
        else "Startup complete with warnings — sensor unhealthy, AUTO blocked")

# ── mode state ────────────────────────────────────────────────────────────────
_mode              = "MANUAL"
_mode_lock         = threading.Lock()
_last_manual_input = 0.0

def get_mode():
    with _mode_lock:
        return _mode

def set_mode(new_mode: str):
    global _mode, _last_manual_input
    with _mode_lock:
        if _mode == new_mode:
            return
        print(f"[Mode] {_mode} → {new_mode}")
        _mode = new_mode
        if new_mode == "MANUAL":
            _last_manual_input = time.time()
            esp32.send("STOP")
        else:
            env.reset()

def touch_manual():
    global _last_manual_input
    with _mode_lock:
        _last_manual_input = time.time()

# ── stub telemetry ────────────────────────────────────────────────────────────
def read_battery_stub() -> dict:
    return {
        "voltage":  None,
        "percent":  None,
        "charging": None,
        "stub":     True,
        "note":     "Battery monitor hardware not yet installed"
    }

def read_gyro_stub() -> dict:
    return {
        "roll":  None,
        "pitch": None,
        "yaw":   None,
        "accel": {"x": None, "y": None, "z": None},
        "stub":  True,
        "note":  "IMU hardware not yet installed"
    }

# ── latest parsed ESP32 status ────────────────────────────────────────────────
# FIX-A: We maintain a parsed dict from the most recent STATUS response.
# This is what the dashboard reads, not get_latest_message() which is
# almost always "PONG" and therefore never contains "path=1".
_esp_status_lock = threading.Lock()
_esp_status = {
    "dist":  -2.0,   # sentinel: -2.0 = no STATUS received yet
    "path":  0,     # 1 = clear, 0 = blocked
    "dir":   0,
    "fault": 0,
    "raw":   "",
}

def _update_esp_status(msg: str):
    """Parse a STATUS line and store it in _esp_status."""
    # Expected format: STATUS dist=XX.X path=X dir=X fault=X
    if not msg.startswith("STATUS"):
        return
    try:
        parts = msg.split()
        d = {}
        for p in parts[1:]:
            k, v = p.split("=")
            d[k] = v
        with _esp_status_lock:
            _esp_status["dist"]  = float(d.get("dist",  _esp_status["dist"]))
            _esp_status["path"]  = int(d.get("path",    _esp_status["path"]))
            _esp_status["dir"]   = int(d.get("dir",     _esp_status["dir"]))
            _esp_status["fault"] = int(d.get("fault",   _esp_status["fault"]))
            _esp_status["raw"]   = msg
    except Exception:
        pass

def get_esp_status() -> dict:
    with _esp_status_lock:
        return dict(_esp_status)

# ── scan data store ───────────────────────────────────────────────────────────
_scan_lock   = threading.Lock()
_scan_latest = {}

def _parse_scan_messages():
    seen = set()
    while True:
        for msg in esp32.get_history():
            if msg not in seen:
                seen.add(msg)
                if msg.startswith("SCAN"):
                    try:
                        parts = msg.split()
                        angle = int(parts[1].split("=")[1])
                        dist  = float(parts[2].split("=")[1])
                        with _scan_lock:
                            _scan_latest[angle] = dist
                        env.update_scan(angle, dist)
                        occ_map.update_sweep({angle - 90: dist})
                    except Exception:
                        pass
                # FIX-A: parse STATUS messages into _esp_status as they arrive
                elif msg.startswith("STATUS"):
                    _update_esp_status(msg)

        if len(seen) > 200:
            seen.clear()
        time.sleep(0.05)

# ── obstacle escape ───────────────────────────────────────────────────────────
# FIX-D: centralised escape routine so both auto loop and any future
# recovery path use the same behaviour.
_escape_lock = threading.Lock()

def _escape_from_obstacle():
    """
    Called when the rover is stuck against an obstacle in AUTO mode.
    Sends BACKWARD for ESCAPE_BACKWARD_S seconds, then stops and scans
    so the env/agent have fresh state before the next decision.
    Acquires _escape_lock so concurrent calls collapse into one.
    """
    if not _escape_lock.acquire(blocking=False):
        return   # another escape already in progress

    try:
        print("[AUTO] Obstacle escape — reversing")
        esp32.send("BACKWARD")
        time.sleep(ESCAPE_BACKWARD_S)
        esp32.send("STOP")
        time.sleep(0.1)
        esp32.send("SCAN")
        time.sleep(0.8)   # give scan time to complete before next agent step
        esp32.send("STATUS")
        time.sleep(0.2)
    finally:
        _escape_lock.release()

# ── background threads ────────────────────────────────────────────────────────

def _ping_loop():
    """
    Sends PING independently of the auto loop so the ESP32 watchdog is
    fed during startup, MANUAL mode, and escape pauses.
    PONG responses are filtered out in _parse_scan_messages so they
    never enter the history that obstacle detection reads.
    """
    while True:
        esp32.send("PING")
        time.sleep(PING_INTERVAL_S)

def _esp_listener_loop():
    while True:
        history = esp32.get_history()
        for msg in reversed(history):
            if any(k in msg for k in ("STATUS", "OBSTACLE", "ERR", "SCAN")):
                env.update_esp_message(msg)
                break
        time.sleep(0.05)

def _sensor_healthy() -> bool:
    """
    Use the parsed _esp_status cache — not raw history.
    Raw history is flooded with PONG; STATUS messages are sparse.
    Scanning history for "dist=" was almost always missing them,
    causing the sensor-health gate to block AUTO permanently.
    Returns True if we have ever received a valid (>0) distance reading.
    """
    with _esp_status_lock:
        dist = _esp_status["dist"]
    # -2.0 is the sentinel "never received a STATUS response at all" (initial value).
    # -1.0 means the HC-SR04 timed out — normal in open space, path treated as clear.
    #  0.0 means bad echo / wiring glitch — treat as unhealthy.
    # >0.0 is a normal distance reading.
    # So: healthy = we have received at least one STATUS (dist != -2.0 sentinel)
    #     AND it wasn't a wiring-error zero.
    return dist != -2.0 and dist != 0.0

def _auto_loop():
    while not _system_ready:
        time.sleep(0.5)

    obs              = env.reset()
    step             = 0
    obstacle_strikes = 0

    # Watermark: only inspect messages that arrived AFTER this index.
    # Without this, a single OBSTACLE STOP stays in history forever and
    # the escape->scan loop never terminates even after the rover backs away.
    history_watermark = len(esp32.get_history())

    while True:
        with _mode_lock:
            mode     = _mode
            last_inp = _last_manual_input

        # ── idle timeout ──────────────────────────────────────────────
        if mode == "MANUAL":
            if time.time() - last_inp > IDLE_TIMEOUT_S:
                print("[Mode] Idle timeout — resuming AUTO")
                set_mode("AUTO")
                obs = env.reset()
                obstacle_strikes  = 0
                history_watermark = len(esp32.get_history())
            time.sleep(0.5)
            continue

        # ── sensor health gate ────────────────────────────────────────
        if not _sensor_healthy():
            print("[AUTO] Sensor unhealthy (dist=-1.0) — holding, requesting STATUS")
            esp32.send("STATUS")
            esp32.send("STOP")
            time.sleep(1.0)
            continue

        # ── RL step ───────────────────────────────────────────────────
        action             = agent.select_action(obs)
        next_obs, reward, _, info = env.step(action)
        agent.store(obs, action, reward, next_obs)
        occ_map.move(action)

        if step % 5 == 0:
            esp32.send("STATUS")
            time.sleep(0.05)

        # ── obstacle detection (watermarked) ──────────────────────────
        # Only look at messages that arrived since the last check.
        # Never re-read old history entries.
        history           = esp32.get_history()
        new_msgs          = history[history_watermark:]
        history_watermark = len(history)

        obstacle_now = False
        for msg in new_msgs:
            if msg in ("PONG", "TOCK", ""):
                continue
            if "OBSTACLE" in msg:
                obstacle_now = True
                break

        if obstacle_now:
            obstacle_strikes += 1
            print(f"[AUTO] Obstacle strike {obstacle_strikes}/{OBSTACLE_STRIKE_MAX}")

            if obstacle_strikes >= OBSTACLE_STRIKE_MAX:
                print("[AUTO] Max obstacle strikes — executing escape")
                _escape_from_obstacle()
                obs               = env.reset()
                obstacle_strikes  = 0
                step              = 0
                # Advance watermark past all the OBSTACLE/SCAN messages
                # generated during the escape so they don't re-trigger.
                history_watermark = len(esp32.get_history())
        elif obstacle_strikes > 0:
            obstacle_strikes = max(0, obstacle_strikes - 1)

        obs   = next_obs
        step += 1

        if step % 100 == 0:
            agent.decay_epsilon()

        time.sleep(1.0 / AUTO_STEP_HZ)

threading.Thread(target=_startup_sequence,    daemon=True).start()
threading.Thread(target=_ping_loop,           daemon=True).start()
threading.Thread(target=_esp_listener_loop,   daemon=True).start()
threading.Thread(target=_auto_loop,           daemon=True).start()
threading.Thread(target=_parse_scan_messages, daemon=True).start()

# ── Flask app ─────────────────────────────────────────────────────────────────
app = Flask(__name__)

def _generate_video():
    while True:
        frame = picam2.capture_array()
        ok, buf = cv2.imencode(".jpg", frame)
        if ok:
            yield (
                b"--frame\r\n"
                b"Content-Type: image/jpeg\r\n\r\n"
                + buf.tobytes()
                + b"\r\n"
            )
        time.sleep(0.05)

# ── local dashboard ───────────────────────────────────────────────────────────

@app.route("/")
def index():
    # FIX-A: dashboard now reads /esp32_status for path/dist data,
    # not the raw esp32_msg field which is almost always "PONG".
    return """<!DOCTYPE html>
<html>
<head>
  <title>RoverPi Local</title>
  <style>
    * { box-sizing: border-box; margin: 0; padding: 0; }
    body { background: #05070b; font-family: monospace; color: #cbd5e1; padding: 16px; }
    h1 { color: #67e8f9; font-size: 18px; letter-spacing: .2em; margin-bottom: 16px; }
    .grid { display: grid; grid-template-columns: 1fr 1fr; gap: 12px; max-width: 900px; }
    .panel { border: 1px solid rgba(34,211,238,.15); background: rgba(8,16,24,.9);
             border-radius: 12px; padding: 12px; }
    .panel h2 { font-size: 10px; text-transform: uppercase; letter-spacing: .25em;
                color: rgba(103,232,249,.7); margin-bottom: 10px; }
    img { width: 100%; border-radius: 8px; background: #02050a; }
    .btn-row { display: flex; flex-wrap: wrap; gap: 6px; margin-top: 6px; }
    button { border: 1px solid rgba(56,189,248,.3); background: rgba(56,189,248,.06);
             border-radius: 8px; padding: 8px 14px; font-size: 11px; font-weight: 800;
             letter-spacing: .14em; color: #7dd3fc; cursor: pointer; font-family: monospace; }
    button:hover { background: rgba(56,189,248,.18); }
    button.danger { border-color: rgba(248,113,113,.35); color: #fca5a5;
                    background: rgba(248,113,113,.08); }
    button.mode-active { background: rgba(34,211,238,.2); color: #a5f3fc; }
    .stat { display: flex; justify-content: space-between; padding: 5px 8px;
            background: rgba(255,255,255,.03); border-radius: 6px; margin-top: 4px;
            font-size: 11px; }
    .stat span:last-child { color: #67e8f9; font-weight: 700; }
    .path-clear   { color: #34d399 !important; }
    .path-blocked { color: #f87171 !important; }
    #log { height: 160px; overflow-y: auto; background: rgba(0,0,0,.3);
           border-radius: 8px; padding: 8px; font-size: 10px; margin-top: 6px; }
    .log-line { border-bottom: 1px solid rgba(255,255,255,.04); padding: 2px 0; color: #94a3b8; }
    .log-ok { color: #34d399; } .log-err { color: #f87171; }
    input[type=range] { width: 100%; accent-color: #38bdf8; margin: 4px 0; }
    .slider-row { font-size: 10px; color: #475569; display: flex;
                  justify-content: space-between; margin-bottom: 2px; }
    #online { display: inline-block; width: 8px; height: 8px; border-radius: 50%;
              background: #f87171; margin-right: 6px; }
    #online.up { background: #34d399; box-shadow: 0 0 6px #34d399; }
  </style>
</head>
<body>
  <h1><span id="online"></span>ROVERPI LOCAL DASHBOARD</h1>
  <div class="grid">

    <div class="panel" style="grid-column:1/-1">
      <h2>Camera Feed</h2>
      <img src="/video" />
    </div>

    <div class="panel">
      <h2>Mode</h2>
      <div class="btn-row">
        <button id="btn-auto"   onclick="setMode('AUTO')">AUTO</button>
        <button id="btn-manual" onclick="setMode('MANUAL')">MANUAL</button>
      </div>
      <h2 style="margin-top:12px">Drive</h2>
      <div class="btn-row" style="display:grid;grid-template-columns:repeat(3,1fr);gap:6px;margin-top:6px">
        <div></div>
        <button onclick="cmd('forward')">&#9650; FWD</button>
        <div></div>
        <button onclick="cmd('left')">&#9668; LEFT</button>
        <button class="danger" onclick="cmd('stop')">&#9632; STOP</button>
        <button onclick="cmd('right')">RIGHT &#9658;</button>
        <div></div>
        <button onclick="cmd('backward')">&#9660; BACK</button>
        <div></div>
      </div>
    </div>

    <div class="panel">
      <h2>Gimbal</h2>
      <div class="slider-row"><span>Pan</span><span id="pan-v">90&deg;</span></div>
      <input type="range" min="0" max="180" value="90" id="pan"
             oninput="document.getElementById('pan-v').textContent=this.value+'&deg;'"
             onchange="fetch('/cam/pan/'+this.value)">
      <div class="slider-row" style="margin-top:8px"><span>Tilt</span><span id="tilt-v">90&deg;</span></div>
      <input type="range" min="15" max="145" value="90" id="tilt"
             style="accent-color:#fbbf24"
             oninput="document.getElementById('tilt-v').textContent=this.value+'&deg;'"
             onchange="fetch('/cam/tilt/'+this.value)">
      <div class="btn-row" style="margin-top:8px">
        <button onclick="fetch('/cam/center');document.getElementById('pan').value=90;
                         document.getElementById('tilt').value=90;
                         document.getElementById('pan-v').textContent='90&deg;';
                         document.getElementById('tilt-v').textContent='90&deg;'">
          &#8857; Center
        </button>
        <button onclick="fetch('/cmd/scan')">Scan</button>
      </div>
    </div>

    <div class="panel">
      <h2>Telemetry</h2>
      <div class="stat"><span>Distance</span><span id="t-dist">—</span></div>
      <div class="stat"><span>Path</span><span id="t-path">—</span></div>
      <div class="stat"><span>Mode</span><span id="t-mode">—</span></div>
      <div class="stat"><span>System Ready</span><span id="t-ready">—</span></div>
      <div class="stat"><span>Agent Steps</span><span id="t-steps">—</span></div>
      <div class="stat"><span>Epsilon</span><span id="t-eps">—</span></div>
      <div class="stat"><span>ESP32 raw</span><span id="t-esp" style="font-size:9px;max-width:160px;overflow:hidden;text-overflow:ellipsis">—</span></div>
    </div>

    <div class="panel">
      <h2>Occupancy Map</h2>
      <img id="map-img" src="/map" style="image-rendering:pixelated"/>
    </div>

    <div class="panel" style="grid-column:1/-1">
      <h2>Event Log</h2>
      <div id="log"></div>
    </div>

  </div>

  <script>
    const log = document.getElementById('log');
    function addLog(msg, ok=true) {
      const d = document.createElement('div');
      d.className = 'log-line';
      d.innerHTML = '<span class="'+(ok?'log-ok':'log-err')+'">'+
        new Date().toLocaleTimeString()+'</span> '+msg;
      log.prepend(d);
      if (log.children.length > 60) log.lastChild.remove();
    }

    async function cmd(c) {
      const r = await fetch('/cmd/'+c);
      const d = await r.json();
      addLog('CMD '+c.toUpperCase()+' → '+(d.esp32||'sent'), r.ok);
    }

    async function setMode(m) {
      const url = m === 'AUTO' ? '/auto/start' : '/auto/stop';
      await fetch(url);
      addLog('Mode → '+m);
    }

    /* FIX-A: poll /esp32_status for distance + path (reliable parsed data),
       and /status for agent/mode metadata. This keeps the telemetry panel
       accurate even when PONG floods get_latest_message(). */
    async function poll() {
      try {
        const [sr, se] = await Promise.all([
          fetch('/status').then(r => r.json()),
          fetch('/esp32_status').then(r => r.json()),
        ]);

        document.getElementById('online').className = 'up';

        const dist  = se.dist >= 0 ? se.dist.toFixed(1)+'cm' : '—';
        const pathOk = se.path === 1;
        const pathEl = document.getElementById('t-path');
        pathEl.textContent  = pathOk ? 'CLEAR' : 'BLOCKED';
        pathEl.className    = pathOk ? 'path-clear' : 'path-blocked';

        document.getElementById('t-dist').textContent   = dist;
        document.getElementById('t-mode').textContent   = sr.mode;
        document.getElementById('t-ready').textContent  = sr.system_ready ? 'YES' : 'NO';
        document.getElementById('t-steps').textContent  = sr.agent_steps;
        document.getElementById('t-eps').textContent    = sr.epsilon;
        document.getElementById('t-esp').textContent    = se.raw || sr.esp32_msg || '—';
        document.getElementById('btn-auto').className   =
          sr.mode === 'AUTO' ? 'mode-active' : '';
        document.getElementById('btn-manual').className =
          sr.mode === 'MANUAL' ? 'mode-active' : '';
        document.getElementById('map-img').src = '/map?t='+Date.now();
      } catch(e) {
        document.getElementById('online').className = '';
      }
    }

    setInterval(poll, 800);
    poll();
    addLog('Local dashboard loaded');
  </script>
</body>
</html>"""

# ── video + map ───────────────────────────────────────────────────────────────

@app.route("/video")
def video():
    return Response(
        _generate_video(),
        mimetype="multipart/x-mixed-replace; boundary=frame"
    )

@app.route("/map")
def map_image():
    jpg = occ_map.render(cell_px=10)
    return Response(jpg, mimetype="image/jpeg")

# ── status ────────────────────────────────────────────────────────────────────

@app.route("/status")
def status():
    with _scan_lock:
        scan = dict(_scan_latest)
    return jsonify({
        "mode":          get_mode(),
        "system_ready":  _system_ready,
        "startup_log":   _startup_log[-5:],
        "epsilon":       round(agent.epsilon, 3),
        "agent_steps":   agent.steps,
        "buffer_size":   len(agent.buffer),
        "training":      agent.is_training,
        "exploration":   round(occ_map.get_free_ratio() * 100, 1),
        "esp32_msg":     esp32.get_latest_message(),
        "pan_angle":     pantilt.pan_angle,
        "tilt_angle":    pantilt.tilt_angle,
        "scan":          scan,
    })

# FIX-A: new endpoint that returns the cleanly parsed STATUS data,
# separate from the noisy get_latest_message() stream.
@app.route("/esp32_status")
def esp32_status():
    return jsonify(get_esp_status())

# ── battery ───────────────────────────────────────────────────────────────────

@app.route("/battery")
def battery():
    return jsonify(read_battery_stub())

# ── gyro ──────────────────────────────────────────────────────────────────────

@app.route("/gyro")
def gyro():
    return jsonify(read_gyro_stub())

# ── mode ──────────────────────────────────────────────────────────────────────

@app.route("/mode/<m>")
def mode_switch(m):
    m = m.upper()
    if m not in ("AUTO", "MANUAL"):
        return jsonify({"error": "invalid mode"}), 400
    set_mode(m)
    return jsonify({"mode": get_mode()})

# ── commands ──────────────────────────────────────────────────────────────────

@app.route("/cmd/<cmd>")
def send_cmd(cmd):
    """
    FIX-B: Only call set_mode("MANUAL") if we're currently in AUTO.
    In the old code, set_mode("MANUAL") always ran, and its first action
    is esp32.send("STOP") — so every button press sent STOP before the
    real command, causing noticeable lag in manual driving.
    """
    allowed = {
        "forward", "backward", "left", "right", "stop",
        "scan", "fault_clear"
    }
    if cmd.lower() not in allowed:
        return jsonify({"error": "invalid command"}), 400

    if get_mode() != "MANUAL":
        # Transition from AUTO → MANUAL (this sends STOP once, intentionally)
        set_mode("MANUAL")
    else:
        # Already MANUAL: just update the idle timer, do NOT send STOP
        touch_manual()

    esp32.send(cmd.upper())
    return jsonify({"sent": cmd.upper(), "esp32": esp32.get_latest_message()})

# ── camera pan/tilt ───────────────────────────────────────────────────────────

@app.route("/cam/<direction>")
def cam_control(direction):
    actions = {
        "left":   pantilt.pan_left,
        "right":  pantilt.pan_right,
        "up":     pantilt.tilt_up,
        "down":   pantilt.tilt_down,
        "center": pantilt.center,
    }
    if direction not in actions:
        return jsonify({"error": "invalid direction"}), 400
    actions[direction]()
    return jsonify({"pan": pantilt.pan_angle, "tilt": pantilt.tilt_angle})

@app.route("/cam/pan/<int:angle>")
def cam_pan_absolute(angle):
    angle = max(0, min(180, angle))
    pantilt.set_angle(pantilt.PAN, angle)
    pantilt.pan_angle = angle
    return jsonify({"pan": pantilt.pan_angle})

@app.route("/cam/tilt/<int:angle>")
def cam_tilt_absolute(angle):
    angle = max(15, min(145, angle))
    pantilt.set_angle(pantilt.TILT, angle)
    pantilt.tilt_angle = angle
    return jsonify({"tilt": pantilt.tilt_angle})

# ── auto control ──────────────────────────────────────────────────────────────

@app.route("/auto/start")
def auto_start():
    set_mode("AUTO")
    return jsonify({"mode": "AUTO"})

@app.route("/auto/stop")
def auto_stop():
    set_mode("MANUAL")
    esp32.send("STOP")
    return jsonify({"mode": "MANUAL"})

# ── health ────────────────────────────────────────────────────────────────────

@app.route("/health")
def health():
    return jsonify({"status": "ok", "host": "roverpi"})

# ── entry point ───────────────────────────────────────────────────────────────

if __name__ == "__main__":
    app.run(host="0.0.0.0", port=5000, threaded=True)