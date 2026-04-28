"""
*=========================================================================
* main.py
* RoverPi Flask Dashboard + Autonomous Controller
* Created on: April 22, 2026
*
* Modes
* ─────
* AUTO   : PI ML agent drives, logs experience, trains in background
* MANUAL : Dashboard/human control; if idle > IDLE_TIMEOUT_S → AUTO
*
* ESP32 motor/sensor controller for RoverPi autonomous rover.
* Architecture:Raspberry Pi --UART--> ESP32
* Authors: Michael Kane
*
* RoverPi ESP32 Controller — Clean Multi-File Version
*
* Raspberry Pi = brain
* ESP32 = body controller
* UART between Pi and ESP32
* ESP32 owns motors, ultrasonic sensor, scanner servo, safety stop, and FSM
* Pi sends commands like "FORWARD", "BACKWARD", "LEFT", "RIGHT", "STOP", "SCAN", etc.
* ESP32 motor/sensor controller for RoverPi autonomous rover.
*
* Architecture:
*   Raspberry Pi --UART--> ESP32
*
* Pi -> ESP32 commands:
*   TICK
*   FORWARD
*   BACKWARD
*   LEFT
*   RIGHT
*   STOP
*   STATUS
*   SCAN
*   SCAN_AT <angle>
*   FAULT_CLEAR
*
* ESP32 -> Pi responses:
*   TOCK
*   OK <CMD>
*   ERR FAULT
*   ERR UNKNOWN
*   ERR HEARTBEAT LOST
*   OBSTACLE STOP
*   STATUS dist=XX.X path=X dir=X fault=X
*   SCAN angle=<d> dist=<cm>
=========================================================================*/
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
PING_INTERVAL_S       = 0.4   # must be well under ESP32 HEARTBEAT_TIMEOUT_MS (500ms)
AUTO_STEP_HZ          = 4
FRAME_W, FRAME_H      = 640, 480

STARTUP_ESP32_TIMEOUT = 10.0   # seconds to wait for ESP32 READY
STARTUP_SENSOR_CHECKS = 5      # number of valid (non -1.0) STATUS reads required
STARTUP_SCAN_WAIT     = 2.0    # seconds to let initial scan complete

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
# Auto loop will not send any motion commands until this is True.
_system_ready = False
_startup_log  = []

def _startup_sequence():
    """
    Runs once in a background thread before auto loop is allowed to drive.
      1. Send STOP immediately
      2. Wait for ESP32 READY
      3. Wait for valid sensor readings (dist > 0)
      4. Centre camera
      5. Trigger initial scan
      6. Switch mode to AUTO and set _system_ready
    """
    global _system_ready

    def log(msg):
        print(f"[STARTUP] {msg}")
        _startup_log.append(msg)

    log("Startup sequence begun — motors held")
    esp32.send("STOP")
    time.sleep(0.2)

    # 1. Wait for ESP32 READY
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

    # 2. Wait for valid sensor data
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
                    if dist > 0:
                        valid_reads += 1
                        log(f"Valid read {valid_reads}/{STARTUP_SENSOR_CHECKS}: {dist:.1f}cm")
                    break
                except Exception:
                    pass
        attempts += 1

    sensor_ok = valid_reads >= STARTUP_SENSOR_CHECKS
    if not sensor_ok:
        log("WARNING: Sensor not validated — check HC-SR04 wiring")
    else:
        log("Sensor validated")

    # 3. Centre camera
    log("Centering camera...")
    pantilt.center()
    time.sleep(0.5)

    # 4. Initial scan for map context
    log("Running initial scan...")
    esp32.send("SCAN")
    time.sleep(STARTUP_SCAN_WAIT)

    # 5. Done
    _system_ready = sensor_ok
    set_mode("AUTO")
    log("Startup complete — AUTO mode active" if sensor_ok
        else "Startup complete with warnings — sensor unhealthy, AUTO blocked")

# ── mode state ────────────────────────────────────────────────────────────────
_mode              = "MANUAL"   # starts MANUAL — _startup_sequence sets AUTO
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
# Replace read_battery_stub() body when INA219 / MAX17043 is wired to Pi I2C.
def read_battery_stub() -> dict:
    return {
        "voltage":  None,
        "percent":  None,
        "charging": None,
        "stub":     True,
        "note":     "Battery monitor hardware not yet installed"
    }

# Replace read_gyro_stub() body when MPU6050 / ICM-42688 is wired to Pi I2C
# or when the ESP32 streams IMU data over UART.
def read_gyro_stub() -> dict:
    return {
        "roll":  None,
        "pitch": None,
        "yaw":   None,
        "accel": {"x": None, "y": None, "z": None},
        "stub":  True,
        "note":  "IMU hardware not yet installed"
    }

# ── scan data store ───────────────────────────────────────────────────────────
_scan_lock   = threading.Lock()
_scan_latest = {}   # {angle_deg: dist_cm}

def _parse_scan_messages():
    """
    Watches ESP32 message history for SCAN lines.
    Feeds both the occupancy map and rover_env scan readings.
    Uses get_history() so no message is ever missed.
    """
    seen = set()   # track already-processed messages by content+index
    while True:
        for msg in esp32.get_history():
            if msg.startswith("SCAN") and msg not in seen:
                seen.add(msg)
                try:
                    parts = msg.split()
                    angle = int(parts[1].split("=")[1])
                    dist  = float(parts[2].split("=")[1])
                    with _scan_lock:
                        _scan_latest[angle] = dist
                    # Feed env so safe_action() has side distance data
                    env.update_scan(angle, dist)
                    # Feed occupancy map (angle relative to rover heading)
                    occ_map.update_sweep({angle - 90: dist})
                except Exception:
                    pass
        # Keep seen set from growing unbounded
        if len(seen) > 200:
            seen.clear()
        time.sleep(0.05)

# ── background threads ────────────────────────────────────────────────────────

def _ping_loop():
    while True:
        esp32.send("PING")
        time.sleep(PING_INTERVAL_S)

def _esp_listener_loop():
    """
    Forwards ESP32 messages into the RL env.
    Prioritises STATUS and OBSTACLE messages over PONG
    so the env always has the most useful data.
    """
    while True:
        history = esp32.get_history()
        # Find the most recent meaningful message (not just PONG)
        for msg in reversed(history):
            if any(k in msg for k in ("STATUS", "OBSTACLE", "ERR", "SCAN")):
                env.update_esp_message(msg)
                break
        time.sleep(0.05)

def _sensor_healthy() -> bool:
    """
    Returns True if any recent ESP32 message contains a valid distance.
    Scans full history so PONG spam can't hide a valid STATUS response.
    """
    for msg in reversed(esp32.get_history()):
        if "dist=" in msg:
            try:
                dist = float(msg.split("dist=")[1].split()[0])
                return dist > 0.0
            except Exception:
                pass
    return False

def _auto_loop():
    # Wait until startup sequence has completed before doing anything
    while not _system_ready:
        time.sleep(0.5)

    obs  = env.reset()
    step = 0

    while True:
        with _mode_lock:
            mode     = _mode
            last_inp = _last_manual_input

        # ── idle timeout: MANUAL → AUTO ──────────────────────────────
        if mode == "MANUAL":
            if time.time() - last_inp > IDLE_TIMEOUT_S:
                print("[Mode] Idle timeout — resuming AUTO")
                set_mode("AUTO")
                obs = env.reset()
            time.sleep(0.5)
            continue

        # ── sensor health gate ────────────────────────────────────────
        # Don't drive if sensor is returning -1.0 — would cause thrashing
        if not _sensor_healthy():
            print("[AUTO] Sensor unhealthy (dist=-1.0) — holding, requesting STATUS")
            esp32.send("STATUS")
            esp32.send("STOP")
            time.sleep(1.0)
            continue

        # ── RL step ───────────────────────────────────────────────────
        action = agent.select_action(obs)
        next_obs, reward, _, info = env.step(action)
        agent.store(obs, action, reward, next_obs)
        occ_map.move(action)

        if step % 5 == 0:
            esp32.send("STATUS")

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
        <button onclick="cmd('forward')">▲ FWD</button>
        <div></div>
        <button onclick="cmd('left')">◄ LEFT</button>
        <button class="danger" onclick="cmd('stop')">■ STOP</button>
        <button onclick="cmd('right')">RIGHT ►</button>
        <div></div>
        <button onclick="cmd('backward')">▼ BACK</button>
        <div></div>
      </div>
    </div>

    <div class="panel">
      <h2>Gimbal</h2>
      <div class="slider-row"><span>Pan</span><span id="pan-v">90°</span></div>
      <input type="range" min="0" max="180" value="90" id="pan"
             oninput="document.getElementById('pan-v').textContent=this.value+'°'"
             onchange="fetch('/cam/pan/'+this.value)">
      <div class="slider-row" style="margin-top:8px"><span>Tilt</span><span id="tilt-v">90°</span></div>
      <input type="range" min="15" max="145" value="90" id="tilt"
             style="accent-color:#fbbf24"
             oninput="document.getElementById('tilt-v').textContent=this.value+'°'"
             onchange="fetch('/cam/tilt/'+this.value)">
      <div class="btn-row" style="margin-top:8px">
        <button onclick="fetch('/cam/center');document.getElementById('pan').value=90;
                         document.getElementById('tilt').value=90;
                         document.getElementById('pan-v').textContent='90°';
                         document.getElementById('tilt-v').textContent='90°'">
          ⊙ Center
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
      <div class="stat"><span>ESP32</span><span id="t-esp" style="font-size:9px;max-width:160px;overflow:hidden;text-overflow:ellipsis">—</span></div>
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
      addLog('CMD '+c.toUpperCase()+' → '+d.esp32, r.ok);
    }

    async function setMode(m) {
      const url = m === 'AUTO' ? '/auto/start' : '/auto/stop';
      await fetch(url);
      addLog('Mode → '+m);
    }

    async function poll() {
      try {
        const r = await fetch('/status');
        const d = await r.json();
        document.getElementById('online').className = 'up';
        document.getElementById('t-dist').textContent =
          d.esp32_msg && d.esp32_msg.includes('dist=')
            ? d.esp32_msg.split('dist=')[1].split(' ')[0]+'cm' : '—';
        document.getElementById('t-path').textContent =
          d.esp32_msg && d.esp32_msg.includes('path=1') ? 'CLEAR' : 'BLOCKED';
        document.getElementById('t-mode').textContent   = d.mode;
        document.getElementById('t-ready').textContent  = d.system_ready ? 'YES' : 'NO';
        document.getElementById('t-steps').textContent  = d.agent_steps;
        document.getElementById('t-eps').textContent    = d.epsilon;
        document.getElementById('t-esp').textContent    = d.esp32_msg || '—';
        document.getElementById('btn-auto').className   =
          d.mode === 'AUTO' ? 'mode-active' : '';
        document.getElementById('btn-manual').className =
          d.mode === 'MANUAL' ? 'mode-active' : '';
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
        "startup_log":   _startup_log[-5:],   # last 5 startup messages
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

# ── battery ───────────────────────────────────────────────────────────────────

@app.route("/battery")
def battery():
    # Swap read_battery_stub() for real INA219/MAX17043 reads when wired
    return jsonify(read_battery_stub())

# ── gyro ──────────────────────────────────────────────────────────────────────

@app.route("/gyro")
def gyro():
    # Swap read_gyro_stub() for real MPU6050/ICM reads when wired
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
    allowed = {
        "forward", "backward", "left", "right", "stop",
        "scan", "fault_clear"
    }
    if cmd.lower() not in allowed:
        return jsonify({"error": "invalid command"}), 400
    set_mode("MANUAL")
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
    """Absolute pan angle for slider control (0–180)."""
    angle = max(0, min(180, angle))
    pantilt.set_angle(pantilt.PAN, angle)
    pantilt.pan_angle = angle
    return jsonify({"pan": pantilt.pan_angle})

@app.route("/cam/tilt/<int:angle>")
def cam_tilt_absolute(angle):
    """Absolute tilt angle for slider control (15–145)."""
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