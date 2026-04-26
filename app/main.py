"""
app.py
RoverPi Flask Dashboard + RL Autonomous Controller

Modes
─────
  AUTO   : RL agent drives, logs experience, trains in background
  MANUAL : Dashboard/human control; if idle > IDLE_TIMEOUT_S → AUTO

ESP32 UART protocol (Pi → ESP32):
  FORWARD / BACKWARD / LEFT / RIGHT / STOP / PING / STATUS / FAULT_CLEAR

ESP32 → Pi responses:
  OK <CMD> / ERR FAULT / ERR UNKNOWN / OBSTACLE STOP
  SCAN angle=<deg> dist=<cm>
  STATUS dist=XX.X path=X dir=X fault=X
  PONG
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
PING_INTERVAL_S       = 0.3    # must be well under ESP32 HEARTBEAT_TIMEOUT_MS (500ms)
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
        msg = esp32.get_latest_message()
        if msg and "READY" in msg:
            esp32_ready = True
            break
        esp32.send("PING")
        time.sleep(0.3)
    log("ESP32 READY confirmed" if esp32_ready else "WARNING: ESP32 READY not received")

    # 2. Wait for valid sensor data
    log("Waiting for valid sensor readings...")
    valid_reads = 0
    attempts    = 0
    while valid_reads < STARTUP_SENSOR_CHECKS and attempts < 30:
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
        "scan", "return", "hold", "lights", "fault_clear"
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