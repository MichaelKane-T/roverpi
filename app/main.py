"""
app.py
RoverPi Flask Dashboard + RL Autonomous Controller

Modes
─────
  AUTO   : RL agent drives, logs experience, trains in background
  MANUAL : Dashboard/human control; if idle > IDLE_TIMEOUT_S → AUTO

ESP32 UART protocol (Pi → ESP32):
  FORWARD / BACKWARD / LEFT / RIGHT / STOP / PING / STATUS

ESP32 → Pi responses:
  OK FORWARD / OK STOP / OBSTACLE STOP / ERR FAULT / PONG
  STATUS dist=XX.X path=X drive=X fault=X
"""

import time
import threading
import io

from flask import Flask, Response, jsonify, request
from picamera2 import Picamera2
import cv2

import pantilt
import serial_comm
from rover_env   import RoverEnv
from rover_agent import RoverAgent
from occupancy_map import OccupancyMap

# ── configuration ─────────────────────────────────────────────────────────────
IDLE_TIMEOUT_S   = 30.0    # seconds of no manual input before AUTO resumes
PING_INTERVAL_S  = 2.0     # heartbeat to ESP32
AUTO_STEP_HZ     = 4       # actions per second in AUTO mode (≤ 1000/STEP_DELAY)
FRAME_W, FRAME_H = 640, 480

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

# ── mode state ────────────────────────────────────────────────────────────────
# "AUTO" or "MANUAL"
_mode      = "AUTO"
_mode_lock = threading.Lock()
_last_manual_input = 0.0   # epoch seconds of last dashboard command

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
            # Reset env when re-entering AUTO
            env.reset()

def touch_manual():
    """Record that a manual command was just issued."""
    global _last_manual_input
    with _mode_lock:
        _last_manual_input = time.time()

# ── background threads ────────────────────────────────────────────────────────

def _ping_loop():
    """Sends PING to ESP32 every few seconds to keep heartbeat alive."""
    while True:
        esp32.send("PING")
        time.sleep(PING_INTERVAL_S)


def _esp_listener_loop():
    """Forwards ESP32 messages into the RL env."""
    while True:
        msg = esp32.get_latest_message()
        if msg:
            env.update_esp_message(msg)
        time.sleep(0.05)


def _auto_loop():
    """
    Main autonomous RL loop.
    Runs at AUTO_STEP_HZ when in AUTO mode.
    Also handles idle timeout: switches MANUAL → AUTO after inactivity.
    """
    obs  = env.reset()
    step = 0

    while True:
        # ── idle timeout: MANUAL → AUTO ──────────────────────────────────
        with _mode_lock:
            mode      = _mode
            last_inp  = _last_manual_input

        if mode == "MANUAL":
            if time.time() - last_inp > IDLE_TIMEOUT_S:
                print("[Mode] Idle timeout — resuming AUTO")
                set_mode("AUTO")
                obs = env.reset()
            time.sleep(0.5)
            continue

        # ── AUTO: RL step ─────────────────────────────────────────────────
        action          = agent.select_action(obs)
        next_obs, reward, _, info = env.step(action)

        agent.store(obs, action, reward, next_obs)
        occ_map.move(action)

        # request STATUS from ESP32 every 5 steps to update distance
        if step % 5 == 0:
            esp32.send("STATUS")

        obs   = next_obs
        step += 1

        # epsilon decay every episode-equivalent (100 steps)
        if step % 100 == 0:
            agent.decay_epsilon()

        time.sleep(1.0 / AUTO_STEP_HZ)


# Start background threads
threading.Thread(target=_ping_loop,         daemon=True).start()
threading.Thread(target=_esp_listener_loop, daemon=True).start()
threading.Thread(target=_auto_loop,         daemon=True).start()

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


# ── routes: dashboard ─────────────────────────────────────────────────────────

@app.route("/")
def index():
    return """
    <!DOCTYPE html>
    <html>
    <head>
      <title>RoverPi</title>
      <style>
        body  { font-family: monospace; background:#111; color:#eee; padding:20px; }
        h1,h2 { color:#4af; }
        button {
          padding:10px 20px; margin:4px; font-size:14px;
          background:#222; color:#eee; border:1px solid #4af;
          border-radius:4px; cursor:pointer;
        }
        button:hover  { background:#4af; color:#000; }
        button.danger { border-color:#f44; }
        button.danger:hover { background:#f44; }
        button.active { background:#4af; color:#000; }
        #status { background:#1a1a1a; padding:12px; border-radius:4px;
                  border:1px solid #333; min-height:80px; }
        .row { display:flex; gap:20px; flex-wrap:wrap; }
        .panel { background:#1a1a1a; padding:16px; border-radius:6px;
                 border:1px solid #333; }
        #mode-badge {
          display:inline-block; padding:4px 12px; border-radius:12px;
          font-weight:bold; margin-left:12px;
        }
        .auto   { background:#2a4; color:#fff; }
        .manual { background:#a42; color:#fff; }
      </style>
    </head>
    <body>
      <h1>RoverPi Dashboard
        <span id="mode-badge" class="auto">AUTO</span>
      </h1>

      <div class="row">

        <div class="panel">
          <h2>Camera</h2>
          <img src="/video" width="480" style="border:1px solid #333"/>
        </div>

        <div class="panel">
          <h2>Occupancy Map</h2>
          <img id="map-img" src="/map" width="300"
               style="border:1px solid #333; image-rendering:pixelated"/>
        </div>

      </div>

      <div class="row" style="margin-top:16px">

        <div class="panel">
          <h2>Mode Control</h2>
          <button onclick="setMode('AUTO')"   id="btn-auto">🤖 Auto</button>
          <button onclick="setMode('MANUAL')" id="btn-manual">🕹 Manual</button>
        </div>

        <div class="panel">
          <h2>Manual Drive</h2>
          <div style="display:grid;grid-template-columns:repeat(3,80px);gap:4px">
            <div></div>
            <button onclick="cmd('forward')">▲</button>
            <div></div>
            <button onclick="cmd('left')">◄</button>
            <button onclick="cmd('stop')" class="danger">■</button>
            <button onclick="cmd('right')">►</button>
            <div></div>
            <button onclick="cmd('backward')">▼</button>
            <div></div>
          </div>
        </div>

        <div class="panel">
          <h2>Camera Pan/Tilt</h2>
          <button onclick="cam('left')">◄</button>
          <button onclick="cam('right')">►</button>
          <button onclick="cam('up')">▲</button>
          <button onclick="cam('down')">▼</button>
          <button onclick="cam('center')">⊙ Center</button>
        </div>

      </div>

      <div class="panel" style="margin-top:16px">
        <h2>Status</h2>
        <pre id="status">Loading...</pre>
      </div>

      <script>
        async function cmd(c) {
          await fetch('/cmd/' + c);
        }
        async function cam(d) {
          await fetch('/cam/' + d);
        }
        async function setMode(m) {
          await fetch('/mode/' + m.toLowerCase());
        }

        setInterval(async () => {
          const res  = await fetch('/status');
          const data = await res.json();
          document.getElementById('status').textContent =
            JSON.stringify(data, null, 2);
          const badge = document.getElementById('mode-badge');
          badge.textContent = data.mode;
          badge.className   = data.mode === 'AUTO' ? 'auto' : 'manual';
          // refresh map
          document.getElementById('map-img').src =
            '/map?t=' + Date.now();
        }, 500);
      </script>
    </body>
    </html>
    """


# ── routes: video + map ───────────────────────────────────────────────────────

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


# ── routes: status ────────────────────────────────────────────────────────────

@app.route("/status")
def status():
    return jsonify({
        "mode":          get_mode(),
        "epsilon":       round(agent.epsilon, 3),
        "agent_steps":   agent.steps,
        "buffer_size":   len(agent.buffer),
        "training":      agent.is_training,
        "exploration":   round(occ_map.get_free_ratio() * 100, 1),
        "esp32_msg":     esp32.get_latest_message(),
        "pan_angle":     pantilt.pan_angle,
        "tilt_angle":    pantilt.tilt_angle,
    })


# ── routes: mode ──────────────────────────────────────────────────────────────

@app.route("/mode/<m>")
def mode_switch(m):
    m = m.upper()
    if m not in ("AUTO", "MANUAL"):
        return jsonify({"error": "invalid mode"}), 400
    set_mode(m)
    return jsonify({"mode": get_mode()})


# ── routes: manual commands ───────────────────────────────────────────────────

@app.route("/cmd/<cmd>")
def send_cmd(cmd):
    allowed = {"forward", "backward", "left", "right", "stop"}
    if cmd not in allowed:
        return jsonify({"error": "invalid command"}), 400

    # switch to MANUAL and reset idle timer
    set_mode("MANUAL")
    touch_manual()

    esp32.send(cmd.upper())
    return jsonify({"sent": cmd.upper(), "esp32": esp32.get_latest_message()})


# ── routes: camera pan/tilt ───────────────────────────────────────────────────

@app.route("/cam/left")
def cam_left():
    pantilt.pan_left()
    return jsonify({"pan": pantilt.pan_angle})

@app.route("/cam/right")
def cam_right():
    pantilt.pan_right()
    return jsonify({"pan": pantilt.pan_angle})

@app.route("/cam/up")
def cam_up():
    pantilt.tilt_up()
    return jsonify({"tilt": pantilt.tilt_angle})

@app.route("/cam/down")
def cam_down():
    pantilt.tilt_down()
    return jsonify({"tilt": pantilt.tilt_angle})

@app.route("/cam/center")
def cam_center():
    pantilt.center()
    return jsonify({"pan": pantilt.pan_angle, "tilt": pantilt.tilt_angle})


# ── routes: auto control ──────────────────────────────────────────────────────

@app.route("/auto/start")
def auto_start():
    set_mode("AUTO")
    return jsonify({"mode": "AUTO"})

@app.route("/auto/stop")
def auto_stop():
    set_mode("MANUAL")
    esp32.send("STOP")
    return jsonify({"mode": "MANUAL"})


# ── entry point ───────────────────────────────────────────────────────────────

if __name__ == "__main__":
    app.run(host="0.0.0.0", port=5000, threaded=True)