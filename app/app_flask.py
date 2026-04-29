"""
===============================================================================
app_flask.py
RoverPi Flask Dashboard + HTTP API
Created: April 28, 2026

Purpose
-------
This file owns ONLY the web/dashboard layer.

Responsibilities
----------------
1. Serve the local dashboard page.
2. Stream camera video.
3. Expose rover control endpoints:
   - /cmd/<cmd>
   - /auto/start
   - /auto/stop
   - /mode/<m>
4. Expose telemetry endpoints:
   - /status
   - /esp32_status
   - /battery
   - /gyro
   - /map
5. Call functions/objects from the context passed by main.py.

Design Rule
-----------
Do NOT run this file directly.

Run:

    python3 main.py

main.py creates the robot objects, starts robot threads, then calls:

    create_app(context)

This avoids circular imports and keeps the robot brain separate from Flask.
===============================================================================
"""

import time
import cv2

from flask import Flask, Response, jsonify

import pantilt


# =============================================================================
# Flask Factory
# =============================================================================

def create_app(ctx):
    """
    Build and return the Flask app.

    ctx is a SimpleNamespace created by main.py containing:
        - picam2
        - esp32
        - occ_map
        - agent
        - get_mode()
        - set_mode()
        - touch_manual()
        - get_esp_status()
        - telemetry helpers
        - scan lock/data
        - startup log/system-ready accessor
    """
    app = Flask(__name__)

    # =========================================================================
    # Video Stream Helper
    # =========================================================================

    def _generate_video():
        """MJPEG camera stream generator."""
        while True:
            frame = ctx.picam2.capture_array()
            ok, buf = cv2.imencode(".jpg", frame)

            if ok:
                yield (
                    b"--frame\r\n"
                    b"Content-Type: image/jpeg\r\n\r\n"
                    + buf.tobytes()
                    + b"\r\n"
                )

            time.sleep(0.05)

    # =========================================================================
    # Dashboard Page
    # =========================================================================

    @app.route("/")
    def index():
        """
        Main dashboard HTML.

        This stays in app_flask.py because it is part of the web UI,
        not part of the robot autonomy logic.
        """
        return """<!DOCTYPE html>
<html>
<head>
  <title>RoverPi Local</title>
  <style>
    * { box-sizing: border-box; margin: 0; padding: 0; }
    body {
      background: #05070b;
      font-family: monospace;
      color: #cbd5e1;
      padding: 16px;
    }
    h1 {
      color: #67e8f9;
      font-size: 18px;
      letter-spacing: .2em;
      margin-bottom: 16px;
    }
    .grid {
      display: grid;
      grid-template-columns: 1fr 1fr;
      gap: 12px;
      max-width: 900px;
    }
    .panel {
      border: 1px solid rgba(34,211,238,.15);
      background: rgba(8,16,24,.9);
      border-radius: 12px;
      padding: 12px;
    }
    .panel h2 {
      font-size: 10px;
      text-transform: uppercase;
      letter-spacing: .25em;
      color: rgba(103,232,249,.7);
      margin-bottom: 10px;
    }
    img {
      width: 100%;
      border-radius: 8px;
      background: #02050a;
    }
    .btn-row {
      display: flex;
      flex-wrap: wrap;
      gap: 6px;
      margin-top: 6px;
    }
    button {
      border: 1px solid rgba(56,189,248,.3);
      background: rgba(56,189,248,.06);
      border-radius: 8px;
      padding: 8px 14px;
      font-size: 11px;
      font-weight: 800;
      letter-spacing: .14em;
      color: #7dd3fc;
      cursor: pointer;
      font-family: monospace;
    }
    button:hover {
      background: rgba(56,189,248,.18);
    }
    button.danger {
      border-color: rgba(248,113,113,.35);
      color: #fca5a5;
      background: rgba(248,113,113,.08);
    }
    button.mode-active {
      background: rgba(34,211,238,.2);
      color: #a5f3fc;
    }
    .stat {
      display: flex;
      justify-content: space-between;
      padding: 5px 8px;
      background: rgba(255,255,255,.03);
      border-radius: 6px;
      margin-top: 4px;
      font-size: 11px;
    }
    .stat span:last-child {
      color: #67e8f9;
      font-weight: 700;
    }
    .path-clear {
      color: #34d399 !important;
    }
    .path-blocked {
      color: #f87171 !important;
    }
    #log {
      height: 160px;
      overflow-y: auto;
      background: rgba(0,0,0,.3);
      border-radius: 8px;
      padding: 8px;
      font-size: 10px;
      margin-top: 6px;
    }
    .log-line {
      border-bottom: 1px solid rgba(255,255,255,.04);
      padding: 2px 0;
      color: #94a3b8;
    }
    .log-ok { color: #34d399; }
    .log-err { color: #f87171; }
    input[type=range] {
      width: 100%;
      accent-color: #38bdf8;
      margin: 4px 0;
    }
    .slider-row {
      font-size: 10px;
      color: #475569;
      display: flex;
      justify-content: space-between;
      margin-bottom: 2px;
    }
    #online {
      display: inline-block;
      width: 8px;
      height: 8px;
      border-radius: 50%;
      background: #f87171;
      margin-right: 6px;
    }
    #online.up {
      background: #34d399;
      box-shadow: 0 0 6px #34d399;
    }
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
        <button id="btn-auto" onclick="setMode('AUTO')">AUTO</button>
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

      <div class="slider-row">
        <span>Pan</span>
        <span id="pan-v">90&deg;</span>
      </div>
      <input type="range" min="0" max="180" value="90" id="pan"
             oninput="document.getElementById('pan-v').textContent=this.value+'&deg;'"
             onchange="fetch('/cam/pan/'+this.value)">

      <div class="slider-row" style="margin-top:8px">
        <span>Tilt</span>
        <span id="tilt-v">90&deg;</span>
      </div>
      <input type="range" min="15" max="145" value="90" id="tilt"
             style="accent-color:#fbbf24"
             oninput="document.getElementById('tilt-v').textContent=this.value+'&deg;'"
             onchange="fetch('/cam/tilt/'+this.value)">

      <div class="btn-row" style="margin-top:8px">
        <button onclick="centerCam()">&#8857; Center</button>
        <button onclick="cmd('scan')">Scan</button>
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
      <div class="stat">
        <span>ESP32 raw</span>
        <span id="t-esp" style="font-size:9px;max-width:160px;overflow:hidden;text-overflow:ellipsis">—</span>
      </div>
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
      d.innerHTML =
        '<span class="'+(ok?'log-ok':'log-err')+'">'+
        new Date().toLocaleTimeString()+
        '</span> '+msg;

      log.prepend(d);

      if (log.children.length > 60) {
        log.lastChild.remove();
      }
    }

    async function cmd(c) {
      const r = await fetch('/cmd/'+c);
      const d = await r.json();
      addLog('CMD '+c.toUpperCase()+' → '+(d.esp32 || 'sent'), r.ok);
    }

    async function setMode(m) {
      const url = m === 'AUTO' ? '/auto/start' : '/auto/stop';
      await fetch(url);
      addLog('Mode → '+m);
    }

    async function centerCam() {
      await fetch('/cam/center');

      document.getElementById('pan').value = 90;
      document.getElementById('tilt').value = 90;
      document.getElementById('pan-v').textContent = '90°';
      document.getElementById('tilt-v').textContent = '90°';

      addLog('Camera centered');
    }

    async function poll() {
      try {
        const [sr, se] = await Promise.all([
          fetch('/status').then(r => r.json()),
          fetch('/esp32_status').then(r => r.json()),
        ]);

        document.getElementById('online').className = 'up';

        const dist = se.dist >= 0 ? se.dist.toFixed(1)+'cm' : '—';
        const pathOk = se.path === 1;

        const pathEl = document.getElementById('t-path');
        pathEl.textContent = pathOk ? 'CLEAR' : 'BLOCKED';
        pathEl.className = pathOk ? 'path-clear' : 'path-blocked';

        document.getElementById('t-dist').textContent = dist;
        document.getElementById('t-mode').textContent = sr.mode;
        document.getElementById('t-ready').textContent = sr.system_ready ? 'YES' : 'NO';
        document.getElementById('t-steps').textContent = sr.agent_steps;
        document.getElementById('t-eps').textContent = sr.epsilon;
        document.getElementById('t-esp').textContent = se.raw || sr.esp32_msg || '—';

        document.getElementById('btn-auto').className =
          sr.mode === 'AUTO' ? 'mode-active' : '';
        document.getElementById('btn-manual').className =
          sr.mode === 'MANUAL' ? 'mode-active' : '';

        document.getElementById('map-img').src = '/map?t=' + Date.now();

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

    # =========================================================================
    # Video + Map
    # =========================================================================

    @app.route("/video")
    def video():
        """MJPEG video endpoint."""
        return Response(
            _generate_video(),
            mimetype="multipart/x-mixed-replace; boundary=frame",
        )

    @app.route("/map")
    def map_image():
        """Occupancy map image endpoint."""
        jpg = ctx.occ_map.render(cell_px=10)
        return Response(jpg, mimetype="image/jpeg")

    # =========================================================================
    # Telemetry
    # =========================================================================

    @app.route("/status")
    def status():
        """General rover status endpoint."""
        with ctx.scan_lock:
            scan = dict(ctx.scan_latest)

        return jsonify({
            "mode": ctx.get_mode(),
            "system_ready": ctx.system_ready(),
            "startup_log": ctx.startup_log[-5:],
            "epsilon": round(ctx.agent.epsilon, 3),
            "agent_steps": ctx.agent.steps,
            "buffer_size": len(ctx.agent.buffer),
            "training": ctx.agent.is_training,
            "exploration": round(ctx.occ_map.get_free_ratio() * 100, 1),
            "esp32_msg": ctx.esp32.get_latest_message(),
            "pan_angle": pantilt.pan_angle,
            "tilt_angle": pantilt.tilt_angle,
            "scan": scan,
        })

    @app.route("/esp32_status")
    def esp32_status():
        """
        Clean parsed ESP32 status endpoint.

        This is better than using raw latest message because raw latest message
        is often PONG.
        """
        return jsonify(ctx.get_esp_status())

    @app.route("/battery")
    def battery():
        """Battery telemetry endpoint."""
        return jsonify(ctx.read_battery_stub())

    @app.route("/gyro")
    def gyro():
        """Gyro telemetry endpoint."""
        return jsonify(ctx.read_gyro_stub())

    @app.route("/health")
    def health():
        """Simple server health endpoint."""
        return jsonify({"status": "ok", "host": "roverpi"})

    # =========================================================================
    # Mode Control
    # =========================================================================

    @app.route("/mode/<m>")
    def mode_switch(m):
        """Switch rover mode using /mode/AUTO or /mode/MANUAL."""
        m = m.upper()

        if m not in ("AUTO", "MANUAL"):
            return jsonify({"error": "invalid mode"}), 400

        ctx.set_mode(m)
        return jsonify({"mode": ctx.get_mode()})

    @app.route("/auto/start")
    def auto_start():
        """Switch rover into AUTO mode."""
        ctx.set_mode("AUTO")
        return jsonify({"mode": "AUTO"})

    @app.route("/auto/stop")
    def auto_stop():
        """Switch rover into MANUAL mode and stop motors."""
        ctx.set_mode("MANUAL")
        ctx.esp32.send("STOP")
        return jsonify({"mode": "MANUAL"})

    # =========================================================================
    # Manual Commands
    # =========================================================================

    @app.route("/cmd/<cmd>")
    def send_cmd(cmd):
        """
        Send manual command to ESP32.

        Important behavior:
        - If currently AUTO, switch to MANUAL first.
        - If already MANUAL, do NOT send an extra STOP before the command.
          That was the cause of sluggish manual driving.
        """
        allowed = {
            "forward",
            "backward",
            "left",
            "right",
            "stop",
            "scan",
            "fault_clear",
        }

        cmd = cmd.lower()

        if cmd not in allowed:
            return jsonify({"error": "invalid command"}), 400

        if ctx.get_mode() != "MANUAL":
            ctx.set_mode("MANUAL")
        else:
            ctx.touch_manual()

        ctx.esp32.send(cmd.upper())
        time.sleep(0.05)

        return jsonify({
            "sent": cmd.upper(),
            "esp32": ctx.esp32.get_latest_message(),
        })

    # =========================================================================
    # Camera Pan/Tilt
    # =========================================================================

    @app.route("/cam/<direction>")
    def cam_control(direction):
        """Relative camera movement endpoint."""
        actions = {
            "left": pantilt.pan_left,
            "right": pantilt.pan_right,
            "up": pantilt.tilt_up,
            "down": pantilt.tilt_down,
            "center": pantilt.center,
        }

        if direction not in actions:
            return jsonify({"error": "invalid direction"}), 400

        actions[direction]()

        return jsonify({
            "pan": pantilt.pan_angle,
            "tilt": pantilt.tilt_angle,
        })

    @app.route("/cam/pan/<int:angle>")
    def cam_pan_absolute(angle):
        """Set absolute pan angle."""
        angle = max(0, min(180, angle))

        pantilt.set_angle(pantilt.PAN, angle)
        pantilt.pan_angle = angle

        return jsonify({"pan": pantilt.pan_angle})

    @app.route("/cam/tilt/<int:angle>")
    def cam_tilt_absolute(angle):
        """Set absolute tilt angle."""
        angle = max(15, min(145, angle))

        pantilt.set_angle(pantilt.TILT, angle)
        pantilt.tilt_angle = angle

        return jsonify({"tilt": pantilt.tilt_angle})

    return app


# =============================================================================
# Safety Guard
# =============================================================================

if __name__ == "__main__":
    print("Do not run app_flask.py directly.")
    print("Run this instead:")
    print("")
    print("    python3 main.py")
