"""
===============================================================================
app_flask.py
RoverPi Flask Dashboard + HTTP API
Created: April 28, 2026
Updated: April 29, 2026

Purpose
-------
This file owns ONLY the web/dashboard layer.

This version serves a built React/Vue/Vite frontend from:

    app/frontend/dist

Your workflow is:

    On development machine:
        cd app/frontend
        npm run build
        git add dist
        git commit -m "Update rover frontend"
        git push

    On Rover Pi:
        cd ~/roverpi
        git pull
        python3 main.py

Responsibilities
----------------
1. Serve the built Vite frontend:
   - /
   - /assets/*
   - fallback routes for React/Vue single-page app behavior

2. Stream camera video:
   - /video

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
   - /health

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

import os
import time
import cv2

from flask import Flask, Response, jsonify, send_from_directory

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

    # -------------------------------------------------------------------------
    # Frontend build path
    # -------------------------------------------------------------------------
    # Expected project layout:
    #
    # roverpi/
    #   └── app/
    #     ├── main.py
    #     ├── app_flask.py
    #     └── frontend/
    #         └── dist/
    #             ├── index.html
    #             └── assets/
    #
    # Flask serves index.html and the compiled static assets from here.
    # -------------------------------------------------------------------------
    BASE_DIR = os.path.dirname(os.path.abspath(__file__))
    DIST_DIR = os.path.join(BASE_DIR, "frontend", "dist")
    ASSETS_DIR = os.path.join(DIST_DIR, "assets")

    app = Flask(
        __name__,
        static_folder=ASSETS_DIR,
        static_url_path="/assets",
    )

    # =========================================================================
    # Video Stream Helper
    # =========================================================================

    def _generate_video():
        """
        MJPEG camera stream generator.

        The frontend simply uses:
            <img src="/video" />
        """
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
    # Built Frontend Routes
    # =========================================================================

    @app.route("/")
    def index():
        """
        Serve built Vite frontend index.html.

        Before running this on the rover, build frontend with:

            cd app/frontend
            npm run build
        """
        index_path = os.path.join(DIST_DIR, "index.html")

        if not os.path.exists(index_path):
            return jsonify({
                "error": "frontend build not found",
                "expected": index_path,
                "fix": "Run: cd app/frontend && npm run build, then git add app/frontend/dist and git push/pull."
            }), 500

        return send_from_directory(DIST_DIR, "index.html")

    @app.route("/<path:path>")
    def frontend_fallback(path):
        """
        Serve frontend files or fall back to index.html.

        Why this exists:
        - Vite puts compiled JS/CSS/images in dist/assets.
        - React/Vue single-page apps may also need refresh-safe routes.
        - If a requested file exists in dist, serve it.
        - Otherwise return index.html and let the frontend router handle it.

        Important:
        API routes like /status, /cmd, /video, etc. are declared separately.
        Flask chooses the more specific API routes before this fallback.
        """
        requested = os.path.join(DIST_DIR, path)

        if os.path.exists(requested) and os.path.isfile(requested):
            return send_from_directory(DIST_DIR, path)

        index_path = os.path.join(DIST_DIR, "index.html")

        if not os.path.exists(index_path):
            return jsonify({
                "error": "frontend build not found",
                "expected": index_path,
                "fix": "Run: cd app/frontend && npm run build, then git add app/frontend/dist and git push/pull."
            }), 500

        return send_from_directory(DIST_DIR, "index.html")

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
