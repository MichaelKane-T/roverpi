from flask import Flask, Response, jsonify
import cv2
import time

import pantilt
import serial_comm
from picamera2 import Picamera2

app = Flask(__name__)

# Camera setup
picam2 = Picamera2()
config = picam2.create_preview_configuration(
    main={"size": (640, 480), "format": "RGB888"}
)
picam2.configure(config)
picam2.start()

# Pan/tilt setup
pantilt.init()

# ESP32 serial setup
esp32 = serial_comm.ESP32Serial()
esp32.connect()
esp32.start_reader()

time.sleep(1)
esp32.send("FORWARD")
print("[ROVER] Auto-start command sent: FORWARD")

def generate():
    while True:
        frame = picam2.capture_array()

        ok, buffer = cv2.imencode(".jpg", frame)
        if not ok:
            continue

        yield (
            b"--frame\r\n"
            b"Content-Type: image/jpeg\r\n\r\n" + buffer.tobytes() + b"\r\n"
        )

        time.sleep(0.05)


@app.route("/")
def index():
    return """
    <html>
      <head><title>RoverPi Dashboard</title></head>
      <body>
        <h1>RoverPi Dashboard</h1>

        <h2>Camera</h2>
        <img src="/video" width="640" />

        <h2>Camera Pan/Tilt</h2>
        <button onclick="fetch('/cam/left')">Left</button>
        <button onclick="fetch('/cam/right')">Right</button>
        <button onclick="fetch('/cam/up')">Up</button>
        <button onclick="fetch('/cam/down')">Down</button>
        <button onclick="fetch('/cam/center')">Center</button>

        <h2>Rover Control</h2>
        <button onclick="fetch('/cmd/forward')">Forward</button>
        <button onclick="fetch('/cmd/backward')">Backward</button>
        <button onclick="fetch('/cmd/left')">Turn Left</button>
        <button onclick="fetch('/cmd/right')">Turn Right</button>
        <button onclick="fetch('/cmd/stop')">Stop</button>

        <h2>Status</h2>
        <pre id="status">Loading...</pre>

        <script>
          setInterval(async () => {
            const res = await fetch('/status');
            const data = await res.json();
            document.getElementById('status').textContent =
              JSON.stringify(data, null, 2);
          }, 500);
        </script>
      </body>
    </html>
    """


@app.route("/video")
def video():
    return Response(
        generate(),
        mimetype="multipart/x-mixed-replace; boundary=frame"
    )


@app.route("/status")
def status():
    return jsonify({
        "status": "ok",
        "camera": "picamera2",
        "esp32": esp32.get_latest_message()
    })


@app.route("/cam/left")
def cam_left():
    pantilt.pan_left()
    return "OK"


@app.route("/cam/right")
def cam_right():
    pantilt.pan_right()
    return "OK"


@app.route("/cam/up")
def cam_up():
    pantilt.tilt_up()
    return "OK"


@app.route("/cam/down")
def cam_down():
    pantilt.tilt_down()
    return "OK"


@app.route("/cam/center")
def cam_center():
    pantilt.center()
    return "OK"


@app.route("/cmd/<cmd>")
def send_cmd(cmd):
    allowed = {"forward", "backward", "left", "right", "stop"}

    if cmd not in allowed:
        return jsonify({"error": "invalid command"}), 400

    esp32.send(cmd.upper())

    return jsonify({
        "sent": cmd.upper(),
        "latest": esp32.get_latest_message()
    })

@app.route("/auto/start")
def auto_start():
    esp32.send("FORWARD")
    return jsonify({"mode": "auto", "cmd": "FORWARD"})


@app.route("/auto/stop")
def auto_stop():
    esp32.send("STOP")
    return jsonify({"mode": "manual/stop", "cmd": "STOP"})

if __name__ == "__main__":
    app.run(host="0.0.0.0", port=5000, threaded=True)