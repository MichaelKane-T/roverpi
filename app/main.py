from flask import Flask, Response
import cv2
import pantilt
from picamera2 import Picamera2

app = Flask(__name__)

picam2 = Picamera2()
config = picam2.create_preview_configuration(
    main={"size": (640, 480), "format": "RGB888"}
)
picam2.configure(config)
picam2.start()
pantilt.init()

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

@app.route("/")
def index():
    return """
    <html>
      <head><title>Rover Camera</title></head>
      <body>
        <h1>Rover Camera</h1>
        <img src="/video" width="640" />
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
    return {"status": "ok", "camera": "picamera2"}

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

if __name__ == "__main__":
    app.run(host="0.0.0.0", port=5000)
