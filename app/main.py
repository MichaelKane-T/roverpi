"""
===============================================================================
main.py
RoverPi Main Launcher + Autonomous Controller
Created: April 28, 2026

Purpose
-------
This file is the robot "brain launcher".

Run this file directly:

    python3 main.py

Responsibilities
----------------
1. Initialize hardware:
   - PiCamera2
   - Pan/Tilt servos
   - ESP32 UART connection
2. Initialize autonomy objects:
   - RoverEnv
   - RoverAgent
   - OccupancyMap
3. Maintain rover state:
   - AUTO / MANUAL mode
   - Parsed ESP32 STATUS cache
   - Scan readings
   - Startup health gate
4. Start background control threads:
   - Startup sequence
   - Heartbeat ping loop
   - ESP32 listener
   - Autonomous Predator Mode loop
   - Scan parser
5. Create and run the Flask dashboard app from app_flask.py

Design Rule
-----------
main.py owns the robot.
app_flask.py owns the web dashboard.

Do NOT run app_flask.py directly.
===============================================================================
"""

import time
import threading
from types import SimpleNamespace

from picamera2 import Picamera2

import pantilt
import serial_comm
from rover_env import RoverEnv
from rover_agent import RoverAgent
from occupancy_map import OccupancyMap
from app_flask import create_app


# =============================================================================
# Configuration
# =============================================================================

IDLE_TIMEOUT_S = 30.0
PING_INTERVAL_S = 0.4
AUTO_STEP_HZ = 4

FRAME_W = 640
FRAME_H = 480

STARTUP_ESP32_TIMEOUT = 10.0
STARTUP_SENSOR_CHECKS = 5
STARTUP_SCAN_WAIT = 2.0

# Obstacle escape behavior
ESCAPE_BACKWARD_S = 0.8
OBSTACLE_STRIKE_MAX = 3

# Predator Mode behavior
PREDATOR_ACTION_HOLD_STEPS = 3
PREDATOR_FORWARD_BIAS = True


# =============================================================================
# Hardware Initialization
# =============================================================================

print("[MAIN] Initializing camera...")
picam2 = Picamera2()
picam2.configure(
    picam2.create_preview_configuration(
        main={"size": (FRAME_W, FRAME_H), "format": "RGB888"}
    )
)
picam2.start()

print("[MAIN] Initializing pan/tilt...")
pantilt.init()

print("[MAIN] Connecting to ESP32...")
esp32 = serial_comm.ESP32Serial()
esp32.connect()
esp32.start_reader()
time.sleep(1.0)


# =============================================================================
# Autonomy Objects
# =============================================================================

occ_map = OccupancyMap()
env = RoverEnv(picam2, esp32)
agent = RoverAgent()


# =============================================================================
# Shared Rover State
# =============================================================================

_system_ready = False
_startup_log = []

_mode = "MANUAL"
_mode_lock = threading.Lock()
_last_manual_input = 0.0

_scan_lock = threading.Lock()
_scan_latest = {}

_esp_status_lock = threading.Lock()
_esp_status = {
    "dist": -2.0,   # -2.0 means no STATUS received yet
    "path": 0,      # 1 = clear, 0 = blocked
    "dir": 0,
    "fault": 0,
    "raw": "",
}

_escape_lock = threading.Lock()


# =============================================================================
# Mode Helpers
# =============================================================================

def get_mode():
    """Return current rover mode: AUTO or MANUAL."""
    with _mode_lock:
        return _mode


def set_mode(new_mode: str):
    """
    Switch rover mode.

    MANUAL:
        Stop rover and reset idle timer.

    AUTO:
        Reset environment before autonomy resumes.
    """
    global _mode, _last_manual_input

    new_mode = new_mode.upper()
    if new_mode not in ("AUTO", "MANUAL"):
        return

    with _mode_lock:
        if _mode == new_mode:
            return

        print(f"[Mode] {_mode} -> {new_mode}")
        _mode = new_mode

        if new_mode == "MANUAL":
            _last_manual_input = time.time()
            esp32.send("STOP")
        else:
            env.reset()


def touch_manual():
    """Update manual control idle timer."""
    global _last_manual_input
    with _mode_lock:
        _last_manual_input = time.time()


# =============================================================================
# Telemetry Stubs
# =============================================================================

def read_battery_stub() -> dict:
    """
    Placeholder battery endpoint.

    Replace this later with real INA219 / MAX17043 / ESP32 ADC data.
    """
    return {
        "voltage": None,
        "percent": None,
        "charging": None,
        "stub": True,
        "note": "Battery monitor hardware not yet installed",
    }


def read_gyro_stub() -> dict:
    """
    Placeholder gyro endpoint.

    Replace this later with real MPU6050 / ICM-42688 data.
    """
    return {
        "roll": None,
        "pitch": None,
        "yaw": None,
        "accel": {"x": None, "y": None, "z": None},
        "stub": True,
        "note": "IMU hardware not yet installed",
    }


# =============================================================================
# ESP32 STATUS Parsing
# =============================================================================

def _update_esp_status(msg: str):
    """
    Parse ESP32 STATUS line into a clean dictionary.

    Expected:
        STATUS dist=XX.X path=X dir=X fault=X

    Why this exists:
        esp32.get_latest_message() is often PONG because the heartbeat runs
        constantly. The dashboard should not use raw latest message to decide
        whether the path is blocked.
    """
    if not msg.startswith("STATUS"):
        return

    try:
        parts = msg.split()
        parsed = {}

        for p in parts[1:]:
            key, value = p.split("=")
            parsed[key] = value

        with _esp_status_lock:
            _esp_status["dist"] = float(parsed.get("dist", _esp_status["dist"]))
            _esp_status["path"] = int(parsed.get("path", _esp_status["path"]))
            _esp_status["dir"] = int(parsed.get("dir", _esp_status["dir"]))
            _esp_status["fault"] = int(parsed.get("fault", _esp_status["fault"]))
            _esp_status["raw"] = msg

    except Exception as e:
        print(f"[STATUS] Parse error: {e} | msg={msg}")


def get_esp_status() -> dict:
    """Return copy of latest parsed ESP32 status."""
    with _esp_status_lock:
        return dict(_esp_status)


def _sensor_healthy() -> bool:
    """
    Sensor is healthy if we have received a STATUS and it is not 0.0.

    Meaning:
        -2.0 = no status yet
        -1.0 = ultrasonic timeout / open space, acceptable
         0.0 = likely bad sensor read / wiring glitch
        >0.0 = normal distance
    """
    with _esp_status_lock:
        dist = _esp_status["dist"]

    return dist != -2.0 and dist != 0.0


# =============================================================================
# Startup Sequence
# =============================================================================

def _startup_sequence():
    """
    Startup gate for autonomous driving.

    Steps:
    1. Stop motors.
    2. Wait for ESP32 READY.
    3. Check sensor readings.
    4. Center camera.
    5. Run initial scan.
    6. Request STATUS to prime cache.
    7. Enable AUTO only if sensor looks usable.
    """
    global _system_ready

    def log(msg):
        print(f"[STARTUP] {msg}")
        _startup_log.append(msg)

    log("Startup sequence begun — motors held")
    esp32.send("STOP")
    time.sleep(0.2)

    log("Waiting for ESP32 READY...")
    deadline = time.time() + STARTUP_ESP32_TIMEOUT
    esp32_ready = False

    while time.time() < deadline:
        esp32.send("HELLO")
        time.sleep(0.2)

        msg = esp32.get_latest_message()
        if msg and "ESP32 READY" in msg:
            esp32_ready = True
            break

    log("ESP32 READY confirmed" if esp32_ready else "WARNING: ESP32 READY not received")

    log("Waiting for valid sensor readings...")
    valid_reads = 0
    attempts = 0

    while valid_reads < STARTUP_SENSOR_CHECKS and attempts < 10:
        esp32.send("STATUS")
        time.sleep(0.4)

        for msg in reversed(esp32.get_history()):
            if msg.startswith("STATUS"):
                _update_esp_status(msg)

                try:
                    dist = float(msg.split("dist=")[1].split()[0])

                    # Accept -1.0 timeout/open-space and positive readings.
                    # Reject 0.0 because it usually means a bad echo/wiring issue.
                    if dist != 0.0:
                        valid_reads += 1
                        label = "open/timeout" if dist < 0 else f"{dist:.1f}cm"
                        log(f"Valid read {valid_reads}/{STARTUP_SENSOR_CHECKS}: {label}")

                    break

                except Exception:
                    pass

        attempts += 1

    sensor_ok = valid_reads >= STARTUP_SENSOR_CHECKS

    if sensor_ok:
        log("Sensor validated")
    else:
        log("WARNING: Sensor not validated — only got 0.0/no STATUS readings")

    log("Centering camera...")
    pantilt.center()
    time.sleep(0.5)

    log("Running initial scan...")
    esp32.send("SCAN")
    time.sleep(STARTUP_SCAN_WAIT)

    log("Priming STATUS cache...")
    esp32.send("STATUS")
    time.sleep(0.4)

    _system_ready = sensor_ok

    if sensor_ok:
        set_mode("AUTO")
        log("Startup complete — AUTO mode active")
    else:
        set_mode("MANUAL")
        log("Startup complete with warnings — AUTO blocked until sensor is healthy")


# =============================================================================
# Predator Mode Action Selection
# =============================================================================

def _predator_choose_action(obs, last_action, hold_count):
    """
    Predator Mode wrapper around the RL agent.

    Goals:
    - Use the machine learning model.
    - Avoid twitchy random movement.
    - Bias toward forward movement when safe.
    - Prevent repeated left/right spinning.
    - Let RoverEnv.safe_action() handle blocked-path overrides.
    """
    esp_st = get_esp_status()
    path_clear = esp_st["path"] == 1

    # If blocked, ask the agent but let env.safe_action() override safely.
    if not path_clear:
        return agent.select_action(obs), 0

    # Hold forward briefly so movement looks intentional.
    if hold_count > 0 and last_action == 0:
        return last_action, hold_count - 1

    action = agent.select_action(obs)

    # Do not stop for no reason when path is clear.
    if action == 4 and PREDATOR_FORWARD_BIAS:
        action = 0

    # Do not reverse on an open path.
    if action == 3 and path_clear:
        action = 0

    # Avoid endless same-direction spinning.
    if last_action in (1, 2) and action == last_action:
        action = 0

    return action, PREDATOR_ACTION_HOLD_STEPS


# =============================================================================
# Scan Parsing
# =============================================================================

def _parse_scan_messages():
    """
    Parse ESP32 SCAN and STATUS messages from UART history.

    SCAN messages update:
    - Dashboard scan dictionary
    - RoverEnv side-distance readings
    - Occupancy map sweep

    STATUS messages update:
    - Clean ESP32 status cache
    """
    seen = set()

    while True:
        for msg in esp32.get_history():
            if msg in seen:
                continue

            seen.add(msg)

            if msg.startswith("SCAN"):
                try:
                    parts = msg.split()
                    angle = int(parts[1].split("=")[1])
                    dist = float(parts[2].split("=")[1])

                    with _scan_lock:
                        _scan_latest[angle] = dist

                    env.update_scan(angle, dist)

                    # Occupancy map expects angle relative to rover heading.
                    occ_map.update_sweep({angle - 90: dist})

                except Exception as e:
                    print(f"[SCAN] Parse error: {e} | msg={msg}")

            elif msg.startswith("STATUS"):
                _update_esp_status(msg)

        # Prevent unbounded memory growth.
        if len(seen) > 200:
            seen.clear()

        time.sleep(0.05)


# =============================================================================
# Obstacle Escape
# =============================================================================

def _escape_from_obstacle():
    """
    Hard escape routine for AUTO mode.

    Used when path remains blocked for multiple strikes:
    1. BACKWARD briefly
    2. STOP
    3. SCAN
    4. STATUS

    This prevents the rover from sitting forever against an obstacle.
    """
    if not _escape_lock.acquire(blocking=False):
        return

    try:
        print("[AUTO] Obstacle escape — reversing")
        esp32.send("BACKWARD")
        time.sleep(ESCAPE_BACKWARD_S)

        esp32.send("STOP")
        time.sleep(0.1)

        esp32.send("SCAN")
        time.sleep(0.8)

        esp32.send("STATUS")
        time.sleep(0.2)

    finally:
        _escape_lock.release()


# =============================================================================
# Background Threads
# =============================================================================

def _ping_loop():
    """
    Keep ESP32 watchdog alive.

    PING runs independently from AUTO mode so the ESP32 does not safe-stop
    during manual driving, startup, scanning, or obstacle escape.
    """
    while True:
        esp32.send("PING")
        time.sleep(PING_INTERVAL_S)


def _esp_listener_loop():
    """
    Feed meaningful ESP32 messages into RoverEnv.

    PONG is ignored because it is not useful for autonomy.
    """
    while True:
        history = esp32.get_history()

        for msg in reversed(history):
            if any(k in msg for k in ("STATUS", "OBSTACLE", "ERR", "SCAN")):
                env.update_esp_message(msg)
                break

        time.sleep(0.05)


def _auto_loop():
    """
    Main autonomous driving loop.

    Flow:
    1. Wait for startup gate.
    2. If in MANUAL, wait until idle timeout.
    3. Check sensor health.
    4. Check obstacle status.
    5. Use Predator Mode + RL agent to choose action.
    6. Execute through RoverEnv.step(), which still applies safe_action().
    7. Store experience for training.
    """
    while not _system_ready:
        time.sleep(0.5)

    obs = env.reset()
    step = 0
    obstacle_strikes = 0

    last_action = 4  # STOP
    action_hold = 0

    while True:
        with _mode_lock:
            mode = _mode
            last_inp = _last_manual_input

        # ---------------------------------------------------------------------
        # Manual idle timeout
        # ---------------------------------------------------------------------
        if mode == "MANUAL":
            if time.time() - last_inp > IDLE_TIMEOUT_S:
                print("[Mode] Idle timeout — resuming AUTO")
                set_mode("AUTO")

                obs = env.reset()
                obstacle_strikes = 0
                step = 0
                last_action = 4
                action_hold = 0

            time.sleep(0.5)
            continue

        # ---------------------------------------------------------------------
        # Sensor health gate
        # ---------------------------------------------------------------------
        if not _sensor_healthy():
            print("[AUTO] Sensor unhealthy — holding and requesting STATUS")
            esp32.send("STATUS")
            esp32.send("STOP")
            time.sleep(1.0)
            continue

        # ---------------------------------------------------------------------
        # Obstacle check before action
        # ---------------------------------------------------------------------
        esp_st = get_esp_status()
        path_clear = esp_st["path"] == 1
        dist_cm = esp_st["dist"]

        if not path_clear:
            obstacle_strikes += 1
            print(
                f"[AUTO] Path blocked ({dist_cm:.1f}cm) "
                f"strike {obstacle_strikes}/{OBSTACLE_STRIKE_MAX}"
            )

            if obstacle_strikes >= OBSTACLE_STRIKE_MAX:
                print("[AUTO] Max obstacle strikes — executing escape")

                _escape_from_obstacle()

                obs = env.reset()
                obstacle_strikes = 0
                step = 0
                last_action = 4
                action_hold = 0

            else:
                time.sleep(1.0 / AUTO_STEP_HZ)

            continue

        obstacle_strikes = 0

        # ---------------------------------------------------------------------
        # Predator Mode + RL step
        # ---------------------------------------------------------------------
        action, action_hold = _predator_choose_action(obs, last_action, action_hold)

        next_obs, reward, _, info = env.step(action)

        agent.store(obs, action, reward, next_obs)
        occ_map.move(info["safe_action"])

        last_action = info["safe_action"]

        print(
            f"[PREDATOR] agent={info['agent_action']} "
            f"safe={info['safe_action']} "
            f"override={info['overridden']} "
            f"reward={reward:.2f} "
            f"dist={info['distance_cm']:.1f}"
        )

        if step % 5 == 0:
            esp32.send("STATUS")
            time.sleep(0.05)

        obs = next_obs
        step += 1

        if step % 100 == 0:
            agent.decay_epsilon()

        time.sleep(1.0 / AUTO_STEP_HZ)


# =============================================================================
# Flask Context
# =============================================================================

def build_flask_context():
    """
    Create a small object passed into app_flask.py.

    This avoids a circular import:
        app_flask.py does NOT import main.py.
        main.py imports create_app() and passes this context.
    """
    return SimpleNamespace(
        picam2=picam2,
        esp32=esp32,
        occ_map=occ_map,
        agent=agent,

        get_mode=get_mode,
        set_mode=set_mode,
        touch_manual=touch_manual,
        get_esp_status=get_esp_status,

        read_battery_stub=read_battery_stub,
        read_gyro_stub=read_gyro_stub,

        scan_lock=_scan_lock,
        scan_latest=_scan_latest,

        startup_log=_startup_log,
        system_ready=lambda: _system_ready,
    )


def start_background_threads():
    """Start all robot background threads."""
    threading.Thread(target=_startup_sequence, daemon=True).start()
    threading.Thread(target=_ping_loop, daemon=True).start()
    threading.Thread(target=_esp_listener_loop, daemon=True).start()
    threading.Thread(target=_auto_loop, daemon=True).start()
    threading.Thread(target=_parse_scan_messages, daemon=True).start()


# =============================================================================
# Entry Point
# =============================================================================

if __name__ == "__main__":
    print("[MAIN] Starting RoverPi background threads...")
    start_background_threads()

    print("[MAIN] Starting Flask dashboard...")
    app = create_app(build_flask_context())
    app.run(host="0.0.0.0", port=5000, threaded=True)
