"""
rover_env.py
RoverPi RL Environment — Pi Zero 2W

Observation space (13 floats):
  [0]    forward distance normalised (0-1, clip 200cm)
  [1]    obstacle flag from ESP32 (0 or 1)
  [2:5]  HSV mean — left third of frame
  [5:8]  HSV mean — centre third
  [8:11] HSV mean — right third
  [11]   pan angle normalised (0-1)
  [12]   tilt angle normalised (0-1)

Action space (discrete, 5):
  0 = FORWARD
  1 = LEFT
  2 = RIGHT
  3 = BACKWARD   ← only used as last resort (see escape hierarchy)
  4 = STOP

Escape hierarchy (enforced in safe_action()):
  1. FORWARD if path clear
  2. LEFT or RIGHT if that side has more clearance
  3. BACKWARD only if forward AND both sides are blocked
     — reverses just enough to create clearance, then rescans
     — never reverses indefinitely
"""

import time
import threading
import numpy as np
import cv2

import pantilt
import serial_comm

# ── constants ────────────────────────────────────────────────────────────────
OBSTACLE_CM        = 20.0    # distance considered blocked
SIDE_OBSTACLE_CM   = 15.0    # side clearance threshold (tighter — turning space)
MAX_DIST_CM        = 200.0
FRAME_W, FRAME_H   = 160, 120
STEP_DELAY_S       = 0.15
BACKWARD_STEPS_MAX = 3       # max consecutive backward steps before stop+rescan
SCAN_WAIT_S        = 1.2     # wait after SCAN command for readings to arrive

ACTION_CMDS = ["FORWARD", "LEFT", "RIGHT", "BACKWARD", "STOP"]
OBS_DIM     = 13

# ── reward weights ────────────────────────────────────────────────────────────
R_FORWARD       =  2.0
R_TURN          = -0.05
R_BACKWARD      = -1.0    # stronger penalty — backward is a last resort
R_STOP          = -0.25
R_OBSTACLE_NEAR = -2.0
R_OBSTACLE_HIT  = -5.0
R_REVISIT       = -0.5
R_BACKWARD_ESCAPE = 0.4  # small positive when backward genuinely opens space


class RoverEnv:
    def __init__(self, picam2, esp32: serial_comm.ESP32Serial):
        self.picam2 = picam2
        self.esp32  = esp32

        self._lock        = threading.Lock()
        self._distance_cm = MAX_DIST_CM   # forward distance
        self._esp_msg     = ""

        # Scan readings from last sweep {angle_deg: dist_cm}
        # Updated externally by _parse_scan_messages() in app.py
        self._scan_readings = {}
        self._scan_lock     = threading.Lock()

        # Visit grid for revisit penalty
        self._visit_grid = np.zeros((20, 20), dtype=np.float32)
        self._grid_x     = 10
        self._grid_y     = 10

        self._last_action      = 4   # STOP
        self._backward_count   = 0   # consecutive backward steps taken
        self._was_all_blocked  = False  # True when we entered backward escape

        self.steps = 0

    # ── public interface ──────────────────────────────────────────────────────

    def reset(self):
        """Stop, centre camera, reset counters, return initial obs."""
        self.esp32.send("STOP")
        pantilt.center()
        self._last_action     = 4
        self._backward_count  = 0
        self._was_all_blocked = False
        self.steps            = 0
        time.sleep(0.3)
        return self._get_obs()

    def update_scan(self, angle_deg: int, dist_cm: float):
        """Called by main.py scan parser whenever a SCAN line arrives."""
        with self._scan_lock:
            self._scan_readings[angle_deg] = dist_cm

    def safe_action(self, agent_action: int) -> int:
        """
        Enforces the escape hierarchy before passing action to hardware.

        Priority:
          1. If forward is clear → allow agent's action (normal RL)
          2. If forward blocked, check sides from scan readings:
             - Pick the clearer side (LEFT or RIGHT)
          3. If all three directions blocked:
             - Reverse up to BACKWARD_STEPS_MAX steps
             - After max steps, STOP and request a rescan
        Returns the safe action integer.
        """
        fwd_dist  = self._get_forward_dist()
        fwd_clear = fwd_dist > OBSTACLE_CM

        if fwd_clear:
            # Path open — let the agent decide, just don't allow backward
            # unless agent genuinely wants to explore backwards
            self._backward_count  = 0
            self._was_all_blocked = False
            if agent_action == 3:
                # Agent chose backward on open path — discourage, use STOP instead
                return 4   # STOP
            return agent_action

        # Forward blocked — check sides
        left_dist, right_dist = self._get_side_dists()
        left_clear  = left_dist  > SIDE_OBSTACLE_CM
        right_clear = right_dist > SIDE_OBSTACLE_CM

        if left_clear or right_clear:
            self._backward_count  = 0
            self._was_all_blocked = False
            if left_clear and right_clear:
                # Both clear — pick the more open side
                return 1 if left_dist >= right_dist else 2
            return 1 if left_clear else 2

        # All three directions blocked — backward escape
        self._was_all_blocked = True
        if self._backward_count < BACKWARD_STEPS_MAX:
            self._backward_count += 1
            print(f"[Escape] All blocked — reversing ({self._backward_count}/{BACKWARD_STEPS_MAX})")
            return 3   # BACKWARD

        # Hit backward limit — stop and request rescan
        print("[Escape] Backward limit reached — stopping and rescanning")
        self._backward_count = 0
        self.esp32.send("SCAN")
        time.sleep(SCAN_WAIT_S)
        return 4   # STOP

    def step(self, action: int):
        """
        Apply safe_action() filter, execute, collect obs and reward.
        Returns (obs, reward, done, info)
        """
        assert 0 <= action < len(ACTION_CMDS), f"Invalid action {action}"

        safe = self.safe_action(action)
        cmd  = ACTION_CMDS[safe]

        self.esp32.send(cmd)
        self._last_action = safe
        self.steps       += 1

        time.sleep(STEP_DELAY_S)

        obs    = self._get_obs()
        reward = self._compute_reward(safe, obs)
        done   = False
        info   = {
            "agent_action":  action,
            "safe_action":   safe,
            "overridden":    action != safe,
            "distance_cm":   self._distance_cm,
            "backward_count":self._backward_count,
            "esp_msg":       self._esp_msg,
            "step":          self.steps,
        }
        return obs, reward, done, info

    # ── observation ───────────────────────────────────────────────────────────

    def _get_obs(self) -> np.ndarray:
        obs = np.zeros(OBS_DIM, dtype=np.float32)

        dist = self._get_forward_dist()
        with self._lock:
            self._distance_cm = dist
        obs[0] = np.clip(dist / MAX_DIST_CM, 0.0, 1.0)

        with self._lock:
            msg = self._esp_msg
        obs[1] = 1.0 if "OBSTACLE" in msg else 0.0

        frame = self._capture_frame()
        if frame is not None:
            hsv = cv2.cvtColor(frame, cv2.COLOR_RGB2HSV)
            w   = hsv.shape[1] // 3
            for i in range(3):
                col        = hsv[:, i*w:(i+1)*w, :]
                means      = col.mean(axis=(0, 1))
                obs[2+i*3] = means[0] / 360.0
                obs[3+i*3] = means[1] / 255.0
                obs[4+i*3] = means[2] / 255.0

        obs[11] = pantilt.pan_angle  / 180.0
        obs[12] = pantilt.tilt_angle / 180.0

        return obs

    # ── reward ────────────────────────────────────────────────────────────────

    def _compute_reward(self, action: int, obs: np.ndarray) -> float:
        reward = 0.0

        if action == 0:
            reward += R_FORWARD
        elif action in (1, 2):
            reward += R_TURN
        elif action == 3:
            reward += R_BACKWARD
            # Small bonus if backward was genuinely needed and opens space
            if self._was_all_blocked:
                fwd_after = obs[0] * MAX_DIST_CM
                if fwd_after > OBSTACLE_CM:
                    reward += R_BACKWARD_ESCAPE
        elif action == 4:
            reward += R_STOP

        dist = obs[0] * MAX_DIST_CM
        if dist < OBSTACLE_CM:
            reward += R_OBSTACLE_NEAR * (1.0 - dist / OBSTACLE_CM)

        if obs[1] > 0.5:
            reward += R_OBSTACLE_HIT

        # Revisit tracking
        if action == 0:
            self._grid_y = np.clip(self._grid_y - 1, 0, 19)
        elif action == 3:
            self._grid_y = np.clip(self._grid_y + 1, 0, 19)
        elif action == 1:
            self._grid_x = np.clip(self._grid_x - 1, 0, 19)
        elif action == 2:
            self._grid_x = np.clip(self._grid_x + 1, 0, 19)

        visits = self._visit_grid[self._grid_y, self._grid_x]
        if visits > 0:
            reward += R_REVISIT * min(visits, 3.0)
        self._visit_grid[self._grid_y, self._grid_x] += 1.0

        return float(reward)

    # ── distance helpers ──────────────────────────────────────────────────────

    def _get_forward_dist(self) -> float:
        """
        Parse forward distance from latest ESP32 STATUS message.
        Falls back to MAX_DIST_CM (assume clear) if unavailable.
        """
        with self._lock:
            msg = self._esp_msg
        try:
            if "dist=" in msg:
                return float(msg.split("dist=")[1].split()[0])
        except Exception:
            pass
        return MAX_DIST_CM

    def _get_side_dists(self) -> tuple:
        """
        Returns (left_dist_cm, right_dist_cm) from latest scan readings.
        Uses SCANNER_SERVO_LEFT_DEG (30) and SCANNER_SERVO_RIGHT_DEG (150).
        Falls back to MAX_DIST_CM if no scan data yet.
        """
        with self._scan_lock:
            readings = dict(self._scan_readings)

        # Angles from hardware_config: LEFT=30, CENTER=90, RIGHT=150
        left  = readings.get(30,  MAX_DIST_CM)
        right = readings.get(150, MAX_DIST_CM)
        return left, right

    # ── helpers ───────────────────────────────────────────────────────────────

    def _capture_frame(self):
        try:
            frame = self.picam2.capture_array()
            return cv2.resize(frame, (FRAME_W, FRAME_H))
        except Exception:
            return None

    def update_esp_message(self, msg: str):
        """Called by ESP32 reader thread on every new message."""
        with self._lock:
            self._esp_msg = msg