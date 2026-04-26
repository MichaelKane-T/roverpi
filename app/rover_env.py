"""
rover_env.py
RoverPi RL Environment

Wraps the real hardware (camera + ESP32 UART) into an RL-compatible
step/reset interface. Designed for the Pi Zero 2W — all ops are
intentionally lightweight.

Observation space (flat numpy array):
  [0]     ultrasonic distance (normalised 0-1, clip at 200cm)
  [1]     esp32_obstacle flag (0 or 1)
  [2:5]   HSV mean of left third of frame   (H/360, S/255, V/255)
  [5:8]   HSV mean of centre third
  [8:11]  HSV mean of right third
  [11]    pan angle (normalised 0-1)
  [12]    tilt angle (normalised 0-1)
  Total: 13 floats

Action space (discrete, 5 actions):
  0 = FORWARD
  1 = LEFT
  2 = RIGHT
  3 = BACKWARD
  4 = STOP
"""

import time
import threading
import numpy as np
import cv2

import pantilt
import serial_comm

# ── tuneable constants ───────────────────────────────────────────────────────
OBSTACLE_CM          = 20.0   # distance below which we penalise / block
MAX_DIST_CM          = 200.0  # normalisation ceiling
IDLE_TIMEOUT_S       = 30.0   # switch back to AUTO after this many idle seconds
FRAME_W, FRAME_H     = 160, 120  # downscale for speed on Zero 2W
STEP_DELAY_S         = 0.15   # seconds between actions
ACTION_CMDS          = ["FORWARD", "LEFT", "RIGHT", "BACKWARD", "STOP"]
OBS_DIM              = 13

# ── reward shaping weights ───────────────────────────────────────────────────
R_FORWARD            =  1.0   # moving forward is good
R_TURN               =  0.2   # turning is ok (exploring)
R_BACKWARD           = -0.3   # backing up is undesirable
R_STOP               = -0.1   # standing still is mildly penalised
R_OBSTACLE_NEAR      = -2.0   # obstacle within threshold
R_OBSTACLE_HIT       = -5.0   # ESP32 reported OBSTACLE STOP
R_REVISIT            = -0.5   # returning to a recently seen grid cell


class RoverEnv:
    def __init__(self, picam2, esp32: serial_comm.ESP32Serial):
        self.picam2  = picam2
        self.esp32   = esp32

        # latest sensor snapshot written by background thread
        self._lock        = threading.Lock()
        self._distance_cm = MAX_DIST_CM
        self._esp_msg     = ""
        self._frame       = None

        # occupancy / revisit tracking (coarse 20×20 grid, 1m cells)
        self._visit_grid  = np.zeros((20, 20), dtype=np.float32)
        self._grid_x      = 10  # start in centre
        self._grid_y      = 10

        # pan/tilt state (degrees)
        self._pan   = pantilt.pan_angle
        self._tilt  = pantilt.tilt_angle

        # last action for reward calculation
        self._last_action = 4  # STOP

        # episode step counter
        self.steps = 0

    # ── public interface ─────────────────────────────────────────────────────

    def reset(self):
        """Send STOP, centre camera, return initial observation."""
        self.esp32.send("STOP")
        pantilt.center()
        self._pan  = pantilt.pan_angle
        self._tilt = pantilt.tilt_angle
        self._last_action = 4
        self.steps = 0
        time.sleep(0.3)
        return self._get_obs()

    def step(self, action: int):
        """
        Execute action, collect next obs, compute reward.
        Returns (obs, reward, done, info)
        """
        assert 0 <= action < len(ACTION_CMDS), f"Invalid action {action}"

        cmd = ACTION_CMDS[action]
        self.esp32.send(cmd)
        self._last_action = action
        self.steps += 1

        time.sleep(STEP_DELAY_S)

        obs    = self._get_obs()
        reward = self._compute_reward(action, obs)
        done   = False   # episodes are open-ended; caller decides when to reset
        info   = {
            "distance_cm": self._distance_cm,
            "esp_msg":     self._esp_msg,
            "step":        self.steps,
        }
        return obs, reward, done, info

    # ── observation builder ──────────────────────────────────────────────────

    def _get_obs(self) -> np.ndarray:
        obs = np.zeros(OBS_DIM, dtype=np.float32)

        # distance (from ESP32 UART message or default)
        dist = self._parse_distance_from_esp()
        with self._lock:
            self._distance_cm = dist
        obs[0] = np.clip(dist / MAX_DIST_CM, 0.0, 1.0)

        # obstacle flag from ESP32
        with self._lock:
            msg = self._esp_msg
        obs[1] = 1.0 if "OBSTACLE" in msg else 0.0

        # vision — three column HSV means
        frame = self._capture_frame()
        if frame is not None:
            hsv = cv2.cvtColor(frame, cv2.COLOR_RGB2HSV)
            w   = hsv.shape[1] // 3
            for i in range(3):
                col         = hsv[:, i*w:(i+1)*w, :]
                means       = col.mean(axis=(0, 1))
                obs[2+i*3]  = means[0] / 360.0
                obs[3+i*3]  = means[1] / 255.0
                obs[4+i*3]  = means[2] / 255.0

        # pan / tilt
        obs[11] = pantilt.pan_angle  / 180.0
        obs[12] = pantilt.tilt_angle / 180.0

        return obs

    # ── reward function ──────────────────────────────────────────────────────

    def _compute_reward(self, action: int, obs: np.ndarray) -> float:
        reward = 0.0

        # action-based base reward
        if action == 0:    reward += R_FORWARD
        elif action in (1, 2): reward += R_TURN
        elif action == 3:  reward += R_BACKWARD
        elif action == 4:  reward += R_STOP

        # obstacle proximity penalty
        dist = obs[0] * MAX_DIST_CM
        if dist < OBSTACLE_CM:
            reward += R_OBSTACLE_NEAR * (1.0 - dist / OBSTACLE_CM)

        # hard obstacle hit from ESP32
        if obs[1] > 0.5:
            reward += R_OBSTACLE_HIT

        # revisit penalty — update coarse grid
        if action == 0:   # moved forward
            self._grid_y = np.clip(self._grid_y - 1, 0, 19)
        elif action == 3:  # backward
            self._grid_y = np.clip(self._grid_y + 1, 0, 19)
        elif action == 1:  # left
            self._grid_x = np.clip(self._grid_x - 1, 0, 19)
        elif action == 2:  # right
            self._grid_x = np.clip(self._grid_x + 1, 0, 19)

        visits = self._visit_grid[self._grid_y, self._grid_x]
        if visits > 0:
            reward += R_REVISIT * min(visits, 3.0)
        self._visit_grid[self._grid_y, self._grid_x] += 1.0

        return float(reward)

    # ── helpers ──────────────────────────────────────────────────────────────

    def _capture_frame(self):
        try:
            frame = self.picam2.capture_array()
            return cv2.resize(frame, (FRAME_W, FRAME_H))
        except Exception:
            return None

    def _parse_distance_from_esp(self) -> float:
        """
        Pull latest UART message; parse 'STATUS dist=XX.X ...' if present.
        Falls back to MAX_DIST_CM if no distance available.
        """
        with self._lock:
            msg = self._esp_msg
        try:
            if "dist=" in msg:
                part = msg.split("dist=")[1].split()[0]
                return float(part)
        except Exception:
            pass
        return MAX_DIST_CM

    def update_esp_message(self, msg: str):
        """Called by the UART reader thread whenever a new message arrives."""
        with self._lock:
            self._esp_msg = msg