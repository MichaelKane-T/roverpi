"""
occupancy_map.py
Lightweight occupancy grid for RoverPi exploration.

The ESP32 scanner servo sweeps 30°→90°→150° and reports distances.
This module maintains a 2D grid in the Pi's memory and provides
a PNG render for the dashboard.

Grid:
  - Each cell = 10cm × 10cm
  - 40×40 grid = 4m × 4m map
  - Origin (rover start) at centre
  - Values: 0.0 = unknown, 0.5 = free, 1.0 = occupied
"""

import numpy as np
import math
import threading
import cv2

GRID_SIZE    = 40          # cells per axis
CELL_CM      = 10.0        # cm per cell
ORIGIN       = GRID_SIZE // 2   # rover starts at centre
MAX_RANGE_CM = 200.0

# confidence update amounts (log-odds style, simplified)
HIT_INC  =  0.4
MISS_DEC = -0.1
CLAMP    = (-2.0, 2.0)


class OccupancyMap:
    def __init__(self):
        self._lock    = threading.Lock()
        # log-odds grid
        self._log     = np.zeros((GRID_SIZE, GRID_SIZE), dtype=np.float32)
        # rover pose in grid coords
        self._rx      = float(ORIGIN)
        self._ry      = float(ORIGIN)
        self._heading = 0.0   # degrees, 0 = forward (+Y), 90 = right (+X)

    # ── update from ultrasonic sweep ─────────────────────────────────────────

    def update_sweep(self, readings: dict):
        """
        readings: {angle_deg: distance_cm, ...}
        angle_deg is relative to rover heading (0 = straight ahead).
        Called after each ESP32 scanner sweep.
        """
        with self._lock:
            for rel_angle, dist_cm in readings.items():
                abs_angle = math.radians(self._heading + rel_angle)
                self._ray_cast(abs_angle, dist_cm)

    def _ray_cast(self, angle_rad: float, dist_cm: float):
        """Mark cells along ray as free; mark endpoint as occupied."""
        steps = int(min(dist_cm, MAX_RANGE_CM) / CELL_CM)
        for s in range(steps + 1):
            cx = int(self._rx + s * math.sin(angle_rad))
            cy = int(self._ry - s * math.cos(angle_rad))   # -Y = forward
            if not self._in_bounds(cx, cy):
                break
            if s < steps:
                # free cell
                self._log[cy, cx] = np.clip(
                    self._log[cy, cx] + MISS_DEC, *CLAMP
                )
            else:
                # occupied only if within sensor max range
                if dist_cm < MAX_RANGE_CM:
                    self._log[cy, cx] = np.clip(
                        self._log[cy, cx] + HIT_INC, *CLAMP
                    )

    # ── pose update from odometry ────────────────────────────────────────────

    def move(self, action: int):
        """
        Coarse dead-reckoning — updates rover position in grid.
        One action ≈ one cell movement.
        """
        with self._lock:
            angle_rad = math.radians(self._heading)
            if action == 0:   # FORWARD
                self._ry -= math.cos(angle_rad)
                self._rx += math.sin(angle_rad)
            elif action == 3:  # BACKWARD
                self._ry += math.cos(angle_rad)
                self._rx -= math.sin(angle_rad)
            elif action == 1:  # LEFT — rotate
                self._heading = (self._heading - 15) % 360
            elif action == 2:  # RIGHT — rotate
                self._heading = (self._heading + 15) % 360

            # clamp to grid
            self._rx = np.clip(self._rx, 0, GRID_SIZE - 1)
            self._ry = np.clip(self._ry, 0, GRID_SIZE - 1)

    # ── render to image for dashboard ────────────────────────────────────────

    def render(self, cell_px: int = 12) -> bytes:
        """
        Returns JPEG bytes of the current occupancy grid.
        Suitable for serving via Flask.
        """
        with self._lock:
            log = self._log.copy()
            rx, ry = int(self._rx), int(self._ry)

        # convert log-odds to probability 0-1
        prob = 1.0 / (1.0 + np.exp(-log))

        # colour mapping
        # unknown (≈0.5) → grey, free (<0.5) → white, occupied (>0.5) → black
        img = np.ones((GRID_SIZE, GRID_SIZE, 3), dtype=np.uint8) * 180  # grey

        free_mask = prob < 0.45
        occ_mask  = prob > 0.55
        img[free_mask] = [255, 255, 255]
        img[occ_mask]  = [30,  30,  30]

        # scale up
        img = cv2.resize(
            img,
            (GRID_SIZE * cell_px, GRID_SIZE * cell_px),
            interpolation=cv2.INTER_NEAREST
        )

        # draw rover position
        px = rx * cell_px + cell_px // 2
        py = ry * cell_px + cell_px // 2
        cv2.circle(img, (px, py), cell_px // 2, (0, 180, 0), -1)

        # draw heading arrow
        angle_rad = math.radians(self._heading)
        ex = int(px + cell_px * 1.5 * math.sin(angle_rad))
        ey = int(py - cell_px * 1.5 * math.cos(angle_rad))
        cv2.arrowedLine(img, (px, py), (ex, ey), (0, 255, 0), 2, tipLength=0.4)

        ok, buf = cv2.imencode(".jpg", img)
        return buf.tobytes() if ok else b""

    # ── helpers ───────────────────────────────────────────────────────────────

    def _in_bounds(self, x: int, y: int) -> bool:
        return 0 <= x < GRID_SIZE and 0 <= y < GRID_SIZE

    def get_free_ratio(self) -> float:
        """Fraction of known-free cells — proxy for exploration progress."""
        with self._lock:
            total = GRID_SIZE * GRID_SIZE
            free  = np.sum(self._log < -0.1)
        return float(free) / total