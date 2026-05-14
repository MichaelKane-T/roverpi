'''
*=========================================================================
 * ml.py
 *
 * Placeholder ML module for RoverPi. Implements a simple vision-based scene
 * classifier using OpenCV. In a real implementation, this would be replaced
 * with a more sophisticated model (e.g. CNN) and possibly integrated with
 * sensor fusion for better decision-making.
 *
 * Created on: April 22, 2026
 *=================================================================
'''

import cv2
import numpy as np


class SimpleVision:
    def analyze_features(self, frame):
        small = cv2.resize(frame, (160, 120))
        hsv = cv2.cvtColor(small, cv2.COLOR_RGB2HSV)

        h, w, _ = hsv.shape
        thirds = [
            hsv[:, :w//3, :],
            hsv[:, w//3:2*w//3, :],
            hsv[:, 2*w//3:, :],
        ]

        features = []

        for region in thirds:
            mean_h = float(region[:, :, 0].mean()) / 180.0
            mean_s = float(region[:, :, 1].mean()) / 255.0
            mean_v = float(region[:, :, 2].mean()) / 255.0
            edge_density = self._edge_density(region)

            features.extend([mean_h, mean_s, mean_v, edge_density])

        return features

    def _edge_density(self, hsv_region):
        gray = cv2.cvtColor(hsv_region, cv2.COLOR_HSV2BGR)
        gray = cv2.cvtColor(gray, cv2.COLOR_BGR2GRAY)
        edges = cv2.Canny(gray, 50, 150)
        return float(np.mean(edges > 0))