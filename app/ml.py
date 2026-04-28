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
    def __init__(self):
        pass

    def analyze(self, frame):
        """
        Very lightweight placeholder logic.
        Returns an annotated frame and a label.
        """
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        mean_brightness = np.mean(gray)

        if mean_brightness < 60:
            label = "dark_scene"
        elif mean_brightness > 180:
            label = "bright_scene"
        else:
            label = "normal_scene"

        annotated = frame.copy()
        cv2.putText(
            annotated,
            f"Scene: {label}",
            (20, 40),
            cv2.FONT_HERSHEY_SIMPLEX,
            1,
            (0, 255, 0),
            2
        )

        return annotated, label