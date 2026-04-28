'''=================================================================
 * camera.py
 *
 * Python module for handling camera input using OpenCV.
 *
 * Created on: Jun 6, 2024
 * Author: Michael Kane
 * =================================================================
 '''
import cv2

class CameraStream:
    def __init__(self, camera_index=0, width=640, height=480):
        self.cap = cv2.VideoCapture(camera_index)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)

    def get_frame(self):
        ret, frame = self.cap.read()
        if not ret:
            return None
        return frame

    def release(self):
        self.cap.release()