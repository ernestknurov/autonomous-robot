import time
from urllib.parse import urljoin

import cv2
import numpy as np
import requests

from src.logger_factory import get_logger


logger = get_logger(__name__, log_file=f"logs/{__name__}.log")

class Vision:
    """
    Uses IP Webcam's HTTP API to get frames and do ArUco detection. 
    Using requests instead of OpenCV's VideoCapture for simplicity."""
    def __init__(self, base_url: str):
        self.base_url = base_url
        self.mjpeg_urls = [urljoin(base_url, "videofeed"), urljoin(base_url, "video")]
        self.shot_url = urljoin(base_url, "shot.jpg")

        aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        params = cv2.aruco.DetectorParameters()
        self.detector = cv2.aruco.ArucoDetector(aruco_dict, params)
        self.session = requests.Session()

    def get_frame(self) -> np.ndarray:
        # For simplicity, we skip MJPEG streaming and just poll shot.jpg.
        try:
            r = self.session.get(self.shot_url, timeout=1)
            r.raise_for_status()
            img = cv2.imdecode(np.frombuffer(r.content, dtype=np.uint8), cv2.IMREAD_COLOR)
            if img is None:
                logger.error("[VISION] Failed to decode JPEG frame from %s", self.shot_url)
                return np.zeros((480, 640, 3), dtype=np.uint8)  # Return black frame on decode failure
            return img
        except Exception as e:
            logger.error("[VISION] Error fetching frame from %s: %s", self.shot_url, e)
            return np.zeros((480, 640, 3), dtype=np.uint8)  # Return black frame on error
    
    def detect_markers(self, frame: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = self.detector.detectMarkers(gray)
        if ids is not None:
            return corners, ids
        return [], []

    def process_frame(self, frame: np.ndarray) -> np.ndarray:
        # frame: BGR image (H, W, 3). 
        cv2.putText(frame, time.strftime("%H:%M:%S"), (20, 40),
                    cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255, 255, 255), 2, cv2.LINE_AA)
        
        # ArUco detection
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = self.detector.detectMarkers(gray)
        if ids is not None:
            cv2.aruco.drawDetectedMarkers(frame, corners, ids)
        return frame
