import time
import requests
from urllib.parse import urljoin

import cv2
import torch
import numpy as np

from src.logger_factory import get_logger
from src.schemas import DepthHazard
from src.config import (
    DEPTH_ESTIMATION_BLOCKED_THRESHOLD,
    AREA_OF_INTEREST,
    CLOSE_WIRING_MASK
)

logger = get_logger(__name__, log_file=f"logs/{__name__}.log", level="INFO")


def aruco_marker_points(marker_corners: np.ndarray) -> np.ndarray:
    return marker_corners.reshape(-1, 2)


class DepthEstimator:
    def __init__(self, model_type="DPT_Hybrid", device: str | None = None):
        self.midas = torch.hub.load("intel-isl/MiDaS", model_type)
        self.device = self._select_device(device)
        self._move_model_to_device()
        self.midas.eval()

        midas_transforms = torch.hub.load("intel-isl/MiDaS", "transforms")
        if model_type == "DPT_Large" or model_type == "DPT_Hybrid":
            self.transform = midas_transforms.dpt_transform
        else:
            self.transform = midas_transforms.small_transform

    def _select_device(self, requested_device: str | None) -> torch.device:
        if requested_device is not None:
            return torch.device(requested_device)

        return torch.device("cuda") if torch.cuda.is_available() else torch.device("cpu")

    def _move_model_to_device(self) -> None:
        try:
            self.midas.to(self.device)
        except torch.AcceleratorError as exc:
            if self.device.type != "cuda" or "out of memory" not in str(exc).lower():
                raise
            logger.warning(
                "[VISION] CUDA out of memory while loading MiDaS. Falling back to CPU."
            )
            if torch.cuda.is_available():
                torch.cuda.empty_cache()
            self.device = torch.device("cpu")
            self.midas.to(self.device)

    def _fallback_to_cpu(self) -> None:
        if self.device.type != "cuda":
            raise RuntimeError("CPU fallback requested while already on CPU")
        logger.warning("[VISION] CUDA out of memory during depth estimation. Switching to CPU.")
        if torch.cuda.is_available():
            torch.cuda.empty_cache()
        self.device = torch.device("cpu")
        self.midas.to(self.device)

    def estimate(self, img):
        input_batch = self.transform(img).to(self.device)

        with torch.no_grad():
            try:
                prediction = self.midas(input_batch)
            except torch.AcceleratorError as exc:
                if self.device.type != "cuda" or "out of memory" not in str(exc).lower():
                    raise
                self._fallback_to_cpu()
                input_batch = input_batch.to(self.device)
                prediction = self.midas(input_batch)

            prediction = torch.nn.functional.interpolate(
                prediction.unsqueeze(1),
                size=img.shape[:2],
                mode="bicubic",
                align_corners=False,
            ).squeeze()
            # normalize to [0, 1]
            prediction -= prediction.min()
            prediction /= prediction.max()

        return prediction.cpu().numpy()
    
def analyze_depth_hazard(depth_map: np.ndarray) -> DepthHazard:
    """Function that analyze depth map and tells is it safe to go forward"""

    depth_map = depth_map.copy()
    height, width = depth_map.shape
    x0 = int(width * AREA_OF_INTEREST[0])
    x1 = int(width * AREA_OF_INTEREST[1])
    y0 = int(height * AREA_OF_INTEREST[2])
    y1 = int(height * AREA_OF_INTEREST[3])

    # Ignore known camera artifacts (close wiring in camera view).
    depth_map[int(height * CLOSE_WIRING_MASK[0]):, :int(width * CLOSE_WIRING_MASK[1])] = 0.0

    roi = depth_map[y0:y1, x0:x1].copy()
    valid = roi[roi > 0.0]
    q90 = float(np.percentile(valid, 90))
    blocked = q90 >= DEPTH_ESTIMATION_BLOCKED_THRESHOLD
    return DepthHazard(blocked=blocked, depth_score=q90)


class Vision:
    """
    Uses IP Webcam's HTTP API to get frames and do ArUco detection.
    Using requests instead of OpenCV's VideoCapture for simplicity."""

    def __init__(self, base_url: str, depth_estimator: DepthEstimator | None = None):
        self.base_url = base_url
        self.mjpeg_urls = [urljoin(base_url, "videofeed"), urljoin(base_url, "video")]
        self.shot_url = urljoin(base_url, "shot.jpg")
        self.depth_estimator = depth_estimator

        aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        params = cv2.aruco.DetectorParameters()
        self.detector = cv2.aruco.ArucoDetector(aruco_dict, params)
        self.session = requests.Session()

    def get_frame(self) -> np.ndarray:
        # For simplicity, we skip MJPEG streaming and just poll shot.jpg.
        try:
            r = self.session.get(self.shot_url, timeout=1)
            r.raise_for_status()
            img_raw = cv2.imdecode(np.frombuffer(r.content, dtype=np.uint8), cv2.IMREAD_COLOR)
            if img_raw is None:
                logger.error("[VISION] Failed to decode JPEG frame from %s", self.shot_url)
                return np.zeros((480, 640, 3), dtype=np.uint8)
            img = cv2.rotate(img_raw, cv2.ROTATE_180)
            return img
        except Exception as e:
            logger.error("[VISION] Error fetching frame from %s: %s", self.shot_url, e)
            return np.zeros((480, 640, 3), dtype=np.uint8)

    def detect_markers(self, frame: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = self.detector.detectMarkers(gray)
        if ids is not None:
            return corners, ids
        return [], []

    def estimate_depth_hazard(self, frame: np.ndarray) -> DepthHazard:
        if self.depth_estimator is None:
            return DepthHazard()

        depth_map = self.depth_estimator.estimate(frame)
        return analyze_depth_hazard(depth_map)

    def process_frame(self, frame: np.ndarray) -> np.ndarray:
        cv2.putText(
            frame,
            time.strftime("%H:%M:%S"),
            (20, 40),
            cv2.FONT_HERSHEY_SIMPLEX,
            1.0,
            (255, 255, 255),
            2,
            cv2.LINE_AA,
        )

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = self.detector.detectMarkers(gray)
        if ids is not None:
            cv2.aruco.drawDetectedMarkers(frame, corners, ids)
        return frame
