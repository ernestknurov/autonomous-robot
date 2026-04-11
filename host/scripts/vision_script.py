import time
import logging
from urllib.parse import urljoin

import cv2
import numpy as np
import requests
from src.schemas import DepthHazard
from src.vision import DepthEstimator, analyze_depth_hazard

from src.config import (
    AREA_OF_INTEREST,
    CLOSE_WIRING_MASK,
    VISION_BASE_URL as BASE_URL,
    DEPTH_ESTIMATOR_MODEL_TYPE
)

logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s %(message)s")
depth_estimator = DepthEstimator(model_type=DEPTH_ESTIMATOR_MODEL_TYPE)


MJPEG_URLS = [urljoin(BASE_URL, "videofeed"), urljoin(BASE_URL, "video")]
SHOT_URL = urljoin(BASE_URL, "shot.jpg")

aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
params = cv2.aruco.DetectorParameters()
detector = cv2.aruco.ArucoDetector(aruco_dict, params)


def estimate_depth_hazard(frame: np.ndarray) -> tuple[np.ndarray, DepthHazard, dict]:
    depth_frame = depth_estimator.estimate(frame)
    hazard = analyze_depth_hazard(depth_frame)
    debug = build_simple_depth_debug(depth_frame, hazard)
    return depth_frame, hazard, debug


def build_simple_depth_debug(depth_map: np.ndarray, hazard: DepthHazard) -> dict:
    depth_map = depth_map.copy()
    height, width = depth_map.shape
    x0 = int(width * AREA_OF_INTEREST[0])
    x1 = int(width * AREA_OF_INTEREST[1])
    y0 = int(height * AREA_OF_INTEREST[2])
    y1 = int(height * AREA_OF_INTEREST[3])

    # Ignore known camera artifacts (close wiring in camera view).
    depth_map[int(height * CLOSE_WIRING_MASK[0]):, :int(width * CLOSE_WIRING_MASK[1])] = 0.0

    debug = {
        "roi": (x0, y0, x1, y1),
        "wiring_mask": None,
        "q90": 0.0,
        "blocked": hazard.blocked,
    }

    if x1 <= x0 or y1 <= y0:
        return debug

    roi = depth_map[y0:y1, x0:x1].copy()
    if roi.size == 0:
        return debug

    mask_y0 = int(height * CLOSE_WIRING_MASK[0])
    mask_x1 = int(width * CLOSE_WIRING_MASK[1])
    debug["wiring_mask"] = (0, mask_y0, mask_x1, height)

    valid = roi[roi > 0.0]
    if valid.size > 0:
        debug["q90"] = float(np.percentile(valid, 90))
    return debug

def log_depth_hazard(frame: np.ndarray) -> None:
    _, hazard, debug = estimate_depth_hazard(frame)
    logging.info(
        "[DEPTH] blocked=%s, q90=%.3f, scores=(left=%.3f, center=%.3f, right=%.3f), recommended_turn=%s",
        hazard.blocked,
        debug.get("q90", 0.0),
        hazard.left_score,
        hazard.center_score,
        hazard.right_score,
        hazard.recommended_turn,
    )

def visualize_depth_hazard(frame: np.ndarray) -> np.ndarray:
    depth_frame, hazard, debug = estimate_depth_hazard(frame)
    x0, y0, x1, y1 = debug["roi"]
    depth_colored = cv2.applyColorMap(
        (depth_frame * 255).astype(np.uint8), cv2.COLORMAP_INFERNO
    )
    visualized = cv2.addWeighted(frame, 0.1, depth_colored, 0.9, 0)

    if x1 <= x0 or y1 <= y0:
        return visualized

    wiring_mask = debug.get("wiring_mask")
    if wiring_mask is not None:
        mx0, my0, mx1, my1 = wiring_mask
        cv2.rectangle(visualized, (mx0, my0), (mx1, my1), (0, 0, 0), -1)
        cv2.rectangle(visualized, (mx0, my0), (mx1, my1), (255, 255, 0), 2)

    cv2.rectangle(visualized, (x0, y0), (x1, y1), (255, 255, 255), 2)

    status_text = "BLOCKED" if hazard.blocked else "CLEAR"
    status_color = (0, 0, 255) if hazard.blocked else (0, 255, 0)
    cv2.putText(
        visualized,
        f"{status_text} q90={debug.get('q90', 0.0):.3f}",
        (x0 + 10, y1 + 25),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.7,
        status_color,
        2,
        cv2.LINE_AA,
    )

    cv2.putText(
        visualized,
        "ROI",
        (x0 + 10, max(y0 - 10, 25)),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.7,
        (255, 255, 255),
        2,
        cv2.LINE_AA,
    )

    if wiring_mask is not None:
        cv2.putText(
            visualized,
            "WIRING MASK",
            (wiring_mask[0] + 8, max(wiring_mask[1] - 8, 25)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            (255, 255, 0),
            2,
            cv2.LINE_AA,
        )

    return visualized

def process_frame(frame: np.ndarray) -> np.ndarray:
    img = cv2.rotate(frame, cv2.ROTATE_180)
    visualized = visualize_depth_hazard(img)

    cv2.putText(visualized, time.strftime("%H:%M:%S"), (20, 40),
                cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255, 255, 255), 2, cv2.LINE_AA)

    # ArUco detection
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    corners, ids, rejected = detector.detectMarkers(gray)
    if ids is not None:
        cv2.aruco.drawDetectedMarkers(visualized, corners, ids)
        # print("Detected ids:", ids.flatten().tolist())
    return visualized

def try_opencv_stream(url: str) -> cv2.VideoCapture | None:
    cap = cv2.VideoCapture(url)
    if not cap.isOpened():
        cap.release()
        return None
    # Warm-up read
    ok, _ = cap.read()
    if not ok:
        cap.release()
        return None
    return cap

def mjpeg_loop(url: str) -> None:
    logging.info("Trying MJPEG/OpenCV stream: %s", url)
    cap = try_opencv_stream(url)
    if cap is None:
        raise RuntimeError("OpenCV cannot open stream")
    logging.info("MJPEG stream opened")
    try:
        while True:
            ok, frame = cap.read()
            if not ok or frame is None:
                raise RuntimeError("Stream read failed (reconnect needed)")
            processed = process_frame(frame)
            cv2.imshow("IP Webcam", processed)
            key = cv2.waitKey(1) & 0xFF
            if key == 32:
                log_depth_hazard(cv2.rotate(frame, cv2.ROTATE_180))
            if key == 27:  # ESC
                break
    finally:
        cap.release()

def shot_loop(fps: float = 10.0) -> None:
    logging.info("Falling back to shot.jpg polling: %s", SHOT_URL)
    period_s = 1.0 / max(fps, 0.1)
    s = requests.Session()
    try:
        while True:
            t0 = time.time()
            r = s.get(SHOT_URL, timeout=5)
            r.raise_for_status()
            img = cv2.imdecode(np.frombuffer(r.content, dtype=np.uint8), cv2.IMREAD_COLOR)
            if img is None:
                logging.warning("Failed to decode JPEG frame")
                continue
            processed = process_frame(img)
            cv2.namedWindow("Camera", cv2.WINDOW_NORMAL)   # resizable window
            cv2.resizeWindow("Camera", 960, 540)          # optional starting size
            cv2.imshow("Camera", processed)

            key = cv2.waitKey(1) & 0xFF
            if key == 32:
                log_depth_hazard(cv2.rotate(img, cv2.ROTATE_180))
            if key == 27:  # ESC
                break
            dt = time.time() - t0
            if dt < period_s:
                time.sleep(period_s - dt)
    finally:
        s.close()

def main() -> None:
    # last_err = None
    # for url in MJPEG_URLS:
    #     try:
    #         mjpeg_loop(url)
    #         return
    #     except Exception as e:
    #         last_err = e
    #         logging.warning("MJPEG failed for %s: %s", url, e)
    # logging.info("All MJPEG options failed, switching to shot.jpg. Last error: %s", last_err)
    shot_loop(fps=10.0)

if __name__ == "__main__":
    try:
        main()
    finally:
        cv2.destroyAllWindows()
