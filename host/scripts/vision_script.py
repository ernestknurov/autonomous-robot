import time
import logging
from urllib.parse import urljoin

import cv2
import numpy as np
import requests

logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s %(message)s")

BASE_URL = "http://192.168.0.139:8080/"
# BASE_URL = "http://192.168.107.74:8080/"
MJPEG_URLS = [urljoin(BASE_URL, "videofeed"), urljoin(BASE_URL, "video")]
SHOT_URL = urljoin(BASE_URL, "shot.jpg")

aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
params = cv2.aruco.DetectorParameters()
detector = cv2.aruco.ArucoDetector(aruco_dict, params)

def process_frame(frame: np.ndarray) -> np.ndarray:
    # frame: BGR image (H, W, 3). Put your logic here.
    # Example: draw timestamp
    cv2.putText(frame, time.strftime("%H:%M:%S"), (20, 40),
                cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255, 255, 255), 2, cv2.LINE_AA)
    
    # ArUco detection
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    corners, ids, rejected = detector.detectMarkers(gray)
    if ids is not None:
        cv2.aruco.drawDetectedMarkers(frame, corners, ids)
        # print("Detected ids:", ids.flatten().tolist())
    return frame

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
            frame = process_frame(frame)
            cv2.imshow("IP Webcam", frame)
            if cv2.waitKey(1) & 0xFF == 27:  # ESC
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
            img = process_frame(img)
            cv2.imshow("IP Webcam", img)
            if cv2.waitKey(1) & 0xFF == 27:  # ESC
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
