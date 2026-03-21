import re
import cv2
import time
import socket
import numpy as np
from src.vision import Vision
from src.logger_factory import get_logger
from src.schemas import SensorSnapshot, MarkerDetection
from src.config import VISION_RESOLUTION, MAX_MOVE_M, MAX_TURN_DEG, DEFAULT_COMMAND_TIMEOUT_S, GET_DISTANCE_TIMEOUT_S     

logger = get_logger(__name__, log_file=f"logs/{__name__}.log", level="DEBUG")

class RobotHardware:
    """
    Abstraction over the actual hardware.
    This is where you'll later plug in your motor control classes,
    ultrasonic reading, and frame/detection retrieval.
    """
    def __init__(self, sock: socket.socket, vision: Vision):
        self.sock = sock
        self.vision = vision

    def read_sensors(self) -> SensorSnapshot:
        """
        Return the current snapshot:
        - distance to obstacle
        - result of ArUco / YOLO detection
        Note: for now we allow only one Aruco marker to be detected.
        """
        # Shapes:
        # bbox: (N, 4, 2). Each detection has a bounding box with 4 corners and (x, y) coordinates.
        # ids: (N, 1). Each detection has an integer ID.
        frame = self.vision.get_frame()
        bbox, ids = self.vision.detect_markers(frame)

        marker_found = ids is not None and len(ids) > 0
        x_offset, area = 0.0, 0.0
        corners, marker_id = None, None

        if marker_found:
            corners = bbox[0]  # shape (4, 2)
            marker_id = int(ids[0][0])
            marker_center_x = np.mean(corners[:, 0])  # Average x of the 4 corners
            image_center_x = VISION_RESOLUTION[0] / 2
            x_offset = (marker_center_x - image_center_x) / image_center_x  # Normalize to [-1, 1]
            area = cv2.contourArea(corners[0]) / (VISION_RESOLUTION[0] * VISION_RESOLUTION[1])  # Relative area

        distance_to_obstacle = self.get_distance()

        snapshot = SensorSnapshot(
            obstacle_distance_cm=distance_to_obstacle,
            marker=MarkerDetection(
                visible=marker_found,
                x_offset=x_offset,
                area=area,
                corners=corners,
                marker_id=marker_id
            )
        )
        logger.debug(f"[HARDWARE] Sensor snapshot: marker_id = {marker_id}, x_offset = {x_offset:<10.3f}, area = {area:<10.5f}, distance = {distance_to_obstacle:<10.3f} cm")
        return snapshot

    def get_distance(self) -> float:
        # Get response like "DISTANCE 23.5\nDONE\n"
        response = self.send_and_wait_done("GET_DISTANCE", timeout_s=GET_DISTANCE_TIMEOUT_S)
        if not response:
            logger.error("[HARDWARE] No response for GET_DISTANCE command")
            return float('inf')
        
        # Get the value after "DISTANCE " and before the next newline
        m = re.search(r'DISTANCE\s+([0-9.]+)', response)
        if not m:
            logger.error("[HARDWARE] Distance pattern not found in response: %s", response)
            return float('inf')
        
        distance = float(m.group(1))
        return distance
    
    def move_forward(self, distance: float = 0.3, timeout_s: float = DEFAULT_COMMAND_TIMEOUT_S) -> str | None:
        distance = self.clip(distance, 0.0, MAX_MOVE_M)
        response = self.send_and_wait_done(f"MOVE {distance:.2f}", timeout_s=timeout_s)
        return response

    def move_backward(self, distance: float = 0.3, timeout_s: float = DEFAULT_COMMAND_TIMEOUT_S) -> str | None:
        distance = self.clip(distance, 0.0, MAX_MOVE_M)
        response = self.send_and_wait_done(f"MOVE {-distance:.2f}", timeout_s=timeout_s)
        return response

    def turn_left(self, degrees: int = 90, timeout_s: float = DEFAULT_COMMAND_TIMEOUT_S) -> str | None:
        degrees = self.clip(degrees, 0, MAX_TURN_DEG)
        response = self.send_and_wait_done(f"TURN {-degrees}", timeout_s=timeout_s)
        return response

    def turn_right(self, degrees: int = 90, timeout_s: float = DEFAULT_COMMAND_TIMEOUT_S) -> str | None:
        degrees = self.clip(degrees, 0, MAX_TURN_DEG)
        response = self.send_and_wait_done(f"TURN {degrees}", timeout_s=timeout_s)
        return response
    
    def stop(self) -> None:
        "Doesn't make sense for now, since we have blocking MOVE commands, but let's keep it for future use"
        pass
    
    def clip(self, x: float, lo: float, hi: float) -> float:
        return max(lo, min(hi, x))

    def send_and_wait_done(self, cmd: str, timeout_s: float = DEFAULT_COMMAND_TIMEOUT_S) -> str | None:
        """Send command and wait for DONE response."""
        self.sock.sendall((cmd.strip() + "\n").encode("utf-8"))
        
        self.sock.settimeout(timeout_s)
        deadline = time.time() + timeout_s
        buffer = ""
        response = ""
        
        while time.time() < deadline:
            try:
                data = self.sock.recv(1024)
                if not data:
                    raise ConnectionError("Connection closed by ESP32")
                
                buffer += data.decode(errors="ignore")
                lines = buffer.split("\n")
                buffer = lines[-1]  # Keep incomplete line in buffer
                
                for line in lines[:-1]:
                    text = line.strip()
                    if text:
                        response += "<< " + text + "\n"
                    if text == "DONE":
                        return response
            except socket.timeout:
                continue
        
        logger.error("[HARDWARE] Timed out waiting DONE for command: %s. Last response: %s", cmd, response)
