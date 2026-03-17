import socket
import time
import re
from dataclasses import asdict
from src.schemas import SensorSnapshot, MarkerDetection
from src.vision import Vision
from src.logger_factory import get_logger
from src.config import TIMEOUT_S, MAX_MOVE_M, MAX_TURN_DEG     

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
        """
        frame = self.vision.get_frame()
        # Shapes:
        # corners: (N, 4, 2). Each detection has 4 corners with (x, y) coordinates.
        # ids: (N, 1). Each detection has an integer ID.
        corners, ids = self.vision.detect_markers(frame)
        distance_to_obstacle = self.get_distance()

        snapshot = SensorSnapshot(
            obstacle_distance_cm=distance_to_obstacle,
            marker=MarkerDetection(
                visible=ids is not None and len(ids) > 0,
                corners=corners[0] if len(corners) > 0 else None,
                marker_id=int(ids[0][0]) if ids is not None and len(ids) > 0 else None
            )
        )
        logger.debug("Sensor snapshot: %s", asdict(snapshot))
        return snapshot

    def get_distance(self) -> float:
        # Get response like "DISTANCE 23.5\nDONE\n"
        response = self.send_and_wait_done("GET_DISTANCE", timeout_s=3.0)
        if not response:
            logger.error("No response for GET_DISTANCE command")
            return float('inf')
        
        # Get the value after "DISTANCE " and before the next newline
        m = re.search(r'DISTANCE\s+([0-9.]+)', response)
        if not m:
            logger.error("Distance pattern not found in response: %s", response)
            return float('inf')
        
        distance = float(m.group(1))
        return distance
    
    def move_forward(self, distance: float = 0.3, timeout_s: float = 20.0) -> str | None:
        distance = self.clip(distance, 0.0, MAX_MOVE_M)
        response = self.send_and_wait_done(f"MOVE {distance:.2f}", timeout_s=timeout_s)
        return response

    def move_backward(self, distance: float = 0.3, timeout_s: float = 20.0) -> str | None:
        distance = self.clip(distance, 0.0, MAX_MOVE_M)
        response = self.send_and_wait_done(f"MOVE {-distance:.2f}", timeout_s=timeout_s)
        return response

    def turn_left(self, degrees: int = 90, timeout_s: float = 20.0) -> str | None:
        degrees = self.clip(degrees, 0, MAX_TURN_DEG)
        response = self.send_and_wait_done(f"TURN {-degrees}", timeout_s=timeout_s)
        return response

    def turn_right(self, degrees: int = 90, timeout_s: float = 20.0) -> str | None:
        degrees = self.clip(degrees, 0, MAX_TURN_DEG)
        response = self.send_and_wait_done(f"TURN {degrees}", timeout_s=timeout_s)
        return response
    
    def stop(self) -> None:
        "Doesn't make sense for now, since we have blocking MOVE commands, but let's keep it for future use"
        pass
    
    def clip(self, x: float, lo: float, hi: float) -> float:
        return max(lo, min(hi, x))

    def send_and_wait_done(self, cmd: str, timeout_s: float = 20.0) -> str | None:
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
        
        logger.error("Timed out waiting DONE for command: %s. Last response: %s", cmd, response)
