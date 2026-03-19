from src.hardware import RobotHardware
from src.robot import ObjectHunterRobot
from src.vision import Vision
from src.config import VISION_BASE_URL, ESP32_IP, ESP32_PORT
from src.logger_factory import get_logger
import socket

logger = get_logger(__name__, log_file=f"logs/{__name__}.log")

def main() -> None:
    # Socket connection to ESP32
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.settimeout(5.0)
    sock.connect((ESP32_IP, ESP32_PORT))
    logger.info("[MAIN] Connected to Robot!")

    # Wait for READY message
    sock.settimeout(2.0)
    data = sock.recv(1024).decode(errors="ignore")
    logger.info("[MAIN] Received status: %s", data.strip())

    # Vision system instance
    vision = Vision(VISION_BASE_URL)

    hardware = RobotHardware(
        sock=sock,
        vision=vision,
    )
    robot = ObjectHunterRobot(hardware)

    try:
        robot.run()
    except KeyboardInterrupt:
        robot.stop()
        sock.close()
        logger.info("[MAIN] Connection closed.")


if __name__ == "__main__":
    main()