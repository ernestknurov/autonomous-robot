import argparse
import select
import socket
import sys
import termios
import tty

from src.hardware import RobotHardware
from src.robot import ObjectHunterRobot
from src.vision import Vision, DepthEstimator
from src.config import VISION_BASE_URL, ESP32_IP, ESP32_PORT, DEPTH_ESTIMATOR_MODEL_TYPE
from src.logger_factory import get_logger

logger = get_logger(__name__, log_file=f"logs/{__name__}.log")

def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--run-manually", action="store_true", help="Run one robot iteration when space is pressed.")
    return parser.parse_args()

def wait_for_manual_key() -> str:
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        while True:
            ready, _, _ = select.select([sys.stdin], [], [])
            if ready:
                return sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)

def run_manually(robot: ObjectHunterRobot) -> None:
    logger.info("[MAIN] Manual mode enabled. Press space to run one iteration, q to quit.")

    while True:
        key = wait_for_manual_key()
        if key == "q":
            logger.info("[MAIN] Manual mode interrupted by user.")
            break
        if key != " ":
            continue
        if not robot.run_iteration():
            logger.info("[MAIN] Robot finished or stopped.")
            break

def main() -> None:
    args = parse_args()

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
    vision = Vision(VISION_BASE_URL, depth_estimator=DepthEstimator(model_type=DEPTH_ESTIMATOR_MODEL_TYPE))

    hardware = RobotHardware(
        sock=sock,
        vision=vision,
    )
    robot = ObjectHunterRobot(hardware)

    try:
        if args.run_manually:
            run_manually(robot)
        else:
            robot.run()
    except KeyboardInterrupt:
        pass
    
    robot.stop()
    sock.close()
    logger.info("[MAIN] Connection closed.")

if __name__ == "__main__":
    main()
