TIMEOUT_S = 20.0
MAX_MOVE_M = 2.0
MAX_TURN_DEG = 180.0 

ESP32_IP = "192.168.0.183"
ESP32_PORT = 8080
DEFAULT_COMMAND_TIMEOUT_S = 5.0
GET_DISTANCE_TIMEOUT_S = 7.0

# Vision parameters
DEPTH_ESTIMATOR_MODEL_TYPE = "DPT_Hybrid"  # "DPT_Large" for better quality but slower inference
VISION_BASE_URL = "http://192.168.0.139:8080/"
VISION_RESOLUTION = (1920, 1080)
DEPTH_ESTIMATION_BLOCKED_THRESHOLD = 0.45
AREA_OF_INTEREST = (0.15, 0.85, 0.15, 0.85)  # (x0_ratio, x1_ratio, y0_ratio, y1_ratio)
CLOSE_WIRING_MASK = (0.55, 0.40)  # Known camera artifact region in left lower corner: (y_start_ratio, x_end_ratio)