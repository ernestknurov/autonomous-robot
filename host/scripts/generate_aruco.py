import cv2
import numpy as np
import os

aruco_dict = cv2.aruco.getPredefinedDictionary(
    cv2.aruco.DICT_4X4_50
)

marker_id = 0
marker_size_px = 400
border_px = 80

marker = cv2.aruco.generateImageMarker(
    aruco_dict,
    marker_id,
    marker_size_px
)

img_size = marker_size_px + 2 * border_px
canvas = np.ones((img_size, img_size), dtype=np.uint8) * 255

canvas[
    border_px:border_px + marker_size_px,
    border_px:border_px + marker_size_px
] = marker

output_path = os.path.expanduser(f"~/Projects/autonomous-robot/aruco_markers/aruco_4x4_50_id{marker_id}_with_border.png")
cv2.imwrite(output_path, canvas)
print(f"Marker ID {marker_id} saved to {output_path}")