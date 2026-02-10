import cv2
import cv2.aruco as aruco
import numpy as np
import os

# Define dictionary (must match DICT_4X4_50 used in detector)
aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)

# Generate Marker ID 0
# 256x256 pixels (Power of 2 is safer)
img_gray = aruco.drawMarker(aruco_dict, 0, 256)
img = cv2.cvtColor(img_gray, cv2.COLOR_GRAY2BGR)

# Save to file
path = 'models/arucotag/materials/textures/aruco_marker_0.png'
cv2.imwrite(path, img)
print(f"Generated {path}")
