#!/usr/bin/env python3
"""Recreate the committed texture: 400 mm black square, 500 mm total plate."""
from pathlib import Path
import cv2

root = Path(__file__).resolve().parents[1]
dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
tag = cv2.aruco.drawMarker(dictionary, 42, 400)
tag = cv2.copyMakeBorder(tag, 50, 50, 50, 50, cv2.BORDER_CONSTANT, value=255)
path = root / 'models/robot_apriltag/materials/textures/tag42.png'
if not cv2.imwrite(str(path), tag):
    raise SystemExit('Failed to write tag texture')
print(path)
