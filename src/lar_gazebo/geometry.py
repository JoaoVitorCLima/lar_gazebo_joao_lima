"""Geometry independent of ROS. Coordinates: world ENU, optical right/down/forward."""
import math
import cv2
import numpy as np


def wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def estimate(corners, K, distortion, size, world_camera, base_tag):
    """T_world_base = T_world_camera T_camera_tag inverse(T_base_tag)."""
    s = size / 2.0
    points = np.array([[-s, s, 0], [s, s, 0], [s, -s, 0], [-s, -s, 0]], np.float64)
    ok, rvec, tvec = cv2.solvePnP(points, np.asarray(corners, np.float64).reshape(4, 2),
                                 K, distortion, flags=cv2.SOLVEPNP_ITERATIVE)
    if not ok or tvec[2, 0] <= 0:
        raise ValueError('Invalid PnP solution')
    projected, _ = cv2.projectPoints(points, rvec, tvec, K, distortion)
    residual = float(np.sqrt(np.mean(np.sum((projected.reshape(4, 2) - corners.reshape(4, 2))**2, axis=1))))
    camera_tag = np.eye(4)
    camera_tag[:3, :3] = cv2.Rodrigues(rvec)[0]
    camera_tag[:3, 3] = tvec.ravel()
    return world_camera @ camera_tag @ np.linalg.inv(base_tag), residual


def estimate_on_plane(corners, K, distortion, world_camera, base_tag, height):
    """Intersect undistorted optical rays with surveyed horizontal tag plane.

    For a ground robot this avoids the depth/tilt ambiguity of a small planar PnP
    target. Height is a fixed mechanical calibration, never simulator telemetry.
    """
    pixels = np.asarray(corners, np.float64).reshape(4, 1, 2)
    normalized = cv2.undistortPoints(pixels, K, distortion).reshape(4, 2)
    rays = np.column_stack((normalized, np.ones(4))) @ world_camera[:3, :3].T
    origin = world_camera[:3, 3]
    if np.any(abs(rays[:, 2]) < 1e-8):
        raise ValueError('Camera ray parallel to tag plane')
    distance = (height-origin[2])/rays[:, 2]
    if np.any(distance <= 0):
        raise ValueError('Plane behind camera')
    points = origin + distance[:, None]*rays
    x_axis = (points[1]-points[0]) + (points[2]-points[3])
    angle = math.atan2(x_axis[1], x_axis[0])
    c, s = math.cos(angle), math.sin(angle)
    world_tag = np.eye(4)
    world_tag[:3, :3] = [[c, -s, 0], [s, c, 0], [0, 0, 1]]
    world_tag[:3, 3] = points.mean(axis=0)
    return world_tag @ np.linalg.inv(base_tag)
