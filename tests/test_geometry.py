import unittest
import cv2
import numpy as np
from lar_gazebo.geometry import estimate, estimate_on_plane, wrap


class GeometryTest(unittest.TestCase):
    def test_project_recover_and_offset(self):
        K = np.array([[950., 0, 800], [0, 950, 600], [0, 0, 1]])
        wc = np.array([[0., -1, 0, 4.65], [-1, 0, 0, 3], [0, 0, -1, 9], [0, 0, 0, 1]])
        bt = np.eye(4); bt[2, 3] = .55
        for yaw in [0, .7, -2., 3.13]:
            wb = np.eye(4)
            c, s = np.cos(yaw), np.sin(yaw)
            wb[:2, :2] = [[c, -s], [s, c]]
            wb[:3, 3] = [5.2, 3.4, .192]
            ct = np.linalg.inv(wc) @ wb @ bt
            points = np.array([[-.2, .2, 0], [.2, .2, 0], [.2, -.2, 0], [-.2, -.2, 0]])
            corners, _ = cv2.projectPoints(points, cv2.Rodrigues(ct[:3, :3])[0], ct[:3, 3], K, np.zeros(5))
            actual, residual = estimate(corners, K, np.zeros(5), .4, wc, bt)
            np.testing.assert_allclose(actual, wb, atol=1e-5)
            self.assertLess(residual, 1e-5)
            planar = estimate_on_plane(corners, K, np.zeros(5), wc, bt, .742)
            np.testing.assert_allclose(planar, wb, atol=1e-8)

    def test_tag_identity(self):
        dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
        root = __import__('pathlib').Path(__file__).resolve().parents[1]
        image = cv2.imread(str(root/'models/robot_apriltag/materials/textures/tag42.png'), 0)
        self.assertIsNotNone(image)
        _, ids, _ = cv2.aruco.detectMarkers(image, dictionary)
        self.assertEqual(ids.ravel().tolist(), [42])

    def test_wrap_boundary(self):
        self.assertAlmostEqual(wrap(np.deg2rad(179 - (-179))), np.deg2rad(-2))


if __name__ == '__main__':
    unittest.main()
