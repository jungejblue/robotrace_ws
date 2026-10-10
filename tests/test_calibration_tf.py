"""Physical point-transform checks for the shared LiDAR--IMU calibration."""

import importlib.util
import math
from pathlib import Path
import unittest


SCRIPT = (Path(__file__).resolve().parents[1] / "src" /
          "robotrace_localization" / "scripts" / "calibration_tf_node.py")
SPEC = importlib.util.spec_from_file_location("calibration_tf_node", SCRIPT)
CALIBRATION = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(CALIBRATION)
IDENTITY = [1, 0, 0, 0, 1, 0, 0, 0, 1]


def point_in_parent(matrix, point):
    return [sum(matrix[i][j] * point[j] for j in range(3)) + matrix[i][3]
            for i in range(3)]


class CalibrationTfTests(unittest.TestCase):
    def test_cloud_forward_axis_rotates_left_in_imu(self):
        # +90 deg about parent z: a cloud +x point is parent +y.
        # Translation describes cloud origin IN IMU, not the reverse vector.
        rotation = [0, -1, 0, 1, 0, 0, 0, 0, 1]
        matrix = CALIBRATION.calibration_matrix("imu", "cloud", [10, 20, 30],
                                                rotation)
        self.assertEqual(point_in_parent(matrix, [1, 0, 0]), [10, 21, 30])
        self.assertEqual(point_in_parent(matrix, [0, 1, 0]), [9, 20, 30])
        self.assertEqual(point_in_parent(matrix, [0, 0, 0]), [10, 20, 30])

    def test_existing_ouster_mount_origin_is_unchanged(self):
        # Bag edge os_sensor -> os_imu is +[.006253,-.011775,.007645].
        # IMU -> input-cloud edge therefore uses the negative vector for I.
        matrix = CALIBRATION.calibration_matrix(
            "lio_imu", "os_sensor", [-0.006253, 0.011775, -0.007645], IDENTITY)
        imu_origin_in_cloud = [0.006253, -0.011775, 0.007645]
        for coordinate in point_in_parent(matrix, imu_origin_in_cloud):
            self.assertAlmostEqual(coordinate, 0.0, places=12)

    def test_general_rotation_row_major_convention(self):
        # This is a cyclic x->y->z->x rotation, not a symmetric matrix.
        matrix = CALIBRATION.calibration_matrix(
            "imu", "cloud", [0, 0, 0], [0, 0, 1, 1, 0, 0, 0, 1, 0])
        self.assertEqual(point_in_parent(matrix, [2, 3, 5]), [5, 2, 3])

    def test_nonrigid_rotations_are_rejected(self):
        for rotation in ([1, 0, 0, 0, 1, 0, 0, 0, -1],  # reflection
                         [2, 0, 0, 0, 1, 0, 0, 0, 1],  # scaling
                         [1, 0.1, 0, 0, 1, 0, 0, 0, 1]):  # shear
            with self.subTest(rotation=rotation), self.assertRaises(ValueError):
                CALIBRATION.calibration_matrix("imu", "cloud", [0, 0, 0], rotation)

    def test_invalid_numeric_calibration_is_rejected(self):
        for translation in ([0, 0], [math.nan, 0, 0], [0, math.inf, 0],
                            [True, 0, 0], ["0", 0, 0]):
            with self.subTest(translation=translation), self.assertRaises(ValueError):
                CALIBRATION.calibration_matrix("imu", "cloud", translation, IDENTITY)
        with self.assertRaises(ValueError):
            CALIBRATION.calibration_matrix("imu", "cloud", [0, 0, 0],
                                           [math.nan] + IDENTITY[1:])
        with self.assertRaises(ValueError):
            CALIBRATION.calibration_matrix("imu", "cloud", [0, 0, 0], IDENTITY[:8])

    def test_invalid_frame_edges_are_rejected(self):
        for parent, child in (("imu", "imu"), ("", "cloud"), ("/imu", "cloud"),
                              ("imu", "cloud frame"), ("imu", None)):
            with self.subTest(parent=parent, child=child), self.assertRaises(ValueError):
                CALIBRATION.calibration_matrix(parent, child, [0, 0, 0], IDENTITY)

    def test_static_tf_rejects_online_calibration(self):
        with self.assertRaises(ValueError):
            CALIBRATION.calibration_matrix("imu", "cloud", [0, 0, 0], IDENTITY,
                                           extrinsic_estimation=True)


if __name__ == "__main__":
    unittest.main()
