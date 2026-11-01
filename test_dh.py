import unittest
import numpy as np

from dh_web import dh_matrix, forward_kinematics, PI


class TestDHKinematics(unittest.TestCase):
    def test_zero_transform_is_identity(self):
        self.assertTrue(np.allclose(dh_matrix(0, 0, 0, 0), np.eye(4), atol=1e-12))

    def test_quarter_turn_about_z(self):
        expected = np.array([
            [0, -1, 0, 0],
            [1,  0, 0, 0],
            [0,  0, 1, 0],
            [0,  0, 0, 1],
        ], dtype=float)
        self.assertTrue(np.allclose(dh_matrix(PI / 2, 0, 0, 0), expected, atol=1e-12))

    def test_prismatic_unit_translation(self):
        expected = np.eye(4)
        expected[2, 3] = 1.0
        self.assertTrue(np.allclose(dh_matrix(0, 1, 0, 0), expected, atol=1e-12))

    def test_two_link_planar_reference(self):
        params = [
            dict(theta=0.0, d=0.0, r=1.0, alpha=0.0),
            dict(theta=0.0, d=0.0, r=1.0, alpha=0.0),
        ]
        _, T = forward_kinematics(params)
        expected = np.eye(4)
        expected[0, 3] = 2.0
        self.assertTrue(np.allclose(T, expected, atol=1e-12))


if __name__ == "__main__":
    unittest.main()
