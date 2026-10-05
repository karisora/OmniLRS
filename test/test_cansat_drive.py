"""Run with python -m unittest discover -s test -p 'test_cansat*.py'."""

import math
import unittest

from src.robots.differential_drive import DifferentialDrive


class DifferentialDriveTests(unittest.TestCase):
    def setUp(self):
        self.drive = DifferentialDrive(0.06070626, 0.1686)
        self.joints = {"left": ["left_wheel_joint"], "right": ["right_wheel_joint"]}

    def test_forward_reverse_and_turn(self):
        for velocity in (-0.1, 0.1):
            left, right = self.drive.wheel_speeds(velocity, 0)
            self.assertAlmostEqual(left, velocity / self.drive.wheel_radius)
            self.assertEqual(left, right)
        left, right = self.drive.wheel_speeds(0, 0.5)
        self.assertLess(left, 0)
        self.assertEqual(left, -right)
        self.assertAlmostEqual((right - left) * self.drive.wheel_radius / self.drive.wheel_track, 0.5)

    def test_saturation_preserves_curvature(self):
        left, right = self.drive.wheel_speeds(3, 5)
        self.assertEqual(max(abs(left), abs(right)), 20)
        self.assertAlmostEqual(left / right, (3 - 5 * 0.1686 / 2) / (3 + 5 * 0.1686 / 2))

    def test_watchdog_reset_and_invalid_command_stop(self):
        self.assertTrue(all(v == 0 for v in self.drive.targets(self.joints, now=1).values()))
        self.drive.receive(0.1, 0, now=1)
        self.assertGreater(self.drive.targets(self.joints, now=1.49)["left_wheel_joint"], 0)
        self.assertTrue(all(v == 0 for v in self.drive.targets(self.joints, now=1.5).values()))
        self.drive.receive(0.1, 0, now=2)
        self.drive.reset()
        self.assertTrue(all(v == 0 for v in self.drive.targets(self.joints, now=2).values()))
        self.drive.receive(math.nan, 0, now=3)
        self.assertTrue(all(v == 0 for v in self.drive.targets(self.joints, now=3).values()))

    def test_invalid_geometry(self):
        for radius, track in [(0, 0.1), (0.1, -1), (math.inf, 0.1)]:
            with self.assertRaises(ValueError):
                DifferentialDrive(radius, track)


if __name__ == "__main__":
    unittest.main()
