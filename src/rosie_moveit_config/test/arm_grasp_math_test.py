#!/usr/bin/env python3
import sys
from pathlib import Path
import unittest

sys.path.insert(0, str(Path(__file__).parents[1] / "scripts"))
from grasp_math import normalize_quaternion, offset_along_grasp_axis


class ArmGraspMathTest(unittest.TestCase):
    def test_normalizes_quaternion(self):
        self.assertEqual(normalize_quaternion((0, 0, 0, 2)), (0.0, 0.0, 0.0, 1.0))

    def test_learned_orientation_changes_approach(self):
        identity = offset_along_grasp_axis((0, 0, 0), (0, 0, 0, 1), 0.2)
        rotated = offset_along_grasp_axis((0, 0, 0), (0, 2**-0.5, 0, 2**-0.5), 0.2)
        self.assertNotEqual(identity, rotated)


if __name__ == "__main__":
    unittest.main()
