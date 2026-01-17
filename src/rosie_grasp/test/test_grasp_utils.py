#!/usr/bin/env python3
import sys
from pathlib import Path
import unittest
import numpy as np

sys.path.insert(0, str(Path(__file__).parents[1] / "scripts"))
from grasp_utils import (depth_to_point_cloud, normalize_quaternion, offset_position,
                         select_candidate)


class GraspUtilsTest(unittest.TestCase):
    def test_depth_projection_and_bounds(self):
        depth = np.ones((2, 2), dtype=float)
        points, mask = depth_to_point_cloud(depth, [[100, 0, 0], [0, 100, 0], [0, 0, 1]], bounds=[-0.001, 0.001, -0.001, 0.001, .5, 1.5])
        self.assertEqual(points.shape[1], 3)
        self.assertEqual(int(mask.sum()), 1)
        np.testing.assert_allclose(points[0], [0, 0, 1])

    def test_quaternion_normalization(self):
        np.testing.assert_allclose(normalize_quaternion([0, 0, 0, 2]), [0, 0, 0, 1])
        with self.assertRaises(ValueError):
            normalize_quaternion([0, 0, 0, 0])

    def test_candidate_selection_rejects_invalid_and_collision(self):
        candidates = [
            {"position": [0, 0, 0], "quaternion": [0, 0, 0, 1], "score": .9, "collision_free": False},
            {"position": [0, 0, 0], "quaternion": [0, 0, 0, 0], "score": .99},
            {"position": [1, 2, 3], "quaternion": [0, 0, 0, 2], "score": .8},
        ]
        selected = select_candidate(candidates, .5)
        self.assertEqual(selected["position"].tolist(), [1, 2, 3])
        np.testing.assert_allclose(selected["quaternion"], [0, 0, 0, 1])

    def test_rotation_changes_offset_direction(self):
        identity = offset_position([0, 0, 0], [0, 0, 0, 1], 1)
        quarter_turn = offset_position([0, 0, 0], [0, np.sqrt(.5), 0, np.sqrt(.5)], 1)
        self.assertFalse(np.allclose(identity, quarter_turn))

    def test_two_model_rotations_remain_distinct(self):
        first = select_candidate([{"position": [0, 0, 1], "quaternion": [0, 0, 0, 1], "score": .8}])
        second = select_candidate([{"position": [0, 0, 1], "quaternion": [0, np.sqrt(.5), 0, np.sqrt(.5)], "score": .8}])
        self.assertFalse(np.allclose(first["quaternion"], second["quaternion"]))


if __name__ == "__main__":
    unittest.main()
