"""CMA-vector mapping tests; these do not need the runtime simulator."""

from __future__ import annotations

import sys
from pathlib import Path
import unittest

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from optimize import BOUNDS, PARAMETER_NAMES, denormalize, design_from_vector  # noqa: E402


class CodesignVectorTests(unittest.TestCase):
    def test_vector_has_the_specified_fifteen_entries_and_bounds(self) -> None:
        expected = (
            "l1", "l2", "theta1", "theta2", "payload_mass", "hip_motor_idx",
            "knee_motor_idx", "hip_gear_ratio", "knee_gear_ratio", "ori_l",
            "ori_theta", "K", "C", "T_gain", "thrust_time",
        )
        self.assertEqual(PARAMETER_NAMES, expected)
        self.assertEqual(BOUNDS.shape, (15, 2))
        self.assertTrue(np.allclose(BOUNDS[:, 0], denormalize(np.zeros(15))))
        self.assertTrue(np.allclose(BOUNDS[:, 1], denormalize(np.ones(15))))

    def test_decoded_design_preserves_independent_angles_and_vmc_parameters(self) -> None:
        vector = np.array([0.50, 0.25, 0.20, 0.70, 0.30, 0.00, 1.00, 0.40, 0.60, 0.55, 0.75, 0.80, 0.30, 0.60, 0.50])
        physical = denormalize(vector)
        design = design_from_vector(vector)
        self.assertAlmostEqual(design.thigh, physical[0])
        self.assertAlmostEqual(design.shank, physical[1])
        self.assertAlmostEqual(design.theta1, physical[2])
        self.assertAlmostEqual(design.theta2, physical[3])
        self.assertAlmostEqual(design.ori_l, physical[9])
        self.assertAlmostEqual(design.ori_theta, physical[10])
        self.assertAlmostEqual(design.K, physical[11])
        self.assertAlmostEqual(design.C, physical[12])
        self.assertAlmostEqual(design.T_gain, physical[13])
        self.assertAlmostEqual(design.thrust_time, physical[14])
        self.assertEqual(design.hip.motor, "U8")
        self.assertEqual(design.knee.motor, "MAD_M6C12")

    def test_out_of_range_normalized_vector_is_rejected_not_clipped(self) -> None:
        with self.assertRaises(ValueError):
            denormalize(np.full(15, 1.01))


if __name__ == "__main__":
    unittest.main()
