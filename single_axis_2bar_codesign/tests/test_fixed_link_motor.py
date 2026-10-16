"""Regression tests for the fixed-hardware 2R co-design configuration."""

from __future__ import annotations

import sys
from pathlib import Path
import unittest

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from optimize_fixed_link_motor import (  # noqa: E402
    AUTHOR_ENERGY_WEIGHT,
    AUTHOR_HEIGHT_WEIGHT,
    FIXED_HIP_MOTOR,
    FIXED_KNEE_MOTOR,
    FIXED_L1_M,
    FIXED_L2_M,
    PARAMETER_NAMES,
    authors_objective,
    design_from_vector,
)


class FixedLinkMotorConfigurationTests(unittest.TestCase):
    def test_hardware_is_fixed_and_uses_sspg(self) -> None:
        self.assertEqual((FIXED_L1_M, FIXED_L2_M), (0.20, 0.20))
        self.assertEqual((FIXED_HIP_MOTOR, FIXED_KNEE_MOTOR), ("MAD_M6C12", "MAD_M6C12"))

        design = design_from_vector(np.full(len(PARAMETER_NAMES), 0.5))

        self.assertEqual((design.thigh, design.shank), (0.20, 0.20))
        self.assertEqual((design.hip.motor, design.knee.motor), ("MAD_M6C12", "MAD_M6C12"))
        self.assertEqual((design.hip.gearbox, design.knee.gearbox), ("SSPG", "SSPG"))

    def test_authors_objective_uses_height_and_total_energy(self) -> None:
        class Result:
            apex_height = 0.5
            total_energy = 2.0

        cost, height_cost, total_energy = authors_objective(Result())
        self.assertAlmostEqual(height_cost, 100.0 * np.exp(-1.0))
        self.assertEqual(total_energy, 2.0)
        self.assertAlmostEqual(cost, AUTHOR_HEIGHT_WEIGHT * height_cost + AUTHOR_ENERGY_WEIGHT * total_energy)


if __name__ == "__main__":
    unittest.main()
