"""Essential kinematics and deterministic aligned-jump controller tests."""
from __future__ import annotations

import sys
import tempfile
from pathlib import Path
import unittest

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from kinematics import (  # noqa: E402
    cartesian_vmc_torques, toe_relative_position, two_link_jacobian, vertical_leg_ik,
)
from model import write_xml  # noqa: E402
from optimize_aligned import PARAMETER_NAMES, design_from_vector  # noqa: E402
from simulate import run_jump  # noqa: E402


class AlignedKinematicsTests(unittest.TestCase):
    def test_vertical_ik_and_unreachable_rejection(self) -> None:
        q = vertical_leg_ik(0.28, 0.20, 0.20)
        np.testing.assert_allclose(toe_relative_position(*q, 0.20, 0.20), [0.0, -0.28], atol=1e-12)
        self.assertGreater(q[1], 0.0)
        with self.assertRaises(ValueError):
            vertical_leg_ik(0.40, 0.20, 0.20)

    def test_analytical_jacobian_matches_finite_difference(self) -> None:
        q = np.array([-0.63, 1.31])
        analytical = two_link_jacobian(*q, 0.20, 0.20)
        epsilon = 1e-7
        numerical = np.column_stack([
            (toe_relative_position(*(q + epsilon * np.eye(2)[index]), 0.20, 0.20)
             - toe_relative_position(*(q - epsilon * np.eye(2)[index]), 0.20, 0.20)) / (2.0 * epsilon)
            for index in range(2)
        ])
        np.testing.assert_allclose(analytical, numerical, rtol=1e-7, atol=1e-9)

    def test_horizontal_feedback_has_correct_sign_and_virtual_work(self) -> None:
        q = np.array([-0.55, 1.35])  # toe is displaced toward -x
        qdot = np.array([0.2, -0.1])
        torque, force = cartesian_vmc_torques(*q, *qdot, 0.20, 0.20, 0.36, 500, 5, 300, 0)
        error = toe_relative_position(*q, 0.20, 0.20)[0]
        self.assertGreater(force[0], 0.0)
        self.assertLess(error, 0.0)
        velocity = two_link_jacobian(*q, 0.20, 0.20) @ qdot
        self.assertAlmostEqual(float(torque @ qdot), float(force @ velocity), places=12)


class AlignedControllerTests(unittest.TestCase):
    def test_fixed_candidate_qualifies_and_respects_applied_torque_limits(self) -> None:
        # This is the optimizer's deterministic center seed.
        physical = np.array([.28, 1.0, 6, 6, 15, 15, .38, 800, 8, .15, 300, 10])
        from optimize_aligned import BOUNDS
        vector = (physical - BOUNDS[:, 0]) / (BOUNDS[:, 1] - BOUNDS[:, 0])
        design = design_from_vector(vector)
        self.assertNotIn("theta1", PARAMETER_NAMES)
        self.assertNotIn("theta2", PARAMETER_NAMES)
        with tempfile.TemporaryDirectory() as temporary:
            xml_path = Path(temporary) / "aligned.xml"
            write_xml(design, xml_path)
            result = run_jump(xml_path, design, duration=0.8, controller_mode="aligned_jump", record=True)
        self.assertIsNone(result.constraint_violation)
        self.assertTrue(result.qualified_flight)
        self.assertGreater(result.ballistic_height, 0.0)
        self.assertTrue(np.isfinite(result.alignment_rms))
        hip_limit = design.hip.efficiency * design.hip.continuous_output_torque
        knee_limit = design.knee.efficiency * design.knee.continuous_output_torque
        self.assertLessEqual(max(abs(sample.applied_hip_torque) for sample in result.telemetry), hip_limit + 1e-12)
        self.assertLessEqual(max(abs(sample.applied_knee_torque) for sample in result.telemetry), knee_limit + 1e-12)
        self.assertLessEqual(result.peak_hip_motor_current, design.hip.max_continuous_current + 1e-10)
        self.assertLessEqual(result.peak_knee_motor_current, design.knee.max_continuous_current + 1e-10)
        hip_speed_limit = design.hip.kv_rpm_per_volt * design.hip.rated_voltage * 2.0 * np.pi / 60.0
        knee_speed_limit = design.knee.kv_rpm_per_volt * design.knee.rated_voltage * 2.0 * np.pi / 60.0
        self.assertLessEqual(result.peak_hip_motor_speed, 1.05 * hip_speed_limit)
        self.assertLessEqual(result.peak_knee_motor_speed, 1.05 * knee_speed_limit)
        self.assertTrue({"PREPARE", "STANCE", "FLIGHT", "LANDING"}.issubset({sample.phase for sample in result.telemetry}))


if __name__ == "__main__":
    unittest.main()
