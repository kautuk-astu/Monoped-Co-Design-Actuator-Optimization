"""Deterministic checks that do not require MuJoCo or CMA-ES to be installed."""

from __future__ import annotations

import math
import sys
from pathlib import Path
import unittest

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from kinematics import (  # noqa: E402
    capsule_clearance, positive_work_increment, reachable_length_bounds,
    saturated_applied_torque, sphere_clearance, thrust_is_active, toe_relative_position, two_link_jacobian,
    virtual_leg_state, vmc_torques,
)


class TwoLinkKinematicsTests(unittest.TestCase):
    def test_analytical_jacobian_matches_finite_difference(self) -> None:
        q = np.array([-0.31, 0.82])
        lengths = (0.27, 0.22)
        analytic = two_link_jacobian(*q, *lengths)
        h = 1e-7
        numeric = np.column_stack([
            (toe_relative_position(*(q + np.eye(2)[i] * h), *lengths)
             - toe_relative_position(*(q - np.eye(2)[i] * h), *lengths)) / (2.0 * h)
            for i in range(2)
        ])
        np.testing.assert_allclose(analytic, numeric, atol=1e-7)

    def test_jacobian_transpose_preserves_virtual_work_sign(self) -> None:
        jacobian = two_link_jacobian(-0.31, 0.82, 0.27, 0.22)
        force = np.array([31.0, -47.0])
        qdot = np.array([1.4, -0.6])
        tau = jacobian.T @ force
        self.assertAlmostEqual(float(tau @ qdot), float(force @ (jacobian @ qdot)), places=12)

    def test_vmc_has_zero_command_at_its_radial_and_orientation_targets(self) -> None:
        q1, q2, l1, l2 = -0.31, 0.82, 0.27, 0.22
        _, _, length, _, theta = virtual_leg_state(q1, q2, 0.0, 0.0, l1, l2)
        torque = vmc_torques(q1, q2, 0.0, 0.0, l1, l2, length, theta, 400.0, 4.0, 20.0)
        np.testing.assert_allclose(torque, np.zeros(2), atol=1e-12)

    def test_independent_angles_change_fk_independently(self) -> None:
        base = toe_relative_position(-0.25, 0.50, 0.25, 0.25)
        hip_changed = toe_relative_position(-0.10, 0.50, 0.25, 0.25)
        knee_changed = toe_relative_position(-0.25, 0.75, 0.25, 0.25)
        self.assertFalse(np.allclose(base, hip_changed))
        self.assertFalse(np.allclose(base, knee_changed))

    def test_reachability_and_collision_clearances(self) -> None:
        lower, upper = reachable_length_bounds(0.25, 0.20, 0.005)
        self.assertAlmostEqual(lower, 0.055)
        self.assertAlmostEqual(upper, 0.445)
        self.assertLess(sphere_clearance(np.zeros(3), 0.06, np.array([0.08, 0.0, 0.0]), 0.025), 0.0)
        self.assertGreater(sphere_clearance(np.zeros(3), 0.06, np.array([0.20, 0.0, 0.0]), 0.025), 0.004)
        pole_start, pole_end = np.array([-0.14, 0.0, 0.0]), np.array([-0.14, 0.0, 2.5])
        self.assertLess(capsule_clearance(np.array([-0.14, 0.0, 0.5]), 0.025, pole_start, pole_end, 0.03), 0.0)
        self.assertGreater(capsule_clearance(np.array([0.10, 0.0, 0.5]), 0.025, pole_start, pole_end, 0.03), 0.004)

    def test_saturation_and_positive_work(self) -> None:
        self.assertAlmostEqual(saturated_applied_torque(20.0, 10.0, 0.9), 9.0)
        self.assertAlmostEqual(saturated_applied_torque(-20.0, 10.0, 0.9), -9.0)
        self.assertAlmostEqual(positive_work_increment(np.array([2.0, 3.0]), np.array([4.0, -1.0]), 0.01), 0.05)
        self.assertEqual(positive_work_increment(np.array([2.0, 3.0]), np.array([-4.0, -1.0]), 0.01), 0.0)

    def test_thrust_duration_gates_command_phase(self) -> None:
        self.assertTrue(thrust_is_active(0.10, 0.12))
        self.assertFalse(thrust_is_active(0.12, 0.12))
        self.assertTrue(thrust_is_active(0.15, 0.20))


if __name__ == "__main__":
    unittest.main()
