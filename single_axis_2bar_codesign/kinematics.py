"""Dependency-free two-link kinematics used by the VMC and its tests."""

from __future__ import annotations

import math
import numpy as np


MIN_LEG_LENGTH_M = 1e-8


def toe_relative_position(q1: float, q2: float, l1: float, l2: float) -> np.ndarray:
    """Return hip-to-toe position ``[x, z]`` for +y-axis MuJoCo hinges."""
    q12 = q1 + q2
    return np.array([
        -l1 * math.sin(q1) - l2 * math.sin(q12),
        -l1 * math.cos(q1) - l2 * math.cos(q12),
    ])


def two_link_jacobian(q1: float, q2: float, l1: float, l2: float) -> np.ndarray:
    """Analytical Jacobian ``d[toe_x, toe_z]/d[q1, q2]``."""
    q12 = q1 + q2
    return np.array([
        [-l1 * math.cos(q1) - l2 * math.cos(q12), -l2 * math.cos(q12)],
        [l1 * math.sin(q1) + l2 * math.sin(q12), l2 * math.sin(q12)],
    ])


def wrap_to_pi(angle: float) -> float:
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def virtual_leg_state(
    q1: float, q2: float, qdot1: float, qdot2: float, l1: float, l2: float,
) -> tuple[np.ndarray, np.ndarray, float, float, float]:
    """Return ``(p, J, L, L_dot, theta)`` for the hip-to-toe virtual leg."""
    p = toe_relative_position(q1, q2, l1, l2)
    length = float(np.linalg.norm(p))
    if not math.isfinite(length) or length < MIN_LEG_LENGTH_M:
        raise ValueError("virtual-leg length is zero or non-finite")
    jacobian = two_link_jacobian(q1, q2, l1, l2)
    unit_radial = p / length
    length_rate = float(unit_radial @ (jacobian @ np.array([qdot1, qdot2])))
    theta = math.atan2(float(p[0]), float(-p[1]))
    return p, jacobian, length, length_rate, theta


def vmc_torques(
    q1: float, q2: float, qdot1: float, qdot2: float, l1: float, l2: float,
    ori_l: float, ori_theta: float, stiffness: float, damping: float, orientation_gain: float,
) -> np.ndarray:
    """Compute ``J.T @ F`` for the radial-plus-orientation VMC convention."""
    p, jacobian, length, length_rate, theta = virtual_leg_state(q1, q2, qdot1, qdot2, l1, l2)
    radial_unit = p / length
    theta_unit = np.array([-radial_unit[1], radial_unit[0]])
    radial_force = stiffness * (ori_l - length) - damping * length_rate
    orientation_moment = orientation_gain * wrap_to_pi(ori_theta - theta)
    force = radial_force * radial_unit + (orientation_moment / length) * theta_unit
    return jacobian.T @ force


def point_to_segment_distance(point: np.ndarray, start: np.ndarray, end: np.ndarray) -> float:
    """Return Euclidean distance from a point to a finite 3D segment."""
    segment = end - start
    denominator = float(segment @ segment)
    if denominator <= 0.0:
        return float(np.linalg.norm(point - start))
    fraction = float(np.clip((point - start) @ segment / denominator, 0.0, 1.0))
    return float(np.linalg.norm(point - (start + fraction * segment)))


def reachable_length_bounds(l1: float, l2: float, epsilon: float) -> tuple[float, float]:
    """Strict virtual-leg bounds, inset from singular folded/straight poses."""
    return abs(l1 - l2) + epsilon, l1 + l2 - epsilon


def sphere_clearance(center_a: np.ndarray, radius_a: float, center_b: np.ndarray, radius_b: float) -> float:
    """Return signed surface clearance of two spheres."""
    return float(np.linalg.norm(center_a - center_b)) - radius_a - radius_b


def capsule_clearance(
    sphere_center: np.ndarray, sphere_radius: float, capsule_start: np.ndarray,
    capsule_end: np.ndarray, capsule_radius: float,
) -> float:
    """Return signed clearance of a sphere from a capsule's axis segment."""
    return point_to_segment_distance(sphere_center, capsule_start, capsule_end) - sphere_radius - capsule_radius


def sphere_box_clearance(
    sphere_center: np.ndarray, sphere_radius: float, box_center: np.ndarray,
    box_rotation: np.ndarray, box_half_extents: np.ndarray,
) -> float:
    """Return signed clearance of a sphere from an oriented box."""
    local = np.asarray(box_rotation, dtype=float).reshape(3, 3).T @ (sphere_center - box_center)
    outside = np.maximum(np.abs(local) - box_half_extents, 0.0)
    if np.all(np.abs(local) <= box_half_extents):
        return -sphere_radius
    return float(np.linalg.norm(outside)) - sphere_radius


def segment_to_segment_distance(a0: np.ndarray, a1: np.ndarray, b0: np.ndarray, b1: np.ndarray) -> float:
    """Closest distance between two finite 3D line segments."""
    u, v, w = a1 - a0, b1 - b0, a0 - b0
    a, b, c = float(u @ u), float(u @ v), float(v @ v)
    d, e = float(u @ w), float(v @ w)
    denominator = a * c - b * b
    if denominator < 1e-14:
        s, t = 0.0, float(np.clip(e / c, 0.0, 1.0)) if c > 0 else 0.0
    else:
        s = float(np.clip((b * e - c * d) / denominator, 0.0, 1.0))
        t = float(np.clip((a * e - b * d) / denominator, 0.0, 1.0))
        s = float(np.clip((b * t - d) / a, 0.0, 1.0)) if a > 0 else 0.0
    return float(np.linalg.norm(w + s * u - t * v))


def saturated_applied_torque(command: float, continuous_limit: float, efficiency: float) -> float:
    """Apply the experiment's output-torque saturation then gearbox efficiency."""
    return float(efficiency * np.clip(command, -continuous_limit, continuous_limit))


def positive_work_increment(applied_torques: np.ndarray, joint_velocities: np.ndarray, timestep: float) -> float:
    """Return the non-regenerative mechanical-work increment for one step."""
    power = float(np.asarray(applied_torques) @ np.asarray(joint_velocities))
    return max(0.0, power) * timestep


def thrust_is_active(simulation_time: float, thrust_time: float) -> bool:
    """The VMC is commanded only in the interval ``[0, thrust_time)``."""
    return simulation_time < thrust_time
