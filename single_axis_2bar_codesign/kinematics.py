"""Dependency-free two-link kinematics used by the VMC and its tests."""

from __future__ import annotations

import math
import numpy as np


MIN_LEG_LENGTH_M = 1e-8
DEFAULT_IK_MARGIN_RAD = 0.03


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


def vertical_leg_ik(
    separation: float,
    l1: float,
    l2: float,
    *,
    knee_branch: int = 1,
    singularity_margin: float = 0.005,
    hip_limits: tuple[float, float] = (-math.pi / 2.0, math.pi / 2.0),
    knee_limits: tuple[float, float] = (DEFAULT_IK_MARGIN_RAD, math.pi - DEFAULT_IK_MARGIN_RAD),
) -> np.ndarray:
    """Return a joint pose whose toe is ``separation`` directly below the hip.

    ``knee_branch=1`` is the model's normal crouch (negative hip angle and
    positive relative-knee angle).  The opposite branch is available for
    kinematic checks, but its negative knee angle is outside the default
    physical reference limits.  The workspace is inset at both singularities.
    """
    if knee_branch not in (-1, 1):
        raise ValueError("knee_branch must be -1 or 1")
    if not all(math.isfinite(value) and value > 0.0 for value in (separation, l1, l2)):
        raise ValueError("separation and link lengths must be finite and positive")
    lower, upper = reachable_length_bounds(l1, l2, singularity_margin)
    if not lower <= separation <= upper:
        raise ValueError(f"vertical separation outside reachable interval [{lower:.6f}, {upper:.6f}]")

    # The desired hip-to-toe vector is [0, -separation].  alpha and beta are
    # the two link angles away from that vector from the law of cosines.
    cos_alpha = np.clip((l1 * l1 + separation * separation - l2 * l2) / (2.0 * l1 * separation), -1.0, 1.0)
    cos_beta = np.clip((l2 * l2 + separation * separation - l1 * l1) / (2.0 * l2 * separation), -1.0, 1.0)
    alpha = math.acos(float(cos_alpha))
    beta = math.acos(float(cos_beta))
    q = np.array([-knee_branch * alpha, knee_branch * (alpha + beta)])
    if not hip_limits[0] <= q[0] <= hip_limits[1]:
        raise ValueError(f"IK hip angle {q[0]:.6f} outside limits {hip_limits}")
    branch_knee_limits = knee_limits if knee_branch > 0 else (-knee_limits[1], -knee_limits[0])
    if not branch_knee_limits[0] <= q[1] <= branch_knee_limits[1]:
        raise ValueError(f"IK knee angle {q[1]:.6f} outside limits {branch_knee_limits}")
    return q


def cartesian_vmc_torques(
    q1: float,
    q2: float,
    qdot1: float,
    qdot2: float,
    l1: float,
    l2: float,
    rest_length: float,
    stiffness: float,
    damping: float,
    horizontal_stiffness: float,
    horizontal_damping: float,
) -> tuple[np.ndarray, np.ndarray]:
    """Return ``(J.T @ [Fx, Fz], [Fx, Fz])`` for aligned stance VMC.

    Horizontal error and velocity are the toe values relative to the hip.
    The vertical component retains the existing radial spring/damper command;
    only its world-z component is used so horizontal regulation is explicit.
    """
    p, jacobian, length, length_rate, _ = virtual_leg_state(q1, q2, qdot1, qdot2, l1, l2)
    relative_velocity = jacobian @ np.array([qdot1, qdot2])
    radial_force = stiffness * (rest_length - length) - damping * length_rate
    fx = -horizontal_stiffness * p[0] - horizontal_damping * relative_velocity[0]
    fz = radial_force * p[1] / length
    force = np.array([fx, fz])
    return jacobian.T @ force, force


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
