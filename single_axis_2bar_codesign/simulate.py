"""Simulation and virtual-model control for the vertical-pole two-bar jumper.

Joint coordinates are ``q = (q1, q2)``: q1 is the hip angle about +y and q2
is the relative knee angle about +y.  The hip-to-toe vector is expressed in
the x-z plane as ``p = toe - hip``.  Its vertical-down orientation is zero:
``theta = atan2(p_x, -p_z)``.  No condition is imposed on ``p_x``.
"""

from __future__ import annotations

from dataclasses import dataclass
import math
from pathlib import Path
from typing import Callable

import mujoco as mj
import numpy as np

from model import Design
from kinematics import (
    capsule_clearance, positive_work_increment, reachable_length_bounds, saturated_applied_torque,
    segment_to_segment_distance, sphere_box_clearance, sphere_clearance, thrust_is_active, vmc_torques,
)


SAFETY_MARGIN_M = 0.004
REACH_EPSILON_M = 0.005
INFEASIBILITY_COST = 1e6


@dataclass(frozen=True)
class JumpResult:
    apex_gain: float
    apex_height: float
    initial_height: float
    takeoff: bool
    takeoff_time: float | None
    positive_work: float
    hip_positive_work: float
    knee_positive_work: float
    peak_power: float
    peak_force: float
    minimum_hip_to_toe_clearance: float
    minimum_toe_to_pole_clearance: float
    minimum_link_to_body_clearance: float
    constraint_violation: str | None = None
    joule_energy: float = 0.0

    @property
    def valid(self) -> bool:
        return self.constraint_violation is None

    @property
    def total_energy(self) -> float:
        """Mechanical positive work plus the authors' winding-loss proxy."""
        return self.positive_work + self.joule_energy


def vmc_joint_torques(design: Design, q1: float, q2: float, qdot1: float, qdot2: float) -> np.ndarray:
    """Map the VMC force/orientation command to hip and knee torques.

    ``F_radial > 0`` points from hip to toe and therefore increases leg length.
    The tangential component is ``(T / L) e_theta`` where increasing theta
    rotates the hip-to-toe vector toward +x.  Consequently virtual work gives
    ``tau = J.T @ F`` and ``tau.dot(qdot) = F.dot(pdot)``.
    """
    return vmc_torques(
        q1, q2, qdot1, qdot2, design.thigh, design.shank, design.ori_l,
        design.ori_theta, design.K, design.C, design.T_gain,
    )


def foot_contact_force(model: mj.MjModel, data: mj.MjData) -> float:
    floor = mj.mj_name2id(model, mj.mjtObj.mjOBJ_GEOM, "floor")
    foot = model.geom("foot").id
    normal_force = 0.0
    for index in range(data.ncon):
        contact = data.contact[index]
        if {contact.geom1, contact.geom2} == {floor, foot}:
            force = np.zeros(6)
            mj.mj_contactForce(model, data, index, force)
            normal_force += abs(float(force[0]))
    return normal_force


def _capsule_segment(model: mj.MjModel, data: mj.MjData, geom_name: str) -> tuple[np.ndarray, np.ndarray, float]:
    geom_id = model.geom(geom_name).id
    center = np.asarray(data.geom_xpos[geom_id], dtype=float)
    axis = np.asarray(data.geom_xmat[geom_id], dtype=float).reshape(3, 3)[:, 2]
    half_length = float(model.geom_size[geom_id, 1])
    return center - half_length * axis, center + half_length * axis, float(model.geom_size[geom_id, 0])


def geometry_clearances(model: mj.MjModel, data: mj.MjData) -> tuple[float, float, float]:
    """Return minimum toe/body, toe/rail, and link/rail surface clearances.

    All radii, pose, and capsule length come from the generated MuJoCo model;
    only ``SAFETY_MARGIN_M`` is an experiment-level constraint parameter.
    """
    hip_id = model.geom("hip_motor_cover").id
    knee_motor_id = model.geom("knee_motor_cover").id
    carriage_id = model.geom("carriage_body").id
    foot_id = model.geom("foot").id
    pole_id = model.geom("vertical_pole").id
    hip_center = np.asarray(data.geom_xpos[hip_id], dtype=float)
    knee_motor_center = np.asarray(data.geom_xpos[knee_motor_id], dtype=float)
    foot_center = np.asarray(data.geom_xpos[foot_id], dtype=float)
    hip_radius = float(model.geom_size[hip_id, 0])
    knee_motor_radius = float(model.geom_size[knee_motor_id, 0])
    foot_radius = float(model.geom_size[foot_id, 0])
    pole_radius = float(model.geom_size[pole_id, 0])

    pole_start, pole_end, _ = _capsule_segment(model, data, "vertical_pole")
    carriage_clearance = sphere_box_clearance(
        foot_center, foot_radius, np.asarray(data.geom_xpos[carriage_id], dtype=float),
        np.asarray(data.geom_xmat[carriage_id], dtype=float), np.asarray(model.geom_size[carriage_id], dtype=float),
    )

    hip_to_toe = min(
        sphere_clearance(hip_center, hip_radius, foot_center, foot_radius),
        sphere_clearance(knee_motor_center, knee_motor_radius, foot_center, foot_radius),
        carriage_clearance,
    )
    toe_to_pole = capsule_clearance(foot_center, foot_radius, pole_start, pole_end, pole_radius)
    link_clearances = []
    for link_name in ("thigh_link", "shank_link"):
        link_start, link_end, link_radius = _capsule_segment(model, data, link_name)
        link_clearances.append(segment_to_segment_distance(link_start, link_end, pole_start, pole_end) - link_radius - pole_radius)
    return hip_to_toe, toe_to_pole, min(link_clearances)


def _reach_violation(design: Design) -> str | None:
    lower, upper = reachable_length_bounds(design.thigh, design.shank, REACH_EPSILON_M)
    if not lower <= design.ori_l <= upper:
        return f"ori_l outside reachable interval [{lower:.6f}, {upper:.6f}]"
    return None


def _state_violation(model: mj.MjModel, data: mj.MjData) -> tuple[str | None, float, float, float]:
    if not (np.isfinite(data.qpos).all() and np.isfinite(data.qvel).all() and math.isfinite(data.time)):
        return "non-finite simulator state", float("nan"), float("nan"), float("nan")
    hip_to_toe, toe_to_pole, link_to_body = geometry_clearances(model, data)
    if not (math.isfinite(hip_to_toe) and math.isfinite(toe_to_pole) and math.isfinite(link_to_body)):
        return "non-finite geometric clearance", hip_to_toe, toe_to_pole, link_to_body
    if hip_to_toe <= SAFETY_MARGIN_M:
        return "toe-to-hip/carriage clearance violation", hip_to_toe, toe_to_pole, link_to_body
    if toe_to_pole <= SAFETY_MARGIN_M:
        return "toe-to-rail clearance violation", hip_to_toe, toe_to_pole, link_to_body
    if link_to_body <= SAFETY_MARGIN_M:
        return "link-to-rail clearance violation", hip_to_toe, toe_to_pole, link_to_body
    return None, hip_to_toe, toe_to_pole, link_to_body


def _invalid_result(
    initial_height: float,
    work: float,
    hip_work: float,
    knee_work: float,
    peak_power: float,
    peak_force: float,
    hip_clearance: float,
    pole_clearance: float,
    link_clearance: float,
    violation: str,
) -> JumpResult:
    return JumpResult(
        apex_gain=0.0, apex_height=initial_height, initial_height=initial_height, takeoff=False, takeoff_time=None,
        positive_work=work, hip_positive_work=hip_work, knee_positive_work=knee_work,
        peak_power=peak_power, peak_force=peak_force,
        minimum_hip_to_toe_clearance=hip_clearance,
        minimum_toe_to_pole_clearance=pole_clearance, minimum_link_to_body_clearance=link_clearance,
        constraint_violation=violation,
    )


def run_jump(
    xml_path: Path,
    design: Design,
    duration: float = 1.5,
    controller_mode: str = "timed_vmc",
    after_step: Callable[[mj.MjModel, mj.MjData], None] | None = None,
) -> JumpResult:
    """Simulate one candidate, stopping immediately on a feasibility violation."""
    if controller_mode not in {"timed_vmc", "authors_contact_vmc", "authors_single_jump"}:
        raise ValueError(f"unknown controller mode: {controller_mode}")
    reach_violation = _reach_violation(design) if controller_mode == "timed_vmc" else None
    if reach_violation is not None:
        return _invalid_result(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, float("nan"), float("nan"), float("nan"), reach_violation)

    model = mj.MjModel.from_xml_path(str(xml_path))
    data = mj.MjData(model)
    hip_joint = model.joint("hip")
    knee_joint = model.joint("knee")
    hip_actuator = model.actuator("hip_motor").id
    knee_actuator = model.actuator("knee_motor").id
    hip_dof = hip_joint.dofadr[0]
    knee_dof = knee_joint.dofadr[0]
    carriage_id = model.body("carriage").id

    # These are independent optimized coordinates, not an inverse-kinematic
    # construction and not a vertical-toe constraint.
    data.qpos[hip_joint.qposadr[0]] = design.theta1
    data.qpos[knee_joint.qposadr[0]] = design.theta2
    mj.mj_forward(model, data)
    initial_height = float(data.xpos[carriage_id, 2])
    violation, hip_clearance, pole_clearance, link_clearance = _state_violation(model, data)
    if violation is not None:
        return _invalid_result(
            initial_height, 0.0, 0.0, 0.0, 0.0, 0.0,
            hip_clearance, pole_clearance, link_clearance, violation,
        )

    takeoff = False
    takeoff_time: float | None = None
    single_jump_complete = False
    had_contact = foot_contact_force(model, data) > 1e-4
    apex = initial_height
    work = 0.0
    hip_work = 0.0
    knee_work = 0.0
    joule_energy = 0.0
    peak_power = 0.0
    peak_force = 0.0
    min_hip_clearance = hip_clearance
    min_pole_clearance = pole_clearance
    min_link_clearance = link_clearance

    while data.time < duration:
        normal_force = foot_contact_force(model, data)
        in_contact = normal_force > 1e-4
        if controller_mode == "authors_single_jump" and takeoff and in_contact:
            # The first touchdown after takeoff ends powered motion. Without
            # this latch, contact VMC would re-engage and create another hop.
            single_jump_complete = True

        if controller_mode == "authors_single_jump" and single_jump_complete:
            raw_torque = np.zeros(2)
        elif controller_mode in {"authors_contact_vmc", "authors_single_jump"} and in_contact:
            # Authors' contact controller: virtual radial spring/damper and
            # orientation spring mapped through the two-link Jacobian.
            raw_torque = vmc_joint_torques(
                design,
                float(data.qpos[hip_joint.qposadr[0]]), float(data.qpos[knee_joint.qposadr[0]]),
                float(data.qvel[hip_dof]), float(data.qvel[knee_dof]),
            )
        elif controller_mode in {"authors_contact_vmc", "authors_single_jump"}:
            # Their flight branch is a joint-space PD controller. The 2R
            # model initializes its joints at theta1/theta2, so those are the
            # equivalent of the authors' zero joint coordinates.
            raw_torque = np.array([
                100.0 * (design.theta1 - data.qpos[hip_joint.qposadr[0]]) - 10.0 * data.qvel[hip_dof],
                100.0 * (design.theta2 - data.qpos[knee_joint.qposadr[0]]) - 10.0 * data.qvel[knee_dof],
            ])
        elif thrust_is_active(data.time, design.thrust_time):
            raw_torque = vmc_joint_torques(
                design,
                float(data.qpos[hip_joint.qposadr[0]]), float(data.qpos[knee_joint.qposadr[0]]),
                float(data.qvel[hip_dof]), float(data.qvel[knee_dof]),
            )
        else:
            raw_torque = np.zeros(2)

        applied_hip = saturated_applied_torque(
            raw_torque[0], design.hip.continuous_output_torque, design.hip.efficiency,
        )
        applied_knee = saturated_applied_torque(
            raw_torque[1], design.knee.continuous_output_torque, design.knee.efficiency,
        )
        data.ctrl[hip_actuator] = applied_hip
        data.ctrl[knee_actuator] = applied_knee

        peak_force = max(peak_force, normal_force)
        if had_contact and not in_contact and not takeoff:
            takeoff = True
            takeoff_time = float(data.time)
        had_contact = had_contact or in_contact

        mechanical_power = applied_hip * data.qvel[hip_dof] + applied_knee * data.qvel[knee_dof]
        work += positive_work_increment(
            np.array([applied_hip, applied_knee]), np.array([data.qvel[hip_dof], data.qvel[knee_dof]]), model.opt.timestep,
        )
        hip_work += positive_work_increment(np.array([applied_hip]), np.array([data.qvel[hip_dof]]), model.opt.timestep)
        knee_work += positive_work_increment(np.array([applied_knee]), np.array([data.qvel[knee_dof]]), model.opt.timestep)
        joule_energy += 0.0955 * (applied_hip ** 2 + applied_knee ** 2) * model.opt.timestep
        peak_power = max(peak_power, abs(mechanical_power))
        mj.mj_step(model, data)
        if after_step is not None:
            after_step(model, data)

        violation, hip_clearance, pole_clearance, link_clearance = _state_violation(model, data)
        min_hip_clearance = min(min_hip_clearance, hip_clearance)
        min_pole_clearance = min(min_pole_clearance, pole_clearance)
        min_link_clearance = min(min_link_clearance, link_clearance)
        if violation is not None:
            return _invalid_result(
                initial_height, work, hip_work, knee_work, peak_power, peak_force,
                min_hip_clearance, min_pole_clearance, min_link_clearance, violation,
            )
        if takeoff:
            apex = max(apex, float(data.xpos[carriage_id, 2]))

    return JumpResult(
        apex_gain=max(0.0, apex - initial_height), apex_height=apex, initial_height=initial_height, takeoff=takeoff,
        takeoff_time=takeoff_time, positive_work=work, hip_positive_work=hip_work, knee_positive_work=knee_work,
        peak_power=peak_power, peak_force=peak_force,
        minimum_hip_to_toe_clearance=min_hip_clearance,
        minimum_toe_to_pole_clearance=min_pole_clearance,
        minimum_link_to_body_clearance=min_link_clearance,
        joule_energy=joule_energy,
    )
