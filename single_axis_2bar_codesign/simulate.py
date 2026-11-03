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
    capsule_clearance, cartesian_vmc_torques, positive_work_increment, reachable_length_bounds,
    saturated_applied_torque, segment_to_segment_distance, sphere_box_clearance, sphere_clearance,
    thrust_is_active, toe_relative_position, two_link_jacobian, vertical_leg_ik, vmc_torques,
)


SAFETY_MARGIN_M = 0.004
REACH_EPSILON_M = 0.005
INFEASIBILITY_COST = 1e6
CONTACT_DEBOUNCE_S = 0.005
MIN_FLIGHT_S = 0.080
MIN_TAKEOFF_VELOCITY_M_S = 0.05
MAX_GROUND_PENETRATION_M = 0.005
HIP_STATE_LIMITS_RAD = (-math.pi / 2.0, math.pi / 2.0)
KNEE_STATE_LIMITS_RAD = (0.03, math.pi - 0.03)


@dataclass(frozen=True)
class TelemetrySample:
    time: float
    hip_x: float
    hip_z: float
    toe_x: float
    toe_z: float
    alignment_error: float
    alignment_velocity: float
    q1: float
    q2: float
    qdot1: float
    qdot2: float
    requested_hip_torque: float
    requested_knee_torque: float
    applied_hip_torque: float
    applied_knee_torque: float
    phase: str
    contact: bool
    com_z: float
    com_vz: float
    takeoff_event: bool
    apex_event: bool
    landing_event: bool
    hip_saturated: bool
    knee_saturated: bool
    hip_motor_current: float
    knee_motor_current: float
    hip_motor_speed: float
    knee_motor_speed: float


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
    landing: bool = False
    landing_time: float | None = None
    qualified_flight: bool = False
    flight_duration: float = 0.0
    takeoff_com_height: float | None = None
    apex_com_height: float | None = None
    ballistic_height: float = 0.0
    crouch_to_apex_height: float = 0.0
    alignment_mse: float = 0.0
    alignment_rms: float = 0.0
    maximum_alignment_error: float = 0.0
    prelanding_alignment_error: float | None = None
    hip_saturation_fraction: float = 0.0
    knee_saturation_fraction: float = 0.0
    peak_hip_motor_current: float = 0.0
    peak_knee_motor_current: float = 0.0
    peak_hip_motor_speed: float = 0.0
    peak_knee_motor_speed: float = 0.0
    telemetry: tuple[TelemetrySample, ...] = ()

    @property
    def valid(self) -> bool:
        return self.constraint_violation is None

    @property
    def total_energy(self) -> float:
        """Mechanical positive work plus the authors' winding-loss proxy."""
        return self.positive_work + self.joule_energy


def aligned_vmc_joint_torques(
    design: Design, q1: float, q2: float, qdot1: float, qdot2: float,
) -> tuple[np.ndarray, np.ndarray]:
    """Cartesian stance controller with explicit horizontal stabilization."""
    return cartesian_vmc_torques(
        q1, q2, qdot1, qdot2, design.thigh, design.shank, design.ori_l,
        design.K, design.C, design.Kx, design.Dx,
    )


def _motor_diagnostics(actuator: object, applied_torque: float, joint_velocity: float) -> tuple[float, float, float]:
    """Return absolute motor current, speed, and rated-voltage no-load speed.

    Current follows the same ``Kt = 60/(2*pi*Kv)`` convention used to create
    the continuous output-torque limit.  This is a diagnostic/feasibility
    check, not a new electrical or battery dynamics model.
    """
    kv = float(actuator.kv_rpm_per_volt)
    kt = 60.0 / (2.0 * math.pi * kv)
    current = abs(applied_torque) / max(float(actuator.efficiency) * float(actuator.ratio) * kt, 1e-12)
    speed = abs(joint_velocity * float(actuator.ratio))
    no_load_speed = kv * float(actuator.rated_voltage) * 2.0 * math.pi / 60.0
    return current, speed, no_load_speed


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
    toe_id = model.site("toe").id
    if float(data.site_xpos[toe_id, 2]) < -MAX_GROUND_PENETRATION_M:
        return "toe-ground penetration exceeds 5 mm", hip_to_toe, toe_to_pole, link_to_body
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


def _mass_weighted_com_z(model: mj.MjModel, data: mj.MjData) -> float:
    masses = np.asarray(model.body_mass[1:], dtype=float)
    return float(masses @ np.asarray(data.xipos[1:, 2], dtype=float) / masses.sum())


def _run_aligned_jump(
    xml_path: Path,
    design: Design,
    duration: float,
    after_step: Callable[[mj.MjModel, mj.MjData], None] | None,
    record: bool,
) -> JumpResult:
    """Run the torque-controlled PREPARE/STANCE/FLIGHT/LANDING controller."""
    reach_violation = _reach_violation(design)
    if reach_violation is not None:
        return _invalid_result(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, float("nan"), float("nan"), float("nan"), reach_violation)

    try:
        crouch_length = float(np.linalg.norm(toe_relative_position(
            design.theta1, design.theta2, design.thigh, design.shank,
        )))
        initial_reference = vertical_leg_ik(crouch_length, design.thigh, design.shank)
        flight_reference = vertical_leg_ik(design.flight_leg_length, design.thigh, design.shank)
        landing_reference = vertical_leg_ik(design.landing_leg_length, design.thigh, design.shank)
    except ValueError as exc:
        return _invalid_result(
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0, float("nan"), float("nan"), float("nan"),
            f"invalid aligned IK reference: {exc}",
        )

    model = mj.MjModel.from_xml_path(str(xml_path))
    data = mj.MjData(model)
    hip_joint, knee_joint = model.joint("hip"), model.joint("knee")
    hip_actuator, knee_actuator = model.actuator("hip_motor").id, model.actuator("knee_motor").id
    hip_qpos, knee_qpos = hip_joint.qposadr[0], knee_joint.qposadr[0]
    hip_dof, knee_dof = hip_joint.dofadr[0], knee_joint.dofadr[0]
    carriage_id, toe_id = model.body("carriage").id, model.site("toe").id
    dt = float(model.opt.timestep)
    debounce_steps = max(1, math.ceil(CONTACT_DEBOUNCE_S / dt))

    # Reset is the only permitted direct pose initialization.  Thereafter all
    # reference tracking is through the two existing motor torques.
    data.qpos[hip_qpos], data.qpos[knee_qpos] = initial_reference
    data.qvel[:] = 0.0
    mj.mj_forward(model, data)
    initial_height = float(data.xpos[carriage_id, 2])
    initial_com = _mass_weighted_com_z(model, data)
    violation, hip_clearance, pole_clearance, link_clearance = _state_violation(model, data)
    if violation is not None:
        return _invalid_result(
            initial_height, 0.0, 0.0, 0.0, 0.0, 0.0,
            hip_clearance, pole_clearance, link_clearance, violation,
        )

    phase = "PREPARE"
    off_steps = on_steps = 0
    loss_start_time: float | None = None
    loss_start_com: float | None = None
    contact_start_time: float | None = None
    takeoff = landing = qualified = False
    takeoff_time = landing_time = None
    takeoff_com = apex_com = None
    apex_carriage = initial_height
    prelanding_error = None
    previous_com = initial_com
    com_vz = 0.0
    work = hip_work = knee_work = joule_energy = 0.0
    peak_power = peak_force = 0.0
    alignment_integral = alignment_duration = 0.0
    max_alignment = 0.0
    hip_saturation_steps = knee_saturation_steps = control_steps = 0
    peak_hip_current = peak_knee_current = peak_hip_speed = peak_knee_speed = 0.0
    min_hip_clearance, min_pole_clearance, min_link_clearance = hip_clearance, pole_clearance, link_clearance
    samples: list[TelemetrySample] = []
    final_violation: str | None = None

    while data.time < duration and not landing:
        q = np.array([float(data.qpos[hip_qpos]), float(data.qpos[knee_qpos])])
        qdot = np.array([float(data.qvel[hip_dof]), float(data.qvel[knee_dof])])
        if not HIP_STATE_LIMITS_RAD[0] <= q[0] <= HIP_STATE_LIMITS_RAD[1]:
            final_violation = "hip state outside aligned-controller feasibility limits"
            break
        if not KNEE_STATE_LIMITS_RAD[0] <= q[1] <= KNEE_STATE_LIMITS_RAD[1]:
            final_violation = "knee state outside aligned-controller feasibility limits"
            break
        hip_position = np.asarray(data.xpos[carriage_id], dtype=float)
        toe_position = np.asarray(data.site_xpos[toe_id], dtype=float)
        relative = toe_relative_position(q[0], q[1], design.thigh, design.shank)
        relative_velocity = two_link_jacobian(q[0], q[1], design.thigh, design.shank) @ qdot
        alignment_error = float(toe_position[0] - hip_position[0])
        alignment_velocity = float(relative_velocity[0])
        # The analytical and MuJoCo measurements must share the same frame.
        if abs(alignment_error - float(relative[0])) > 1e-7:
            final_violation = "MuJoCo/analytical toe-position frame mismatch"
            break

        normal_force = foot_contact_force(model, data)
        in_contact = normal_force > 1e-4
        current_com = _mass_weighted_com_z(model, data)
        com_vz = (current_com - previous_com) / dt if data.time > 0.0 else 0.0
        previous_com = current_com
        takeoff_event = apex_event = landing_event = False

        if not in_contact:
            off_steps += 1
            on_steps = 0
            if off_steps == 1:
                loss_start_time, loss_start_com = float(data.time), current_com
        else:
            on_steps += 1
            off_steps = 0
            loss_start_time = loss_start_com = None

        if phase == "PREPARE" and data.time >= design.prepare_time:
            phase = "STANCE"
        if phase == "STANCE" and off_steps >= debounce_steps and com_vz > MIN_TAKEOFF_VELOCITY_M_S:
            phase = "FLIGHT"
            takeoff = takeoff_event = True
            takeoff_time = loss_start_time if loss_start_time is not None else float(data.time)
            takeoff_com = loss_start_com if loss_start_com is not None else current_com
            apex_com = current_com
            on_steps = 0
        if takeoff and phase in {"FLIGHT", "LANDING"}:
            if apex_com is None or current_com > apex_com:
                apex_com = current_com
            apex_carriage = max(apex_carriage, float(data.xpos[carriage_id, 2]))
            if not in_contact:
                prelanding_error = alignment_error
                contact_start_time = None
            elif on_steps == 1:
                contact_start_time = float(data.time)
            if phase == "FLIGHT" and com_vz <= 0.0 and float(data.time) - float(takeoff_time) >= CONTACT_DEBOUNCE_S:
                phase = "LANDING"
                # The first non-positive COM velocity marks the single apex
                # event; apex_com itself retains the maximum sampled height.
                apex_event = True
            if in_contact and on_steps >= debounce_steps:
                landing = landing_event = True
                landing_time = contact_start_time if contact_start_time is not None else float(data.time)
                qualified = landing_time - float(takeoff_time) >= MIN_FLIGHT_S

        if phase == "PREPARE":
            raw_torque = design.joint_kp * (initial_reference - q) - design.joint_kd * qdot
        elif phase == "STANCE":
            raw_torque, cartesian_force = aligned_vmc_joint_torques(design, q[0], q[1], qdot[0], qdot[1])
            # Thrust has a finite window; horizontal feedback remains active
            # during stance so timing cannot silently become an inactive CMA variable.
            if data.time >= design.prepare_time + design.thrust_time:
                cartesian_force[1] = 0.0
                raw_torque = two_link_jacobian(q[0], q[1], design.thigh, design.shank).T @ cartesian_force
        else:
            reference = flight_reference if phase == "FLIGHT" else landing_reference
            raw_torque = design.joint_kp * (reference - q) - design.joint_kd * qdot

        hip_saturated = abs(float(raw_torque[0])) > design.hip.continuous_output_torque
        knee_saturated = abs(float(raw_torque[1])) > design.knee.continuous_output_torque
        applied = np.array([
            saturated_applied_torque(raw_torque[0], design.hip.continuous_output_torque, design.hip.efficiency),
            saturated_applied_torque(raw_torque[1], design.knee.continuous_output_torque, design.knee.efficiency),
        ])
        hip_current, hip_speed, hip_speed_limit = _motor_diagnostics(design.hip, applied[0], qdot[0])
        knee_current, knee_speed, knee_speed_limit = _motor_diagnostics(design.knee, applied[1], qdot[1])
        peak_hip_current, peak_knee_current = max(peak_hip_current, hip_current), max(peak_knee_current, knee_current)
        peak_hip_speed, peak_knee_speed = max(peak_hip_speed, hip_speed), max(peak_knee_speed, knee_speed)
        actuator_violation = None
        if hip_current > design.hip.max_continuous_current + 1e-8 or knee_current > design.knee.max_continuous_current + 1e-8:
            actuator_violation = "continuous motor-current limit exceeded"
        elif hip_speed > 1.05 * hip_speed_limit or knee_speed > 1.05 * knee_speed_limit:
            actuator_violation = "rated-voltage no-load motor speed exceeded by more than 5%"
        data.ctrl[hip_actuator], data.ctrl[knee_actuator] = applied
        control_steps += 1
        hip_saturation_steps += int(hip_saturated)
        knee_saturation_steps += int(knee_saturated)
        alignment_integral += alignment_error * alignment_error * dt
        alignment_duration += dt
        max_alignment = max(max_alignment, abs(alignment_error))
        peak_force = max(peak_force, normal_force)
        mechanical_power = float(applied @ qdot)
        work += positive_work_increment(applied, qdot, dt)
        hip_work += positive_work_increment(applied[:1], qdot[:1], dt)
        knee_work += positive_work_increment(applied[1:], qdot[1:], dt)
        # Preserve the repository's legacy winding-loss proxy.  It is not a
        # validated battery model and is reported separately from mechanical work.
        joule_energy += 0.0955 * float(applied @ applied) * dt
        peak_power = max(peak_power, abs(mechanical_power))

        if record:
            samples.append(TelemetrySample(
                time=float(data.time), hip_x=float(hip_position[0]), hip_z=float(hip_position[2]),
                toe_x=float(toe_position[0]), toe_z=float(toe_position[2]),
                alignment_error=alignment_error, alignment_velocity=alignment_velocity,
                q1=q[0], q2=q[1], qdot1=qdot[0], qdot2=qdot[1],
                requested_hip_torque=float(raw_torque[0]), requested_knee_torque=float(raw_torque[1]),
                applied_hip_torque=float(applied[0]), applied_knee_torque=float(applied[1]),
                phase=phase, contact=in_contact, com_z=current_com, com_vz=com_vz,
                takeoff_event=takeoff_event, apex_event=apex_event, landing_event=landing_event,
                hip_saturated=hip_saturated, knee_saturated=knee_saturated,
                hip_motor_current=hip_current, knee_motor_current=knee_current,
                hip_motor_speed=hip_speed, knee_motor_speed=knee_speed,
            ))

        if actuator_violation is not None:
            final_violation = actuator_violation
            break

        mj.mj_step(model, data)
        # Refresh site/body transforms and contact forces for the integrated
        # state before measurement, event detection, or rendering.
        mj.mj_forward(model, data)
        if after_step is not None:
            after_step(model, data)
        violation, hip_clearance, pole_clearance, link_clearance = _state_violation(model, data)
        min_hip_clearance = min(min_hip_clearance, hip_clearance)
        min_pole_clearance = min(min_pole_clearance, pole_clearance)
        min_link_clearance = min(min_link_clearance, link_clearance)
        if violation is not None:
            final_violation = violation
            break

    mse = alignment_integral / alignment_duration if alignment_duration > 0.0 else 0.0
    flight_duration = (
        float(landing_time) - float(takeoff_time)
        if landing_time is not None and takeoff_time is not None else 0.0
    )
    ballistic_height = (
        max(0.0, float(apex_com) - float(takeoff_com))
        if apex_com is not None and takeoff_com is not None else 0.0
    )
    return JumpResult(
        apex_gain=max(0.0, apex_carriage - initial_height), apex_height=apex_carriage,
        initial_height=initial_height, takeoff=takeoff, takeoff_time=takeoff_time,
        positive_work=work, hip_positive_work=hip_work, knee_positive_work=knee_work,
        peak_power=peak_power, peak_force=peak_force,
        minimum_hip_to_toe_clearance=min_hip_clearance,
        minimum_toe_to_pole_clearance=min_pole_clearance,
        minimum_link_to_body_clearance=min_link_clearance,
        constraint_violation=final_violation, joule_energy=joule_energy,
        landing=landing, landing_time=landing_time, qualified_flight=qualified,
        flight_duration=flight_duration, takeoff_com_height=takeoff_com,
        apex_com_height=apex_com, ballistic_height=ballistic_height,
        crouch_to_apex_height=max(0.0, float(apex_com or initial_com) - initial_com),
        alignment_mse=mse, alignment_rms=math.sqrt(mse),
        maximum_alignment_error=max_alignment,
        prelanding_alignment_error=prelanding_error,
        hip_saturation_fraction=hip_saturation_steps / max(1, control_steps),
        knee_saturation_fraction=knee_saturation_steps / max(1, control_steps),
        peak_hip_motor_current=peak_hip_current, peak_knee_motor_current=peak_knee_current,
        peak_hip_motor_speed=peak_hip_speed, peak_knee_motor_speed=peak_knee_speed,
        telemetry=tuple(samples),
    )


def run_jump(
    xml_path: Path,
    design: Design,
    duration: float = 1.5,
    controller_mode: str = "timed_vmc",
    after_step: Callable[[mj.MjModel, mj.MjData], None] | None = None,
    record: bool = False,
) -> JumpResult:
    """Simulate one candidate, stopping immediately on a feasibility violation."""
    if controller_mode not in {"timed_vmc", "authors_contact_vmc", "authors_single_jump", "aligned_jump"}:
        raise ValueError(f"unknown controller mode: {controller_mode}")
    if controller_mode == "aligned_jump":
        return _run_aligned_jump(xml_path, design, duration, after_step, record)
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
