"""One jump simulation for the vertical-pole two-bar experiment."""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Callable

import mujoco as mj
import numpy as np

from model import Design


@dataclass(frozen=True)
class JumpResult:
    apex_gain: float
    apex_height: float
    takeoff: bool
    positive_work: float
    peak_power: float
    peak_force: float


def vertical_ik(thigh: float, shank: float, length: float) -> tuple[float, float]:
    """One bent-knee solution with toe directly beneath the hip (x = 0)."""
    length = float(np.clip(length, abs(thigh - shank) + 0.002, thigh + shank - 0.002))
    cos_knee = np.clip((length**2 - thigh**2 - shank**2) / (2.0 * thigh * shank), -1.0, 1.0)
    knee = float(np.arccos(cos_knee))
    hip = float(-np.arctan2(shank * np.sin(knee), thigh + shank * np.cos(knee)))
    return hip, knee


def foot_contact_force(model: mj.MjModel, data: mj.MjData) -> float:
    floor = mj.mj_name2id(model, mj.mjtObj.mjOBJ_GEOM, "floor")
    foot = mj.mj_name2id(model, mj.mjtObj.mjOBJ_GEOM, "foot")
    normal_force = 0.0
    for index in range(data.ncon):
        contact = data.contact[index]
        if {contact.geom1, contact.geom2} == {floor, foot}:
            force = np.zeros(6)
            mj.mj_contactForce(model, data, index, force)
            normal_force += abs(float(force[0]))
    return normal_force


def run_jump(
    xml_path: Path,
    design: Design,
    duration: float = 1.5,
    after_step: Callable[[mj.MjModel, mj.MjData], None] | None = None,
) -> JumpResult:
    """Simulate a jump; ``after_step`` supports rendering or telemetry hooks."""
    model = mj.MjModel.from_xml_path(str(xml_path))
    data = mj.MjData(model)
    hip_joint = model.joint("hip")
    knee_joint = model.joint("knee")
    slide_joint = model.joint("pole_slide")
    hip_actuator = model.actuator("hip_motor").id
    knee_actuator = model.actuator("knee_motor").id
    hip_dof = hip_joint.dofadr[0]
    knee_dof = knee_joint.dofadr[0]
    carriage_id = model.body("carriage").id

    reach = design.thigh + design.shank
    stand = 0.97 * reach
    crouch = max(abs(design.thigh - design.shank) + 0.01, design.crouch_fraction * reach)
    stand_q = vertical_ik(design.thigh, design.shank, stand)
    crouch_q = vertical_ik(design.thigh, design.shank, crouch)
    data.qpos[hip_joint.qposadr[0]], data.qpos[knee_joint.qposadr[0]] = stand_q
    mj.mj_forward(model, data)
    # The slide coordinate is displacement from the XML body's initial pos;
    # use world position so reported heights are actual heights above ground.
    initial_height = float(data.xpos[carriage_id, 2])

    settle_end = 0.12
    crouch_end = 0.38
    thrust_end = crouch_end + design.thrust_time
    kd_hip = 0.08 * np.sqrt(design.hip_kp)
    kd_knee = 0.08 * np.sqrt(design.knee_kp)
    takeoff = False
    had_contact = False
    apex = initial_height
    work = 0.0
    peak_power = 0.0
    peak_force = 0.0

    while data.time < duration:
        if data.time < settle_end:
            target = stand_q
        elif data.time < crouch_end:
            blend = (data.time - settle_end) / (crouch_end - settle_end)
            target = tuple((1.0 - blend) * a + blend * b for a, b in zip(stand_q, crouch_q))
        elif data.time < thrust_end:
            blend = (data.time - crouch_end) / design.thrust_time
            target = tuple((1.0 - blend) * a + blend * b for a, b in zip(crouch_q, stand_q))
        else:
            target = stand_q

        hip_tau = np.clip(
            design.hip_kp * (target[0] - data.qpos[hip_joint.qposadr[0]]) - kd_hip * data.qvel[hip_dof],
            -design.hip.continuous_output_torque, design.hip.continuous_output_torque,
        )
        knee_tau = np.clip(
            design.knee_kp * (target[1] - data.qpos[knee_joint.qposadr[0]]) - kd_knee * data.qvel[knee_dof],
            -design.knee.continuous_output_torque, design.knee.continuous_output_torque,
        )
        # Efficiency converts gearbox-output demand to torque applied in MuJoCo.
        applied_hip = design.hip.efficiency * hip_tau
        applied_knee = design.knee.efficiency * knee_tau
        data.ctrl[hip_actuator] = applied_hip
        data.ctrl[knee_actuator] = applied_knee

        normal_force = foot_contact_force(model, data)
        peak_force = max(peak_force, normal_force)
        in_contact = normal_force > 1e-4
        had_contact = had_contact or in_contact
        if had_contact and not in_contact:
            takeoff = True
        if takeoff:
            apex = max(apex, float(data.xpos[carriage_id, 2]))

        mechanical_power = applied_hip * data.qvel[hip_dof] + applied_knee * data.qvel[knee_dof]
        work += max(0.0, mechanical_power) * model.opt.timestep
        peak_power = max(peak_power, abs(mechanical_power))
        mj.mj_step(model, data)
        if after_step is not None:
            after_step(model, data)

    return JumpResult(
        apex_gain=max(0.0, apex - initial_height), apex_height=apex,
        takeoff=takeoff, positive_work=work, peak_power=peak_power, peak_force=peak_force,
    )
