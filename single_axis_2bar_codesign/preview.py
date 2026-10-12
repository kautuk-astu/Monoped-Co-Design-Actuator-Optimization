"""Open an interactive MuJoCo view of one two-bar jumper and its jump motion."""

from __future__ import annotations

import argparse
from pathlib import Path
import time

import mujoco as mj
import mujoco.viewer
import numpy as np

from model import Design, MOTORS, actuator_for, write_xml
from simulate import SAFETY_MARGIN_M, geometry_clearances, vmc_joint_torques

HERE = Path(__file__).resolve().parent


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--thigh", type=float, default=0.25)
    parser.add_argument("--shank", type=float, default=0.25)
    parser.add_argument("--payload", type=float, default=1.0)
    parser.add_argument("--hip-motor", choices=MOTORS, default="U8")
    parser.add_argument("--knee-motor", choices=MOTORS, default="U8")
    parser.add_argument("--hip-ratio", type=float, default=10.0)
    parser.add_argument("--knee-ratio", type=float, default=10.0)
    parser.add_argument("--theta1", type=float, default=-0.25)
    parser.add_argument("--theta2", type=float, default=0.50)
    parser.add_argument("--ori-l", type=float, default=0.45)
    parser.add_argument("--ori-theta", type=float, default=0.0)
    parser.add_argument("--K", type=float, default=400.0)
    parser.add_argument("--C", type=float, default=4.0)
    parser.add_argument("--T-gain", type=float, default=20.0)
    parser.add_argument("--thrust-time", type=float, default=0.12)
    parser.add_argument("--duration", type=float, default=1.5)
    parser.add_argument("--xml", type=Path, default=HERE / "generated" / "preview.xml")
    args = parser.parse_args()

    design = Design(
        thigh=args.thigh, shank=args.shank, payload_mass=args.payload,
        hip=actuator_for(args.hip_motor, args.hip_ratio), knee=actuator_for(args.knee_motor, args.knee_ratio),
        theta1=args.theta1, theta2=args.theta2, ori_l=args.ori_l, ori_theta=args.ori_theta,
        K=args.K, C=args.C, T_gain=args.T_gain, thrust_time=args.thrust_time,
    )
    write_xml(design, args.xml)
    model = mj.MjModel.from_xml_path(str(args.xml))
    data = mj.MjData(model)
    hip = model.joint("hip")
    knee = model.joint("knee")
    hip_dof, knee_dof = hip.dofadr[0], knee.dofadr[0]
    hip_motor, knee_motor = model.actuator("hip_motor").id, model.actuator("knee_motor").id

    data.qpos[hip.qposadr[0]], data.qpos[knee.qposadr[0]] = design.theta1, design.theta2
    mj.mj_forward(model, data)
    initial_hip_clearance, initial_pole_clearance, initial_link_clearance = geometry_clearances(model, data)
    if min(initial_hip_clearance, initial_pole_clearance, initial_link_clearance) <= SAFETY_MARGIN_M:
        raise ValueError("initial pose violates hip/toe or toe/pole safety clearance")

    print("Viewer controls: drag to orbit, scroll to zoom, and close the window to exit.")
    print(f"Generated model: {args.xml}")

    with mujoco.viewer.launch_passive(model, data) as viewer:
        wall_start = time.perf_counter()
        while viewer.is_running() and data.time < args.duration:
            raw_torque = (
                vmc_joint_torques(design, data.qpos[hip.qposadr[0]], data.qpos[knee.qposadr[0]], data.qvel[hip_dof], data.qvel[knee_dof])
                if data.time < design.thrust_time else np.zeros(2)
            )
            hip_tau = np.clip(raw_torque[0], -design.hip.continuous_output_torque, design.hip.continuous_output_torque)
            knee_tau = np.clip(raw_torque[1], -design.knee.continuous_output_torque, design.knee.continuous_output_torque)
            data.ctrl[hip_motor] = design.hip.efficiency * hip_tau
            data.ctrl[knee_motor] = design.knee.efficiency * knee_tau
            mj.mj_step(model, data)
            hip_clearance, pole_clearance, link_clearance = geometry_clearances(model, data)
            if min(hip_clearance, pole_clearance, link_clearance) <= SAFETY_MARGIN_M:
                print("Stopped: hip/toe or toe/pole safety clearance violated.")
                break
            viewer.sync()
            # Keep the rendered animation close to real time without slowing
            # physics by more than one 1 ms simulation step.
            deadline = wall_start + data.time
            time.sleep(max(0.0, deadline - time.perf_counter()))


if __name__ == "__main__":
    main()
