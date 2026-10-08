"""Open an interactive MuJoCo view of one two-bar jumper and its jump motion."""

from __future__ import annotations

import argparse
from pathlib import Path
import time

import mujoco as mj
import mujoco.viewer
import numpy as np

from model import Design, MOTORS, actuator_for, write_xml
from simulate import vertical_ik

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
    parser.add_argument("--crouch-fraction", type=float, default=0.65)
    parser.add_argument("--thrust-time", type=float, default=0.12)
    parser.add_argument("--hip-kp", type=float, default=300.0)
    parser.add_argument("--knee-kp", type=float, default=300.0)
    parser.add_argument("--duration", type=float, default=1.5)
    parser.add_argument("--xml", type=Path, default=HERE / "generated" / "preview.xml")
    args = parser.parse_args()

    design = Design(
        args.thigh, args.shank, args.payload,
        actuator_for(args.hip_motor, args.hip_ratio),
        actuator_for(args.knee_motor, args.knee_ratio),
        args.crouch_fraction, args.thrust_time, args.hip_kp, args.knee_kp,
    )
    write_xml(design, args.xml)
    model = mj.MjModel.from_xml_path(str(args.xml))
    data = mj.MjData(model)
    hip = model.joint("hip")
    knee = model.joint("knee")
    hip_dof, knee_dof = hip.dofadr[0], knee.dofadr[0]
    hip_motor, knee_motor = model.actuator("hip_motor").id, model.actuator("knee_motor").id

    reach = design.thigh + design.shank
    stand = 0.97 * reach
    crouch = max(abs(design.thigh - design.shank) + 0.01, design.crouch_fraction * reach)
    stand_q = vertical_ik(design.thigh, design.shank, stand)
    crouch_q = vertical_ik(design.thigh, design.shank, crouch)
    data.qpos[hip.qposadr[0]], data.qpos[knee.qposadr[0]] = stand_q
    mj.mj_forward(model, data)

    settle_end, crouch_end = 0.12, 0.38
    thrust_end = crouch_end + design.thrust_time
    kd_hip, kd_knee = 0.08 * np.sqrt(design.hip_kp), 0.08 * np.sqrt(design.knee_kp)
    print("Viewer controls: drag to orbit, scroll to zoom, and close the window to exit.")
    print(f"Generated model: {args.xml}")

    with mujoco.viewer.launch_passive(model, data) as viewer:
        wall_start = time.perf_counter()
        while viewer.is_running() and data.time < args.duration:
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
                design.hip_kp * (target[0] - data.qpos[hip.qposadr[0]]) - kd_hip * data.qvel[hip_dof],
                -design.hip.continuous_output_torque, design.hip.continuous_output_torque,
            )
            knee_tau = np.clip(
                design.knee_kp * (target[1] - data.qpos[knee.qposadr[0]]) - kd_knee * data.qvel[knee_dof],
                -design.knee.continuous_output_torque, design.knee.continuous_output_torque,
            )
            data.ctrl[hip_motor] = design.hip.efficiency * hip_tau
            data.ctrl[knee_motor] = design.knee.efficiency * knee_tau
            mj.mj_step(model, data)
            viewer.sync()
            # Keep the rendered animation close to real time without slowing
            # physics by more than one 1 ms simulation step.
            deadline = wall_start + data.time
            time.sleep(max(0.0, deadline - time.perf_counter()))


if __name__ == "__main__":
    main()
