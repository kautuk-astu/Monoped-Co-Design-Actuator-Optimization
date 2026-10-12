"""Open a static MuJoCo window for inspecting the vertical two-bar robot."""

from __future__ import annotations

import argparse
import os
from pathlib import Path
import time

# This is an interactive GLFW window, not an offscreen EGL render.  Preserve a
# user-selected backend but select GLFW when no backend was supplied.
os.environ.setdefault("MUJOCO_GL", "glfw")

import mujoco as mj
import mujoco.viewer

from model import Design, MOTORS, actuator_for, write_xml

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
    parser.add_argument("--xml", type=Path, default=HERE / "generated" / "static_view.xml")
    args = parser.parse_args()

    design = Design(
        thigh=args.thigh, shank=args.shank, payload_mass=args.payload,
        hip=actuator_for(args.hip_motor, args.hip_ratio),
        knee=actuator_for(args.knee_motor, args.knee_ratio),
        theta1=args.theta1, theta2=args.theta2, ori_l=0.45, ori_theta=0.0,
        K=400.0, C=4.0, T_gain=20.0, thrust_time=0.12,
    )
    write_xml(design, args.xml)
    model = mj.MjModel.from_xml_path(str(args.xml))
    data = mj.MjData(model)

    # Inspect the explicitly supplied independent initial angles.
    data.qpos[model.joint("hip").qposadr[0]] = design.theta1
    data.qpos[model.joint("knee").qposadr[0]] = design.theta2
    mj.mj_forward(model, data)

    print(f"Generated model: {args.xml}")
    print("Static view: drag to orbit, scroll to zoom, then close the window to exit.")
    with mujoco.viewer.launch_passive(model, data) as viewer:
        while viewer.is_running():
            viewer.sync()
            time.sleep(0.02)


if __name__ == "__main__":
    main()
