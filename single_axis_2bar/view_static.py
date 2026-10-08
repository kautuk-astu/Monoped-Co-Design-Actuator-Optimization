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
    parser.add_argument("--xml", type=Path, default=HERE / "generated" / "static_view.xml")
    args = parser.parse_args()

    design = Design(
        thigh=args.thigh, shank=args.shank, payload_mass=args.payload,
        hip=actuator_for(args.hip_motor, args.hip_ratio),
        knee=actuator_for(args.knee_motor, args.knee_ratio),
        # These controller values are unused by this static inspector.
        crouch_fraction=0.65, thrust_time=0.12, hip_kp=300.0, knee_kp=300.0,
    )
    write_xml(design, args.xml)
    model = mj.MjModel.from_xml_path(str(args.xml))
    data = mj.MjData(model)

    # The XML has a neutral joint configuration. Set a standing IK pose so
    # the foot rests on the floor rather than intersecting it in the viewer.
    standing_length = 0.97 * (design.thigh + design.shank)
    hip_angle, knee_angle = vertical_ik(design.thigh, design.shank, standing_length)
    data.qpos[model.joint("hip").qposadr[0]] = hip_angle
    data.qpos[model.joint("knee").qposadr[0]] = knee_angle
    mj.mj_forward(model, data)

    print(f"Generated model: {args.xml}")
    print("Static view: drag to orbit, scroll to zoom, then close the window to exit.")
    with mujoco.viewer.launch_passive(model, data) as viewer:
        while viewer.is_running():
            viewer.sync()
            time.sleep(0.02)


if __name__ == "__main__":
    main()
