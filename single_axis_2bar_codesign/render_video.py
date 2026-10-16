"""Render a two-bar jump to MP4 without opening an interactive window."""

from __future__ import annotations

# Must be selected before importing MuJoCo. EGL renders on the NVIDIA GPU
# without going through X11, which is more reliable than a remote desktop.
import os
os.environ.setdefault("MUJOCO_GL", "egl")

import argparse
from pathlib import Path

import imageio.v2 as imageio
import mujoco as mj

from model import Design, MOTORS, actuator_for, write_xml
from simulate import run_jump

HERE = Path(__file__).resolve().parent
VIEWS = {
    "front": (90.0, -20.0),
    "left_oblique": (45.0, -20.0),
    "right_oblique": (135.0, -20.0),
}


def make_camera(view: str) -> mj.MjvCamera:
    """Create a repeatable free camera aimed at the one-axis jumper."""
    azimuth, elevation = VIEWS[view]
    camera = mj.MjvCamera()
    camera.type = mj.mjtCamera.mjCAMERA_FREE
    camera.lookat[:] = (0.0, 0.0, 0.45)
    camera.distance = 1.25
    camera.azimuth = azimuth
    camera.elevation = elevation
    return camera


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
    parser.add_argument("--authors-contact-vmc", action="store_true", help="use the authors' contact VMC and flight PD controller")
    parser.add_argument("--authors-single-jump", action="store_true", help="disable actuation after the first post-takeoff touchdown")
    parser.add_argument("--duration", type=float, default=1.5)
    parser.add_argument("--view", choices=VIEWS, default="front")
    parser.add_argument("--fps", type=int, default=60)
    parser.add_argument("--width", type=int, default=640)
    parser.add_argument("--height", type=int, default=640)
    parser.add_argument("--output", type=Path, default=HERE / "results" / "preview.mp4")
    parser.add_argument("--xml", type=Path, default=HERE / "generated" / "render.xml")
    args = parser.parse_args()
    if args.fps <= 0 or args.width <= 0 or args.height <= 0:
        parser.error("fps, width, and height must be positive")

    design = Design(
        thigh=args.thigh, shank=args.shank, payload_mass=args.payload,
        hip=actuator_for(args.hip_motor, args.hip_ratio), knee=actuator_for(args.knee_motor, args.knee_ratio),
        theta1=args.theta1, theta2=args.theta2, ori_l=args.ori_l, ori_theta=args.ori_theta,
        K=args.K, C=args.C, T_gain=args.T_gain, thrust_time=args.thrust_time,
    )
    xml_path = args.xml
    write_xml(design, xml_path)
    args.output.parent.mkdir(parents=True, exist_ok=True)

    # The MuJoCo model timestep is 1 ms. Sampling at this stride yields the
    # requested video frame rate without storing all rendered frames in RAM.
    frame_stride = max(1, round(1.0 / (args.fps * 0.001)))
    step_count = 0
    camera = make_camera(args.view)
    writer = imageio.get_writer(args.output, fps=args.fps, macro_block_size=1)
    renderer: mj.Renderer | None = None

    def capture(current_model: mj.MjModel, data: mj.MjData) -> None:
        nonlocal step_count, renderer
        step_count += 1
        if step_count % frame_stride:
            return
        if renderer is None:
            renderer = mj.Renderer(current_model, width=args.width, height=args.height)
        renderer.update_scene(data, camera=camera)
        writer.append_data(renderer.render())

    try:
        result = run_jump(
            xml_path, design, duration=args.duration,
            controller_mode=(
                "authors_single_jump" if args.authors_single_jump
                else "authors_contact_vmc" if args.authors_contact_vmc
                else "timed_vmc"
            ),
            after_step=capture,
        )
    finally:
        writer.close()
        if renderer is not None:
            renderer.close()

    print(f"Video: {args.output}")
    print(f"Apex gain: {result.apex_gain:.3f} m; positive work: {result.positive_work:.2f} J")


if __name__ == "__main__":
    main()
