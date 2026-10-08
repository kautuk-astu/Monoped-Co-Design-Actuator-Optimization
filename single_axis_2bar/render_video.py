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
    parser.add_argument("--fps", type=int, default=60)
    parser.add_argument("--width", type=int, default=640)
    parser.add_argument("--height", type=int, default=640)
    parser.add_argument("--output", type=Path, default=HERE / "results" / "preview.mp4")
    args = parser.parse_args()
    if args.fps <= 0 or args.width <= 0 or args.height <= 0:
        parser.error("fps, width, and height must be positive")

    design = Design(
        args.thigh, args.shank, args.payload,
        actuator_for(args.hip_motor, args.hip_ratio),
        actuator_for(args.knee_motor, args.knee_ratio),
        args.crouch_fraction, args.thrust_time, args.hip_kp, args.knee_kp,
    )
    xml_path = HERE / "generated" / "render.xml"
    write_xml(design, xml_path)
    args.output.parent.mkdir(parents=True, exist_ok=True)

    # The MuJoCo model timestep is 1 ms. Sampling at this stride yields the
    # requested video frame rate without storing all rendered frames in RAM.
    frame_stride = max(1, round(1.0 / (args.fps * 0.001)))
    step_count = 0
    writer = imageio.get_writer(args.output, fps=args.fps, macro_block_size=1)
    renderer: mj.Renderer | None = None

    def capture(current_model: mj.MjModel, data: mj.MjData) -> None:
        nonlocal step_count, renderer
        step_count += 1
        if step_count % frame_stride:
            return
        if renderer is None:
            renderer = mj.Renderer(current_model, width=args.width, height=args.height)
        renderer.update_scene(data)
        writer.append_data(renderer.render())

    try:
        result = run_jump(xml_path, design, duration=args.duration, after_step=capture)
    finally:
        writer.close()
        if renderer is not None:
            renderer.close()

    print(f"Video: {args.output}")
    print(f"Apex gain: {result.apex_gain:.3f} m; positive work: {result.positive_work:.2f} J")


if __name__ == "__main__":
    main()
