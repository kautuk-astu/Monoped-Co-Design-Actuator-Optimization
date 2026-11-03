"""Replay an aligned-controller CMA result exactly, with optional MP4 output."""
from __future__ import annotations

import os
os.environ.setdefault("MUJOCO_GL", "egl")

import argparse
import json
from pathlib import Path

from optimize_aligned import design_from_record, write_telemetry
from model import write_xml
from simulate import run_jump


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("run_dir", type=Path)
    parser.add_argument("--video", action="store_true")
    parser.add_argument("--fps", type=int, default=60)
    args = parser.parse_args()
    record = json.loads((args.run_dir / "best_design.json").read_text())
    metadata = json.loads((args.run_dir / "metadata.json").read_text())
    design = design_from_record(record)
    xml_path = args.run_dir / "best_design.xml"
    write_xml(design, xml_path)
    duration = float(metadata["simulation_duration_s"])

    writer = renderer = camera = None
    step = 0
    if args.video:
        import imageio.v2 as imageio
        import mujoco as mj
        writer = imageio.get_writer(args.run_dir / "best_jump.mp4", fps=args.fps, macro_block_size=1)
        camera = mj.MjvCamera()
        camera.type = mj.mjtCamera.mjCAMERA_FREE
        camera.lookat[:] = (0.0, 0.0, 0.45)
        camera.distance, camera.azimuth, camera.elevation = 1.25, 90.0, -20.0
        stride = max(1, round(1.0 / (args.fps * 0.001)))

        def capture(model: mj.MjModel, data: mj.MjData) -> None:
            nonlocal renderer, step
            step += 1
            if step % stride:
                return
            if renderer is None:
                renderer = mj.Renderer(model, width=640, height=640)
            renderer.update_scene(data, camera=camera)
            writer.append_data(renderer.render())
    else:
        capture = None

    try:
        result = run_jump(
            xml_path, design, duration=duration, controller_mode="aligned_jump",
            after_step=capture, record=True,
        )
    finally:
        if writer is not None:
            writer.close()
        if renderer is not None:
            renderer.close()
    write_telemetry(args.run_dir, result)
    print(f"Ballistic COM height: {result.ballistic_height:.6f} m")
    print(f"RMS / max alignment: {result.alignment_rms:.6f} / {result.maximum_alignment_error:.6f} m")
    print(f"Qualified flight: {result.qualified_flight}; violation: {result.constraint_violation}")


if __name__ == "__main__":
    main()
