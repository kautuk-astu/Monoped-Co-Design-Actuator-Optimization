"""Replay an aligned-controller CMA result exactly, with optional MP4 output."""
from __future__ import annotations

import os
os.environ.setdefault("MUJOCO_GL", "egl")

import argparse
import json
from pathlib import Path

import numpy as np

from optimize_aligned import design_from_record, write_compact_log, write_telemetry
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
    penalties = metadata["penalties"]
    write_compact_log(
        args.run_dir, record, result, seed=int(metadata["seed"]),
        popsize=int(metadata["popsize"]), maxiter=int(metadata["maxiter"]),
        simulation_duration=duration, lambda_x=float(penalties["lambda_x"]),
        lambda_landing=float(penalties["lambda_landing"]),
        lambda_energy=float(penalties["lambda_energy"]),
        bounds=np.asarray([metadata["bounds"][name] for name in metadata["parameter_names"]], dtype=float),
    )
    maximum_base_height = max(float(sample.hip_z) for sample in result.telemetry)
    print(f"Initial / maximum base height: {result.initial_height:.6f} / {maximum_base_height:.6f} m")
    print(f"Total base rise: {maximum_base_height - result.initial_height:.6f} m")
    print(f"Toe alignment RMS / maximum error: {result.alignment_rms:.6f} / {result.maximum_alignment_error:.6f} m")
    print(f"Valid jump: {result.qualified_flight}; violation: {result.constraint_violation}")


if __name__ == "__main__":
    main()
