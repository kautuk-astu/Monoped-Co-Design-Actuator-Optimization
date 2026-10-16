"""Render the saved best 2R design from several reproducible camera angles."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import subprocess
import sys

VIEWS = ("front", "left_oblique", "right_oblique")


def write_collage(video_paths: list[Path], output_path: Path) -> None:
    """Write a horizontal three-panel video from equal-duration view videos."""
    import imageio.v2 as imageio
    import numpy as np

    readers = [imageio.get_reader(path) for path in video_paths]
    writer = None
    try:
        fps = readers[0].get_meta_data().get("fps", 60)
        writer = imageio.get_writer(output_path, fps=fps, macro_block_size=1)
        while True:
            try:
                frames = [reader.get_next_data() for reader in readers]
            except (IndexError, RuntimeError, StopIteration):
                break
            max_height = max(frame.shape[0] for frame in frames)
            max_width = max(frame.shape[1] for frame in frames)
            panels = []
            for frame in frames:
                panel = np.zeros((max_height, max_width, 3), dtype=frame.dtype)
                panel[:frame.shape[0], :frame.shape[1], :frame.shape[2]] = frame[:, :, :3]
                panels.append(panel)
            writer.append_data(np.concatenate(panels, axis=1))
    finally:
        if writer is not None:
            writer.close()
        for reader in readers:
            reader.close()


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--best-json", required=True, type=Path)
    parser.add_argument("--renderer", required=True, type=Path)
    parser.add_argument("--output-dir", required=True, type=Path)
    parser.add_argument("--duration", required=True, type=float)
    parser.add_argument("--single-jump", action="store_true")
    args = parser.parse_args()

    best = json.loads(args.best_json.read_text())
    if not best.get("valid", False):
        raise SystemExit("No valid best candidate; skipping best-design videos.")

    common = [
        sys.executable, str(args.renderer), "--xml", str(args.output_dir / "best_design.xml"),
        "--thigh", str(best["l1"]), "--shank", str(best["l2"]), "--payload", str(best["payload_mass"]),
        "--theta1", str(best["theta1_rad"]), "--theta2", str(best["theta2_rad"]),
        "--hip-motor", str(best["hip_motor"]), "--knee-motor", str(best["knee_motor"]),
        "--hip-ratio", str(best["hip_gear_ratio"]), "--knee-ratio", str(best["knee_gear_ratio"]),
        "--ori-l", str(best["ori_l"]), "--ori-theta", str(best["ori_theta_rad"]),
        "--K", str(best["K"]), "--C", str(best["C"]), "--T-gain", str(best["T_gain"]),
        "--thrust-time", str(best["thrust_time"]), "--duration", str(args.duration),
        "--authors-single-jump" if args.single_jump else "--authors-contact-vmc",
    ]
    videos = []
    for view in VIEWS:
        output = args.output_dir / f"best_jump_{view}.mp4"
        command = [*common, "--output", str(output), "--view", view]
        print("Rendering", view, "view:", " ".join(command))
        subprocess.run(command, check=True)
        videos.append(output)

    collage = args.output_dir / "best_jump_collage.mp4"
    print("Composing three-panel collage:", collage)
    write_collage(videos, collage)


if __name__ == "__main__":
    main()
