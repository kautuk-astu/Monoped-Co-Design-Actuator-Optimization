#!/usr/bin/env python3
"""Plot the best base rise by gearbox across a parallel seed batch."""

from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path
import re


GEARBOX_ORDER = ("SSPG", "CPG", "WPG", "DSPG")
OUTPUT_PATTERN = re.compile(r"^\s*output:\s*(.+?)\s*$", re.MULTILINE)


def result_root_from_log(log_path: Path) -> Path:
    match = OUTPUT_PATTERN.search(log_path.read_text(errors="replace"))
    if match is None:
        raise ValueError(f"No result output path found in {log_path}")
    result_root = Path(match.group(1))
    if not result_root.is_dir():
        raise FileNotFoundError(f"Result directory from {log_path} is missing: {result_root}")
    return result_root


def collect(batch_dir: Path) -> list[dict[str, object]]:
    rows: list[dict[str, object]] = []
    logs = sorted(
        batch_dir.glob("seed_*.log"),
        key=lambda path: int(path.stem.removeprefix("seed_")),
    )
    if not logs:
        raise ValueError(f"No seed_*.log files found in {batch_dir}")

    for log_path in logs:
        seed = int(log_path.stem.removeprefix("seed_"))
        result_root = result_root_from_log(log_path)
        found_for_seed = 0
        for gearbox in GEARBOX_ORDER:
            run_dir = result_root / gearbox.lower()
            compact_path = run_dir / "compact_optimization_log.json"
            best_path = run_dir / "best_design.json"
            if not compact_path.exists() and not best_path.exists():
                continue
            if not compact_path.is_file() or not best_path.is_file():
                raise FileNotFoundError(f"Incomplete {gearbox} result for seed {seed}: {run_dir}")
            found_for_seed += 1
            compact = json.loads(compact_path.read_text())
            best = json.loads(best_path.read_text())
            results = compact.get("key_results", {})
            rows.append({
                "seed": seed,
                "motor_type": best.get("hip_motor", ""),
                "gearbox_type": gearbox,
                "valid_jump": bool(results.get("valid_jump", False)),
                "initial_base_height_m": results.get("initial_base_height_m", ""),
                "maximum_base_height_m": results.get("maximum_base_height_m", ""),
                "best_base_rise_m": results.get("total_base_rise_m", ""),
                "cost": results.get("cost", best.get("cost", "")),
                "hip_actual_ratio": best.get("hip_gear_ratio", ""),
                "knee_actual_ratio": best.get("knee_gear_ratio", ""),
                "run_directory": str(run_dir.resolve()),
            })
        if found_for_seed == 0:
            raise ValueError(f"No gearbox results found for seed {seed}: {result_root}")
    gearboxes = {str(row["gearbox_type"]) for row in rows}
    for seed in {int(row["seed"]) for row in rows}:
        seed_gearboxes = {
            str(row["gearbox_type"]) for row in rows if int(row["seed"]) == seed
        }
        if seed_gearboxes != gearboxes:
            raise ValueError(
                f"Seed {seed} has {sorted(seed_gearboxes)}, expected {sorted(gearboxes)}"
            )
    return rows


def write_csv(batch_dir: Path, rows: list[dict[str, object]]) -> Path:
    output = batch_dir / "best_base_rise_by_seed.csv"
    with output.open("w", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)
    return output


def write_plot(batch_dir: Path, rows: list[dict[str, object]]) -> Path:
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    import numpy as np

    plt.rcParams.update({
        "font.size": 20,
        "axes.titlesize": 28,
        "axes.labelsize": 20,
        "xtick.labelsize": 20,
        "ytick.labelsize": 20,
        "legend.fontsize": 20,
        "legend.title_fontsize": 20,
    })

    seeds = sorted({int(row["seed"]) for row in rows})
    gearboxes = [
        gearbox for gearbox in GEARBOX_ORDER
        if any(row["gearbox_type"] == gearbox for row in rows)
    ]
    positions = np.arange(len(seeds), dtype=float)
    width = 0.82 / len(gearboxes)
    colors = {
        "SSPG": "#3977af", "CPG": "#e1812c",
        "WPG": "#3a923a", "DSPG": "#c44e52",
    }

    figure_width = 14 if len(gearboxes) <= 2 else 22
    figure, axis = plt.subplots(figsize=(figure_width, 8), layout="constrained")
    for index, gearbox in enumerate(gearboxes):
        lookup = {
            int(row["seed"]): row
            for row in rows
            if row["gearbox_type"] == gearbox and row["best_base_rise_m"] != ""
        }
        values = [
            float(lookup[seed]["best_base_rise_m"]) * 100.0
            if seed in lookup else float("nan")
            for seed in seeds
        ]
        labels = [
            (
                f"{value:.2f} cm\n"
                f"Hip {float(lookup[seed]['hip_actual_ratio']):.2f}:1\n"
                f"Knee {float(lookup[seed]['knee_actual_ratio']):.2f}:1"
            )
            if seed in lookup else ""
            for seed, value in zip(seeds, values)
        ]
        offset = (index - (len(gearboxes) - 1) / 2.0) * width
        bars = axis.bar(
            positions + offset, values, width=width, label=gearbox,
            color=colors[gearbox],
        )
        # label_size = 11 if len(gearboxes) <= 2 else 9
        label_size = 12
        axis.bar_label(bars, labels=labels, padding=3, fontsize=label_size)

    motors = sorted({str(row["motor_type"]) for row in rows if row["motor_type"]})
    motor_label = ", ".join(motors) if motors else "fixed motor"
    axis.set_title(f"Best base rise across seeds ({motor_label})")
    axis.set_xlabel("CMA-ES seed")
    axis.set_ylabel("Best base rise [cm]")
    axis.set_xticks(positions, [str(seed) for seed in seeds])
    maximum = max(float(row["best_base_rise_m"]) * 100.0 for row in rows)
    axis.set_ylim(0, maximum * 1.32)
    axis.grid(axis="y", alpha=0.25)
    axis.legend(title="Gearbox type")

    output = batch_dir / "best_base_rise_by_seed.png"
    figure.savefig(output, dpi=200)
    plt.close(figure)
    return output


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("batch_dir", type=Path, help="Directory containing seed_*.log")
    args = parser.parse_args()
    if not args.batch_dir.is_dir():
        parser.error(f"batch directory does not exist: {args.batch_dir}")

    rows = collect(args.batch_dir)
    csv_path = write_csv(args.batch_dir, rows)
    plot_path = write_plot(args.batch_dir, rows)
    print(f"Height data: {csv_path}")
    print(f"Height plot: {plot_path}")


if __name__ == "__main__":
    main()
