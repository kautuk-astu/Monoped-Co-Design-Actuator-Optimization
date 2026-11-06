#!/usr/bin/env python3
"""Consolidate the four fixed-topology aligned CMA-ES runs."""

from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path


GEARBOXES = ("SSPG", "CPG", "WPG", "DSPG")
FIELDS = (
    "gearbox", "valid_candidate", "cost", "hip_actual_ratio", "knee_actual_ratio",
    "hip_actuator_mass_kg", "knee_actuator_mass_kg", "maximum_base_height_m",
    "total_base_rise_m", "toe_horizontal_error_rms_m", "run_directory",
)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("run_root", type=Path)
    args = parser.parse_args()
    rows: list[dict[str, object]] = []

    for gearbox in GEARBOXES:
        run_dir = args.run_root / gearbox.lower()
        best_path = run_dir / "best_design.json"
        compact_log_path = run_dir / "compact_optimization_log.json"
        best = json.loads(best_path.read_text()) if best_path.is_file() else {}
        compact_log = json.loads(compact_log_path.read_text()) if compact_log_path.is_file() else {}
        key_results = compact_log.get("key_results", {})
        valid = best.get("best_valid_candidate", True) is not None and "cost" in best
        rows.append({
            "gearbox": gearbox,
            "valid_candidate": valid,
            "cost": best.get("cost", ""),
            "hip_actual_ratio": best.get("hip_gear_ratio", ""),
            "knee_actual_ratio": best.get("knee_gear_ratio", ""),
            "hip_actuator_mass_kg": best.get("hip_actuator_mass", ""),
            "knee_actuator_mass_kg": best.get("knee_actuator_mass", ""),
            "maximum_base_height_m": key_results.get("maximum_base_height_m", ""),
            "total_base_rise_m": key_results.get("total_base_rise_m", ""),
            "toe_horizontal_error_rms_m": key_results.get("toe_horizontal_error_rms_m", ""),
            "run_directory": str(run_dir.resolve()),
        })

    comparison_path = args.run_root / "gearbox_comparison.csv"
    with comparison_path.open("w", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=FIELDS)
        writer.writeheader()
        writer.writerows(rows)

    valid_rows = [row for row in rows if row["valid_candidate"]]
    winner = min(valid_rows, key=lambda row: float(row["cost"])) if valid_rows else None
    (args.run_root / "best_overall.json").write_text(
        json.dumps({"best_overall": winner, "all_gearboxes": rows}, indent=2) + "\n"
    )
    print(f"Comparison: {comparison_path}")
    if winner is None:
        print("No gearbox run produced a valid candidate.")
    else:
        print(f"Best gearbox: {winner['gearbox']} (cost={float(winner['cost']):.6f})")


if __name__ == "__main__":
    main()
