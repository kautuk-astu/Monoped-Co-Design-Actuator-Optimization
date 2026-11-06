#!/usr/bin/env python3
"""Run aligned CMA-ES using the legacy actuator_selection_nishant.csv schema.

This is intentionally separate from optimize_aligned.py.  It converts the
legacy catalogue into a run-local compatibility CSV and optimizes every
gearbox type that actually has a feasible row.  A gearbox with one catalogue
ratio keeps both joint ratios fixed and optimizes K, C, and T_gain.
"""

from __future__ import annotations

import argparse
import csv
from datetime import datetime
import json
from pathlib import Path
import shutil
import sys
import tempfile
import uuid

import numpy as np
import pandas as pd

from model import write_xml
from optimize_aligned import (
    BOUNDS,
    CMA_INITIAL_SIGMA,
    DEFAULT_LAMBDA_ENERGY,
    DEFAULT_LAMBDA_LANDING,
    DEFAULT_LAMBDA_X,
    FIXED_HIP_MOTOR,
    FIXED_KNEE_MOTOR,
    PARAMETER_NAMES,
    _git_commit,
    _record,
    design_from_vector,
    objective,
    replay_record,
    write_compact_log,
)
from simulate import INFEASIBILITY_COST, JumpResult, run_jump


HERE = Path(__file__).resolve().parent
DEFAULT_CATALOGUE = HERE / "data" / "actuator_selection_nishant.csv"
GEARBOX_ORDER = ("SSPG", "CPG", "WPG", "DSPG")
LEGACY_REQUIRED_COLUMNS = {
    "motor", "target_ratio", "actual_ratio", "gearbox", "mass",
    "efficiency", "status",
}


def adapt_catalogue(source: Path, destination: Path) -> tuple[pd.DataFrame, list[str]]:
    """Validate and write a current-schema copy without changing the source."""
    table = pd.read_csv(source)
    missing = sorted(LEGACY_REQUIRED_COLUMNS - set(table.columns))
    if missing:
        raise ValueError(f"Nishant catalogue is missing columns: {missing}")

    table["gearbox"] = table["gearbox"].astype(str).str.upper()
    table["status"] = table["status"].astype(str).str.upper()
    feasible = table[table["status"] == "FEASIBLE"].copy()
    if feasible.empty:
        raise ValueError("Nishant catalogue contains no FEASIBLE rows")

    for column in ("target_ratio", "actual_ratio", "mass", "efficiency"):
        feasible[column] = pd.to_numeric(feasible[column], errors="coerce")
    feasible = feasible.dropna(
        subset=["target_ratio", "actual_ratio", "mass", "efficiency"]
    )
    feasible = feasible[
        (feasible["target_ratio"] > 0)
        & (feasible["actual_ratio"] > 0)
        & (feasible["mass"] > 0)
        & (feasible["efficiency"] > 0)
        & (feasible["efficiency"] <= 1)
    ].copy()
    if feasible.empty:
        raise ValueError("Nishant catalogue has no usable numeric FEASIBLE rows")

    unknown = sorted(set(feasible["gearbox"]) - set(GEARBOX_ORDER))
    if unknown:
        raise ValueError(f"Unsupported gearbox types in Nishant catalogue: {unknown}")

    adapted = pd.DataFrame({
        "gearbox_type": feasible["gearbox"],
        "requested_ratio": feasible["target_ratio"],
        "actual_ratio": feasible["actual_ratio"],
        "mass_kg": feasible["mass"],
        "efficiency": feasible["efficiency"],
        "feasibility_status": "feasible",
        "failure_reason": feasible.get("reason", ""),
        "source_motor_label": feasible["motor"],
    }).sort_values(["gearbox_type", "requested_ratio", "actual_ratio"])
    destination.parent.mkdir(parents=True, exist_ok=True)
    adapted.to_csv(destination, index=False)
    available = [name for name in GEARBOX_ORDER if name in set(adapted["gearbox_type"])]
    return adapted, available


def ratio_bounds(table: pd.DataFrame, gearbox: str) -> tuple[np.ndarray, bool]:
    ratios = np.sort(
        table.loc[table["gearbox_type"] == gearbox, "requested_ratio"].unique()
    )
    if ratios.size == 0:
        raise ValueError(f"No feasible {gearbox} row")
    bounds = BOUNDS.copy()
    bounds[0] = bounds[1] = [float(ratios[0]), float(ratios[-1])]
    return bounds, ratios.size == 1


def full_vector(sample: np.ndarray, fixed_ratio: bool) -> np.ndarray:
    """Expand CMA's active variables to the optimizer's five-value vector."""
    sample = np.asarray(sample, dtype=float)
    if fixed_ratio:
        if sample.shape != (3,):
            raise ValueError("Expected K, C, and T_gain for a fixed-ratio gearbox")
        return np.array([0.5, 0.5, sample[0], sample[1], sample[2]])
    if sample.shape != (5,):
        raise ValueError("Expected five CMA variables")
    return sample


def optimize_gearbox(
    *, gearbox: str, source_catalogue: Path, catalogue: Path,
    table: pd.DataFrame, motor_config: Path,
    output_dir: Path, seed: int, maxiter: int, popsize: int,
    simulation_duration: float, lambda_x: float, lambda_landing: float,
    lambda_energy: float,
) -> dict[str, object]:
    import cma

    bounds, fixed_ratio = ratio_bounds(table, gearbox)
    active_names = list(PARAMETER_NAMES[2:] if fixed_ratio else PARAMETER_NAMES)
    x0 = np.array([0.78, 0.4, 0.5] if fixed_ratio else [0.5, 0.5, 0.78, 0.4, 0.5])
    output_dir.mkdir(parents=True, exist_ok=False)

    metadata = {
        "timestamp": datetime.now().isoformat(),
        "git_commit": _git_commit(),
        "python_version": sys.version,
        "controller": "aligned_jump_nishant_catalogue",
        "source_catalogue": str(source_catalogue.resolve()),
        "adapted_catalogue": str(catalogue.resolve()),
        "source_motor_label": "MAD_M6C12",
        "simulation_motor": FIXED_HIP_MOTOR,
        "motor_config": str(motor_config.resolve()),
        "gearbox": gearbox,
        "seed": seed,
        "popsize": popsize,
        "maxiter": maxiter,
        "simulation_duration_s": simulation_duration,
        "parameter_names": list(PARAMETER_NAMES),
        "active_cma_variables": active_names,
        "fixed_catalogue_ratio": fixed_ratio,
        "bounds": dict(zip(PARAMETER_NAMES, bounds.tolist())),
        "penalties": {
            "lambda_x": lambda_x,
            "lambda_landing": lambda_landing,
            "lambda_energy": lambda_energy,
        },
    }
    (output_dir / "metadata.json").write_text(json.dumps(metadata, indent=2) + "\n")

    es = cma.CMAEvolutionStrategy(x0, CMA_INITIAL_SIGMA, {
        "bounds": [np.zeros(len(x0)), np.ones(len(x0))],
        "maxiter": maxiter,
        "popsize": popsize,
        "seed": seed,
        "verb_disp": 1,
        "tolflatfitness": maxiter + 1,
    })
    best: dict[str, object] | None = None
    evaluation_index = 0

    with tempfile.TemporaryDirectory(prefix="aligned_nishant_", dir=output_dir) as temporary, \
            (output_dir / "all_candidates.csv").open("w", newline="") as all_stream, \
            (output_dir / "generation_best.csv").open("w", newline="") as generation_stream:
        candidate_xml = Path(temporary) / "candidate.xml"
        all_writer = generation_writer = None
        for generation in range(maxiter):
            samples = es.ask()
            costs: list[float] = []
            records: list[dict[str, object]] = []
            for active_sample in samples:
                evaluation_index += 1
                vector = full_vector(np.asarray(active_sample), fixed_ratio)
                design = design_from_vector(
                    vector,
                    gearbox=gearbox,
                    actuator_catalogue=catalogue,
                    motor_config=motor_config,
                    bounds=bounds,
                )
                try:
                    write_xml(design, candidate_xml)
                    result = run_jump(
                        candidate_xml, design, duration=simulation_duration,
                        controller_mode="aligned_jump",
                    )
                    violation = result.constraint_violation
                    if violation is None and not result.qualified_flight:
                        violation = "no completed qualified first flight"
                except Exception as exc:
                    result = JumpResult(
                        0, 0, 0, False, None, 0, 0, 0, 0, 0,
                        float("nan"), float("nan"), float("nan"),
                        f"evaluation error: {exc}",
                    )
                    violation = result.constraint_violation
                cost = (
                    objective(result, lambda_x, lambda_landing, lambda_energy)
                    if violation is None else INFEASIBILITY_COST
                )
                record = _record(
                    uuid.uuid4().hex[:10], generation, evaluation_index,
                    vector, design, result, float(cost), violation,
                )
                if all_writer is None:
                    all_writer = csv.DictWriter(all_stream, fieldnames=list(record))
                    generation_writer = csv.DictWriter(
                        generation_stream, fieldnames=list(record)
                    )
                    all_writer.writeheader()
                    generation_writer.writeheader()
                all_writer.writerow(record)
                all_stream.flush()
                records.append(record)
                costs.append(float(cost))
                if violation is None and (
                    best is None or float(cost) < float(best["cost"])
                ):
                    best = record.copy()
                    shutil.copy2(candidate_xml, output_dir / "best_design.xml")
                    (output_dir / "best_design.json").write_text(
                        json.dumps(best, indent=2, default=float) + "\n"
                    )
                    print(
                        f"{gearbox} new best: cost={cost:.6f}, "
                        f"base rise={result.apex_gain:.4f} m, "
                        f"toe error rms={result.alignment_rms * 1000:.2f} mm",
                        flush=True,
                    )
            valid = [record for record in records if record["valid"]]
            if valid:
                generation_writer.writerow(
                    min(valid, key=lambda item: float(item["cost"]))
                )
                generation_stream.flush()
            es.tell(samples, costs)

    if best is None:
        (output_dir / "best_design.json").write_text(
            json.dumps({"best_valid_candidate": None}, indent=2) + "\n"
        )
        replay = None
        summary = "No valid jump was found; see all_candidates.csv.\n"
    else:
        replay = replay_record(best, output_dir, simulation_duration)
        maximum_base_height = max(
            (float(sample.hip_z) for sample in replay.telemetry),
            default=float(replay.apex_height),
        )
        summary = (
            f"Initial base height: {replay.initial_height:.6f} m\n"
            f"Maximum base height: {maximum_base_height:.6f} m\n"
            f"Total base rise: {maximum_base_height - replay.initial_height:.6f} m\n"
            f"Toe horizontal error RMS: {replay.alignment_rms:.6f} m\n"
            f"Valid jump: {replay.qualified_flight}\n"
            f"Cost: {float(best['cost']):.6f}\n"
        )
    write_compact_log(
        output_dir, best, replay, seed=seed, popsize=popsize, maxiter=maxiter,
        simulation_duration=simulation_duration, lambda_x=lambda_x,
        lambda_landing=lambda_landing, lambda_energy=lambda_energy,
        bounds=bounds,
    )
    (output_dir / "summary.txt").write_text(summary)
    print(summary, flush=True)
    return {
        "gearbox": gearbox,
        "valid_candidate": best is not None,
        "cost": "" if best is None else float(best["cost"]),
        "hip_actual_ratio": "" if best is None else float(best["hip_gear_ratio"]),
        "knee_actual_ratio": "" if best is None else float(best["knee_gear_ratio"]),
        "fixed_catalogue_ratio": fixed_ratio,
        "run_directory": str(output_dir.resolve()),
    }


def write_comparison(output_root: Path, rows: list[dict[str, object]]) -> None:
    path = output_root / "gearbox_comparison.csv"
    with path.open("w", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)
    valid = [row for row in rows if row["valid_candidate"]]
    winner = min(valid, key=lambda row: float(row["cost"])) if valid else None
    (output_root / "best_overall.json").write_text(
        json.dumps({"best_overall": winner, "all_gearboxes": rows}, indent=2) + "\n"
    )


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--catalogue", type=Path, default=DEFAULT_CATALOGUE)
    parser.add_argument("--motor-config", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--seed", type=int, default=5)
    parser.add_argument("--maxiter", type=int, default=1000)
    parser.add_argument("--popsize", type=int, default=8)
    parser.add_argument("--simulation-duration", type=float, default=10.0)
    parser.add_argument("--lambda-x", type=float, default=DEFAULT_LAMBDA_X)
    parser.add_argument("--lambda-landing", type=float, default=DEFAULT_LAMBDA_LANDING)
    parser.add_argument("--lambda-energy", type=float, default=DEFAULT_LAMBDA_ENERGY)
    args = parser.parse_args()
    if not args.catalogue.is_file():
        parser.error(f"catalogue does not exist: {args.catalogue}")
    if not args.motor_config.is_file():
        parser.error(f"motor config does not exist: {args.motor_config}")
    if args.output_dir.exists():
        parser.error(f"output directory already exists: {args.output_dir}")
    if args.maxiter <= 0 or args.popsize <= 0 or args.simulation_duration <= 0:
        parser.error("maxiter, popsize, and simulation duration must be positive")

    args.output_dir.mkdir(parents=True)
    adapted_path = args.output_dir / "adapted_nishant_catalogue.csv"
    table, available = adapt_catalogue(args.catalogue, adapted_path)
    unavailable = [name for name in GEARBOX_ORDER if name not in available]
    print(f"Available gearbox types: {', '.join(available)}")
    if unavailable:
        print(f"Skipped (no feasible catalogue row): {', '.join(unavailable)}")

    rows = []
    for gearbox in available:
        rows.append(optimize_gearbox(
            gearbox=gearbox,
            source_catalogue=args.catalogue,
            catalogue=adapted_path,
            table=table,
            motor_config=args.motor_config,
            output_dir=args.output_dir / gearbox.lower(),
            seed=args.seed,
            maxiter=args.maxiter,
            popsize=args.popsize,
            simulation_duration=args.simulation_duration,
            lambda_x=args.lambda_x,
            lambda_landing=args.lambda_landing,
            lambda_energy=args.lambda_energy,
        ))
    write_comparison(args.output_dir, rows)
    print(f"Results: {args.output_dir}")


if __name__ == "__main__":
    main()
