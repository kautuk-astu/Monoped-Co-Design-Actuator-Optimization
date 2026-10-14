"""CMA-ES optimizer with fixed 0.20 m links and MAD_M6C12 motors at both joints."""
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

from model import Design, actuator_for, write_xml
from optimize_fixed_link import (
    FIXED_L1_M, FIXED_L2_M, HEADERS, HIP_ANGLE_BOUNDS, KNEE_ANGLE_BOUNDS,
    ORI_L_GLOBAL_BOUNDS, RESULTS, _git_commit, _record, _summary,
)

HERE = Path(__file__).resolve().parent
FIXED_HIP_MOTOR = "MAD_M6C12"
FIXED_KNEE_MOTOR = "MAD_M6C12"
PARAMETER_NAMES = (
    "theta1", "theta2", "payload_mass", "hip_gear_ratio", "knee_gear_ratio",
    "ori_l", "ori_theta", "K", "C", "T_gain", "thrust_time",
)
BOUNDS = np.array([
    HIP_ANGLE_BOUNDS, KNEE_ANGLE_BOUNDS, [0.25, 3.0], [4, 25], [4, 25],
    ORI_L_GLOBAL_BOUNDS, [-np.pi / 2, np.pi / 2], [50, 1000], [0, 10], [10, 50], [0.04, 0.25],
], dtype=float)


def denormalize(vector: np.ndarray) -> np.ndarray:
    vector = np.asarray(vector, dtype=float)
    if vector.shape != (len(PARAMETER_NAMES),) or not np.isfinite(vector).all() or np.any(vector < 0) or np.any(vector > 1):
        raise ValueError("normalized CMA vector must be finite, length 11, and within [0, 1]")
    return BOUNDS[:, 0] + vector * (BOUNDS[:, 1] - BOUNDS[:, 0])


def design_from_vector(vector: np.ndarray) -> Design:
    p = denormalize(vector)
    return Design(
        thigh=FIXED_L1_M, shank=FIXED_L2_M, theta1=float(p[0]), theta2=float(p[1]), payload_mass=float(p[2]),
        hip=actuator_for(FIXED_HIP_MOTOR, float(p[3])), knee=actuator_for(FIXED_KNEE_MOTOR, float(p[4])),
        ori_l=float(p[5]), ori_theta=float(p[6]), K=float(p[7]), C=float(p[8]), T_gain=float(p[9]), thrust_time=float(p[10]),
    )


def main() -> None:
    import cma
    from simulate import INFEASIBILITY_COST, JumpResult, run_jump
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--maxiter", "--iterations", dest="maxiter", type=int, default=100)
    parser.add_argument("--popsize", "--population", dest="popsize", type=int, default=8)
    parser.add_argument("--seed", type=int, default=5)
    parser.add_argument("--energy-weight", type=float, default=0.015)
    parser.add_argument("--output-dir", type=Path)
    args = parser.parse_args()

    run_id = datetime.now().strftime("fixed_link_motor_run_%Y-%m-%d_%H-%M-%S") + f"_seed{args.seed}"
    output_dir = args.output_dir or RESULTS / run_id
    output_dir.mkdir(parents=True, exist_ok=True)
    metadata = {
        "timestamp": datetime.now().isoformat(), "git_commit": _git_commit(), "seed": args.seed,
        "popsize": args.popsize, "maxiter": args.maxiter, "energy_weight": args.energy_weight,
        "python_version": sys.version, "fixed_l1_m": FIXED_L1_M, "fixed_l2_m": FIXED_L2_M,
        "fixed_hip_motor": FIXED_HIP_MOTOR, "fixed_knee_motor": FIXED_KNEE_MOTOR,
        "candidate_xml_logging": "temporary only; best_design.xml retained",
    }
    try:
        import mujoco
        metadata["mujoco_version"] = mujoco.__version__
    except Exception:
        metadata["mujoco_version"] = None
    (output_dir / "metadata.json").write_text(json.dumps(metadata, indent=2))

    x0_physical = np.array([-.70, 1.40, 1.0, 10, 10, .37, 0, 600, 5, 20, .15])
    x0 = (x0_physical - BOUNDS[:, 0]) / (BOUNDS[:, 1] - BOUNDS[:, 0])
    es = cma.CMAEvolutionStrategy(x0, .06, {
        "bounds": [np.zeros(len(PARAMETER_NAMES)), np.ones(len(PARAMETER_NAMES))],
        "maxiter": args.maxiter, "popsize": args.popsize, "seed": args.seed, "verb_disp": 1,
        "tolflatfitness": args.maxiter + 1,
    })
    best: dict[str, object] | None = None
    evaluation_index = 0
    # Candidate XMLs are sequentially overwritten in a temporary directory.
    # Only the current global-best XML is copied into the persistent run output.
    with tempfile.TemporaryDirectory(prefix="monoped_candidate_") as temporary_dir, \
            (output_dir / "all_candidates.csv").open("w", newline="") as all_file, \
            (output_dir / "generation_best.csv").open("w", newline="") as gen_file, \
            (output_dir / "best_so_far.csv").open("w", newline="") as best_file:
        candidate_xml = Path(temporary_dir) / "candidate.xml"
        all_writer, gen_writer, best_writer = (csv.DictWriter(f, fieldnames=HEADERS) for f in (all_file, gen_file, best_file))
        for writer in (all_writer, gen_writer, best_writer):
            writer.writeheader()
        for generation in range(args.maxiter):
            samples, costs, records = es.ask(), [], []
            for sample in samples:
                evaluation_index += 1
                candidate_id = uuid.uuid4().hex[:10]
                design = design_from_vector(np.asarray(sample))
                try:
                    write_xml(design, candidate_xml)
                    result = run_jump(candidate_xml, design)
                    violation = result.constraint_violation or (None if result.takeoff else "no takeoff")
                except Exception as exc:
                    result = JumpResult(0, 0, 0, False, None, 0, 0, 0, 0, 0, float("nan"), float("nan"), float("nan"), f"evaluation error: {exc}")
                    violation = result.constraint_violation
                valid = violation is None
                cost = (-result.apex_gain + args.energy_weight * result.positive_work) if valid else INFEASIBILITY_COST
                record = _record(candidate_id, generation, evaluation_index, design, result, float(cost), violation, valid)
                all_writer.writerow(record)
                all_file.flush()
                records.append(record)
                costs.append(float(cost))
                if valid and (best is None or float(cost) < float(best["cost"])):
                    best = record
                    shutil.copy2(candidate_xml, output_dir / "best_design.xml")
                    best_writer.writerow(record)
                    best_file.flush()
                    print(f"new global best: {cost:.6f}; jump={result.apex_gain:.4f} m")
            valid_records = [record for record in records if record["valid"]]
            if valid_records:
                gen_writer.writerow(min(valid_records, key=lambda record: float(record["cost"])))
                gen_file.flush()
            es.tell(samples, costs)

    if best is None:
        summary = "No valid takeoff candidate was found; see all_candidates.csv.\n"
        (output_dir / "best_design.json").write_text(json.dumps({"best_valid_candidate": None}, indent=2))
        (output_dir / "best_design.csv").write_text("")
    else:
        (output_dir / "best_design.json").write_text(json.dumps(best, indent=2, default=float))
        with (output_dir / "best_design.csv").open("w", newline="") as stream:
            writer = csv.DictWriter(stream, fieldnames=HEADERS)
            writer.writeheader()
            writer.writerow(best)
        summary = _summary(best)
    (output_dir / "summary.txt").write_text(summary)
    print(summary)
    print(f"Results: {output_dir}")


if __name__ == "__main__":
    main()
