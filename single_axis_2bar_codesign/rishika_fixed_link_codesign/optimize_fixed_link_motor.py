"""CMA-ES optimizer with fixed 0.20 m links and MAD_M6C12 motors at both joints."""
from __future__ import annotations

import argparse
import csv
from dataclasses import replace
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
K_INDEX = PARAMETER_NAMES.index("K")
RATIO_INDICES = (PARAMETER_NAMES.index("hip_gear_ratio"), PARAMETER_NAMES.index("knee_gear_ratio"))
ORI_L_INDEX = PARAMETER_NAMES.index("ori_l")
ENERGY_METRICS = ("net", "per-joint")


def make_bounds(
    k_max: float | None = None, ratio_max: float | None = None, ori_l_max: float | None = None,
) -> np.ndarray:
    """Return a copy of BOUNDS with the K, gear-ratio and/or ori_l upper limits replaced."""
    bounds = BOUNDS.copy()
    if ori_l_max is not None:
        if not ori_l_max > BOUNDS[ORI_L_INDEX, 0]:
            raise ValueError(f"--ori-l-max must exceed the lower ori_l bound {BOUNDS[ORI_L_INDEX, 0]:g}")
        bounds[ORI_L_INDEX, 1] = ori_l_max
    if k_max is not None:
        if not k_max > BOUNDS[K_INDEX, 0]:
            raise ValueError(f"--k-max must exceed the lower K bound {BOUNDS[K_INDEX, 0]:g}")
        bounds[K_INDEX, 1] = k_max
    if ratio_max is not None:
        lower = BOUNDS[RATIO_INDICES[0], 0]
        if not ratio_max > lower:
            raise ValueError(f"--ratio-max must exceed the lower gear-ratio bound {lower:g}")
        for index in RATIO_INDICES:
            bounds[index, 1] = ratio_max
    return bounds


def denormalize(vector: np.ndarray, bounds: np.ndarray = BOUNDS) -> np.ndarray:
    vector = np.asarray(vector, dtype=float)
    if vector.shape != (len(PARAMETER_NAMES),) or not np.isfinite(vector).all() or np.any(vector < 0) or np.any(vector > 1):
        raise ValueError("normalized CMA vector must be finite, length 11, and within [0, 1]")
    return bounds[:, 0] + vector * (bounds[:, 1] - bounds[:, 0])


def design_from_vector(
    vector: np.ndarray, bounds: np.ndarray = BOUNDS, gearbox_efficiency: float | None = None,
    payload_mass: float | None = None,
) -> Design:
    """Build a design; ``gearbox_efficiency`` overrides the table value for both actuators.

    A fixed ``payload_mass`` replaces the sampled payload, which then has no effect on cost.
    """
    p = denormalize(vector, bounds)
    if payload_mass is not None:
        p[2] = payload_mass
    hip = actuator_for(FIXED_HIP_MOTOR, float(p[3]))
    knee = actuator_for(FIXED_KNEE_MOTOR, float(p[4]))
    if gearbox_efficiency is not None:
        hip = replace(hip, efficiency=gearbox_efficiency)
        knee = replace(knee, efficiency=gearbox_efficiency)
    return Design(
        thigh=FIXED_L1_M, shank=FIXED_L2_M, theta1=float(p[0]), theta2=float(p[1]), payload_mass=float(p[2]),
        hip=hip, knee=knee,
        ori_l=float(p[5]), ori_theta=float(p[6]), K=float(p[7]), C=float(p[8]), T_gain=float(p[9]), thrust_time=float(p[10]),
    )


def energy_for_cost(result, metric: str) -> float:
    """``net``: max(0, hip+knee power) per step.  ``per-joint``: hip and knee positive work summed separately."""
    if metric == "net":
        return float(result.positive_work)
    if metric == "per-joint":
        return float(result.hip_positive_work + result.knee_positive_work)
    raise ValueError(f"unknown energy metric {metric!r}")


def main() -> None:
    import cma
    from simulate import CONTROL_MODES, INFEASIBILITY_COST, JumpResult, run_jump
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--maxiter", "--iterations", dest="maxiter", type=int, default=100)
    parser.add_argument("--popsize", "--population", dest="popsize", type=int, default=8)
    parser.add_argument("--seed", type=int, default=5)
    parser.add_argument("--energy-weight", type=float, default=0.015)
    parser.add_argument("--output-dir", type=Path)
    parser.add_argument(
        "--energy-metric", choices=ENERGY_METRICS, default="net",
        help="net: max(0, hip+knee power) (default); per-joint: hip and knee positive work summed separately",
    )
    parser.add_argument(
        "--gearbox-efficiency", type=float, default=None,
        help="override both actuators' gearbox efficiency (e.g. 0.7 for 3D-printed gears); default: lookup table",
    )
    parser.add_argument("--k-max", type=float, default=float(BOUNDS[K_INDEX, 1]), help="upper bound of the VMC stiffness K")
    parser.add_argument(
        "--ratio-max", type=float, default=float(BOUNDS[RATIO_INDICES[0], 1]),
        help="upper bound of both hip and knee gear ratios (e.g. 8.7 for single-stage QDD)",
    )
    parser.add_argument(
        "--ori-l-max", type=float, default=float(BOUNDS[ORI_L_INDEX, 1]),
        help="upper bound of the VMC rest length ori_l in m (e.g. 0.395, the reach limit for 0.20 m links)",
    )
    parser.add_argument("--payload", type=float, default=None, help="fix the carriage payload mass in kg instead of optimizing it")
    parser.add_argument(
        "--control-mode", choices=CONTROL_MODES, default="thrust-window",
        help="thrust-window: VMC only during thrust_time (default); contact: VMC whenever the foot touches "
             "the ground, as in the co-design paper (also acts on landing; thrust_time unused)",
    )
    parser.add_argument(
        "--min-apex-gain", type=float, default=0.0,
        help="treat candidates whose apex gain is below this many metres as invalid (default 0: disabled)",
    )
    args = parser.parse_args()
    if args.min_apex_gain < 0:
        parser.error("--min-apex-gain must be non-negative")
    if args.gearbox_efficiency is not None and not 0 < args.gearbox_efficiency <= 1:
        parser.error("--gearbox-efficiency must be in (0, 1]")
    if args.payload is not None and not args.payload > 0:
        parser.error("--payload must be positive")
    try:
        bounds = make_bounds(args.k_max, args.ratio_max, args.ori_l_max)
    except ValueError as exc:
        parser.error(str(exc))

    run_id = datetime.now().strftime("fixed_link_motor_run_%Y-%m-%d_%H-%M-%S") + f"_seed{args.seed}"
    output_dir = args.output_dir or RESULTS / run_id
    output_dir.mkdir(parents=True, exist_ok=True)
    metadata = {
        "timestamp": datetime.now().isoformat(), "git_commit": _git_commit(), "seed": args.seed,
        "popsize": args.popsize, "maxiter": args.maxiter, "energy_weight": args.energy_weight,
        "python_version": sys.version, "fixed_l1_m": FIXED_L1_M, "fixed_l2_m": FIXED_L2_M,
        "fixed_hip_motor": FIXED_HIP_MOTOR, "fixed_knee_motor": FIXED_KNEE_MOTOR,
        "candidate_xml_logging": "temporary only; best_design.xml retained",
        "energy_metric": args.energy_metric, "gearbox_efficiency_override": args.gearbox_efficiency,
        "k_bounds": bounds[K_INDEX].tolist(), "gear_ratio_bounds": bounds[RATIO_INDICES[0]].tolist(),
        "fixed_payload_mass": args.payload, "min_apex_gain_m": args.min_apex_gain, "control_mode": args.control_mode,
        "ori_l_bounds": bounds[ORI_L_INDEX].tolist(),
    }
    try:
        import mujoco
        metadata["mujoco_version"] = mujoco.__version__
    except Exception:
        metadata["mujoco_version"] = None
    (output_dir / "metadata.json").write_text(json.dumps(metadata, indent=2))

    x0_physical = np.array([-.70, 1.40, 1.0, 10, 10, .37, 0, 600, 5, 20, .15])
    # Narrowed bounds (e.g. --ratio-max 8.7) can exclude the seed, so keep it inside them.
    x0_physical = np.clip(x0_physical, bounds[:, 0], bounds[:, 1])
    x0 = (x0_physical - bounds[:, 0]) / (bounds[:, 1] - bounds[:, 0])
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
                design = design_from_vector(np.asarray(sample), bounds, args.gearbox_efficiency, args.payload)
                try:
                    write_xml(design, candidate_xml)
                    result = run_jump(candidate_xml, design, control_mode=args.control_mode)
                    violation = result.constraint_violation or (None if result.takeoff else "no takeoff")
                    # Losing foot contact without rising otherwise counts as a valid, zero-cost
                    # "jump" that CMA-ES can get stuck on.
                    if violation is None and result.apex_gain < args.min_apex_gain:
                        violation = f"apex gain below {args.min_apex_gain:g} m"
                except Exception as exc:
                    result = JumpResult(0, 0, 0, False, None, 0, 0, 0, 0, 0, float("nan"), float("nan"), float("nan"), f"evaluation error: {exc}")
                    violation = result.constraint_violation
                valid = violation is None
                cost = (-result.apex_gain + args.energy_weight * energy_for_cost(result, args.energy_metric)) if valid else INFEASIBILITY_COST
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
