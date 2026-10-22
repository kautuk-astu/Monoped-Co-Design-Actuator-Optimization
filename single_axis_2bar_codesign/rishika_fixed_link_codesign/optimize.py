"""CMA-ES full co-design optimizer for the vertically guided two-link monoped."""
from __future__ import annotations

import argparse
import csv
from datetime import datetime
import json
from pathlib import Path
import subprocess
import sys
import uuid

import numpy as np

from model import CARRIAGE_BASE_MASS_KG, Design, MOTORS, actuator_for, write_xml

HERE = Path(__file__).resolve().parent
RESULTS = HERE / "results"
HIP_ANGLE_BOUNDS = (-np.pi / 2, np.pi / 2)
KNEE_ANGLE_MARGIN_RAD = 0.03
KNEE_ANGLE_BOUNDS = (KNEE_ANGLE_MARGIN_RAD, np.pi - KNEE_ANGLE_MARGIN_RAD)
ORI_L_GLOBAL_BOUNDS = (0.05, 0.60)
PARAMETER_NAMES = (
    "l1", "l2", "theta1", "theta2", "payload_mass", "hip_motor_idx", "knee_motor_idx",
    "hip_gear_ratio", "knee_gear_ratio", "ori_l", "ori_theta", "K", "C", "T_gain", "thrust_time",
)
BOUNDS = np.array([
    [0.15, 0.30], [0.15, 0.30], HIP_ANGLE_BOUNDS, KNEE_ANGLE_BOUNDS, [0.25, 3.0],
    [1, 6], [1, 6], [4, 25], [4, 25], ORI_L_GLOBAL_BOUNDS,
    [-np.pi / 2, np.pi / 2], [50, 1000], [0, 10], [10, 50], [0.04, 0.25],
], dtype=float)


def denormalize(vector: np.ndarray) -> np.ndarray:
    vector = np.asarray(vector, dtype=float)
    if vector.shape != (15,) or not np.isfinite(vector).all() or np.any(vector < 0) or np.any(vector > 1):
        raise ValueError("normalized CMA vector must be finite, length 15, and within [0, 1]")
    return BOUNDS[:, 0] + vector * (BOUNDS[:, 1] - BOUNDS[:, 0])


def motor_name(value: float) -> str:
    return MOTORS[int(np.clip(round(value), 1, len(MOTORS))) - 1]


def design_from_vector(vector: np.ndarray) -> Design:
    p = denormalize(vector)
    return Design(
        thigh=float(p[0]), shank=float(p[1]), theta1=float(p[2]), theta2=float(p[3]), payload_mass=float(p[4]),
        hip=actuator_for(motor_name(p[5]), float(p[7])), knee=actuator_for(motor_name(p[6]), float(p[8])),
        ori_l=float(p[9]), ori_theta=float(p[10]), K=float(p[11]), C=float(p[12]), T_gain=float(p[13]), thrust_time=float(p[14]),
    )


HEADERS = [
    "candidate_id", "generation", "evaluation_index", "cost", "apex_gain_m", "apex_height_m", "initial_height_m",
    "positive_work_j", "hip_positive_work_j", "knee_positive_work_j", "peak_power_w", "peak_force_n", "takeoff", "takeoff_time_s",
    "l1", "l2", "theta1_rad", "theta1_deg", "theta2_rad", "theta2_deg", "payload_mass",
    "hip_motor_idx", "hip_motor", "hip_gear_ratio", "hip_gearbox", "hip_efficiency", "hip_actuator_mass", "hip_torque_limit",
    "knee_motor_idx", "knee_motor", "knee_gear_ratio", "knee_gearbox", "knee_efficiency", "knee_actuator_mass", "knee_torque_limit",
    "total_actuator_mass", "total_carriage_mass", "ori_l", "ori_theta_rad", "ori_theta_deg", "K", "C", "T_gain", "thrust_time",
    "minimum_toe_to_rail_clearance", "minimum_toe_to_hip_clearance", "minimum_link_to_body_clearance",
    "constraint_violation", "constraint_violation_reason", "valid",
]


def _record(candidate_id: str, generation: int, evaluation_index: int, design: Design, result, cost: float, violation: str | None, valid: bool) -> dict[str, object]:
    hip_limit = design.hip.efficiency * design.hip.continuous_output_torque
    knee_limit = design.knee.efficiency * design.knee.continuous_output_torque
    return {
        "candidate_id": candidate_id, "generation": generation, "evaluation_index": evaluation_index, "cost": cost,
        "apex_gain_m": result.apex_gain, "apex_height_m": result.apex_height, "initial_height_m": result.initial_height,
        "positive_work_j": result.positive_work, "hip_positive_work_j": result.hip_positive_work,
        "knee_positive_work_j": result.knee_positive_work, "peak_power_w": result.peak_power, "peak_force_n": result.peak_force,
        "takeoff": result.takeoff, "takeoff_time_s": result.takeoff_time,
        "l1": design.thigh, "l2": design.shank, "theta1_rad": design.theta1, "theta1_deg": np.degrees(design.theta1),
        "theta2_rad": design.theta2, "theta2_deg": np.degrees(design.theta2), "payload_mass": design.payload_mass,
        "hip_motor_idx": MOTORS.index(design.hip.motor) + 1, "hip_motor": design.hip.motor, "hip_gear_ratio": design.hip.ratio,
        "hip_gearbox": design.hip.gearbox, "hip_efficiency": design.hip.efficiency, "hip_actuator_mass": design.hip.mass, "hip_torque_limit": hip_limit,
        "knee_motor_idx": MOTORS.index(design.knee.motor) + 1, "knee_motor": design.knee.motor, "knee_gear_ratio": design.knee.ratio,
        "knee_gearbox": design.knee.gearbox, "knee_efficiency": design.knee.efficiency, "knee_actuator_mass": design.knee.mass, "knee_torque_limit": knee_limit,
        "total_actuator_mass": design.hip.mass + design.knee.mass,
        "total_carriage_mass": CARRIAGE_BASE_MASS_KG + design.payload_mass + design.hip.mass + design.knee.mass,
        "ori_l": design.ori_l, "ori_theta_rad": design.ori_theta, "ori_theta_deg": np.degrees(design.ori_theta),
        "K": design.K, "C": design.C, "T_gain": design.T_gain, "thrust_time": design.thrust_time,
        "minimum_toe_to_rail_clearance": result.minimum_toe_to_pole_clearance,
        "minimum_toe_to_hip_clearance": result.minimum_hip_to_toe_clearance,
        "minimum_link_to_body_clearance": result.minimum_link_to_body_clearance,
        "constraint_violation": bool(violation), "constraint_violation_reason": violation or "", "valid": valid,
    }


def _git_commit() -> str | None:
    try:
        return subprocess.check_output(["git", "rev-parse", "HEAD"], cwd=HERE.parent, text=True).strip()
    except Exception:
        return None


def _summary(record: dict[str, object]) -> str:
    return """============================================================
BEST TWO-LINK MONOPED CO-DESIGN
============================================================

Performance
-----------
Vertical jump (apex gain):  {apex_gain_m:.6f} m
Absolute apex height:       {apex_height_m:.6f} m
Initial height:             {initial_height_m:.6f} m
Mechanical energy:          {positive_work_j:.6f} J
Hip mechanical work:        {hip_positive_work_j:.6f} J
Knee mechanical work:       {knee_positive_work_j:.6f} J
Best cost:                  {cost:.6f}
Peak power:                 {peak_power_w:.6f} W
Takeoff time:               {takeoff_time_s} s

Morphology
----------
Thigh L1:   {l1:.6f} m ({l1_mm:.3f} mm)
Shank L2:   {l2:.6f} m ({l2_mm:.3f} mm)
Theta1:     {theta1_rad:.6f} rad ({theta1_deg:.3f} deg)
Theta2:     {theta2_rad:.6f} rad ({theta2_deg:.3f} deg)

Payload
-------
{payload_mass:.6f} kg

Hip Actuator
------------
Motor: {hip_motor}; Gearbox: {hip_gearbox}; Ratio: {hip_gear_ratio:.6f}; Efficiency: {hip_efficiency:.6f}
Mass: {hip_actuator_mass:.6f} kg; Torque limit: {hip_torque_limit:.6f} Nm

Knee Actuator (physically mounted at hip)
-----------------------------------------
Motor: {knee_motor}; Gearbox: {knee_gearbox}; Ratio: {knee_gear_ratio:.6f}; Efficiency: {knee_efficiency:.6f}
Mass: {knee_actuator_mass:.6f} kg; Torque limit: {knee_torque_limit:.6f} Nm

Controller
----------
ori_l: {ori_l:.6f} m; ori_theta: {ori_theta_rad:.6f} rad ({ori_theta_deg:.3f} deg)
K: {K:.6f}; C: {C:.6f}; T_gain: {T_gain:.6f}; Thrust time: {thrust_time:.6f} s

System / Feasibility
--------------------
Total actuator mass: {total_actuator_mass:.6f} kg
Moving carriage mass: {total_carriage_mass:.6f} kg
Toe/rail min clearance: {minimum_toe_to_rail_clearance:.6f} m
Toe/hip min clearance: {minimum_toe_to_hip_clearance:.6f} m
Link/rail min clearance: {minimum_link_to_body_clearance:.6f} m
Valid: YES
============================================================
""".format(**record, l1_mm=float(record["l1"]) * 1000, l2_mm=float(record["l2"]) * 1000)


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
    run_id = datetime.now().strftime("run_%Y-%m-%d_%H-%M-%S") + f"_seed{args.seed}"
    output_dir = args.output_dir or RESULTS / run_id
    generated = output_dir / "generated_xml"
    output_dir.mkdir(parents=True, exist_ok=True)
    generated.mkdir()
    metadata = {"timestamp": datetime.now().isoformat(), "git_commit": _git_commit(), "seed": args.seed, "popsize": args.popsize,
                "maxiter": args.maxiter, "energy_weight": args.energy_weight, "python_version": sys.version}
    try:
        import mujoco
        metadata["mujoco_version"] = mujoco.__version__
    except Exception:
        metadata["mujoco_version"] = None
    (output_dir / "metadata.json").write_text(json.dumps(metadata, indent=2))
    # A collision-free crouched seed: theta1=-theta2/2 keeps the toe near the
    # rail centerline without imposing that relation on sampled candidates.
    # Its initial virtual-leg length is about 0.31 m and ori_l=0.37 m commands
    # extension, unlike the former nearly straight, non-thrusting seed.
    x0_physical = np.array([.20, .20, -.70, 1.40, 1, 1, 1, 10, 10, .37, 0, 600, 5, 20, .15])
    x0 = (x0_physical - BOUNDS[:, 0]) / (BOUNDS[:, 1] - BOUNDS[:, 0])
    es = cma.CMAEvolutionStrategy(x0, .06, {
        "bounds": [np.zeros(15), np.ones(15)], "maxiter": args.maxiter,
        "popsize": args.popsize, "seed": args.seed, "verb_disp": 1,
        # Do not end a study after the first population merely because every
        # candidate has the same 1e6 infeasibility cost.
        "tolflatfitness": args.maxiter + 1,
    })
    best: dict[str, object] | None = None
    evaluation_index = 0
    with (output_dir / "all_candidates.csv").open("w", newline="") as all_file, (output_dir / "generation_best.csv").open("w", newline="") as gen_file, (output_dir / "best_so_far.csv").open("w", newline="") as best_file:
        all_writer, gen_writer, best_writer = (csv.DictWriter(f, fieldnames=HEADERS) for f in (all_file, gen_file, best_file))
        for writer in (all_writer, gen_writer, best_writer): writer.writeheader()
        # Continue for the requested study length even if a first population
        # is flat at the hard infeasibility cost.  The usual ``while
        # not es.stop()`` would otherwise terminate after only one generation
        # before CMA-ES has sampled enough feasible morphology/pose pairs.
        for generation in range(args.maxiter):
            samples, costs, records = es.ask(), [], []
            for sample in samples:
                evaluation_index += 1; candidate_id = uuid.uuid4().hex[:10]; design = design_from_vector(np.asarray(sample))
                try:
                    xml_path = generated / f"{candidate_id}.xml"; write_xml(design, xml_path); result = run_jump(xml_path, design)
                    violation = result.constraint_violation or (None if result.takeoff else "no takeoff")
                except Exception as exc:
                    result = JumpResult(0, 0, 0, False, None, 0, 0, 0, 0, 0, float("nan"), float("nan"), float("nan"), f"evaluation error: {exc}")
                    violation = result.constraint_violation
                valid = violation is None
                cost = (-result.apex_gain + args.energy_weight * result.positive_work) if valid else INFEASIBILITY_COST
                record = _record(candidate_id, generation, evaluation_index, design, result, float(cost), violation, valid)
                all_writer.writerow(record); all_file.flush(); records.append(record); costs.append(float(cost))
                if valid and (best is None or float(cost) < float(best["cost"])):
                    best = record; best_writer.writerow(record); best_file.flush(); print(f"new global best: {cost:.6f}; jump={result.apex_gain:.4f} m")
            valid_records = [r for r in records if r["valid"]]
            if valid_records: gen_writer.writerow(min(valid_records, key=lambda r: float(r["cost"]))); gen_file.flush()
            es.tell(samples, costs)
    if best is None:
        summary = "No valid takeoff candidate was found; see all_candidates.csv.\n"
        (output_dir / "best_design.json").write_text(json.dumps({"best_valid_candidate": None}, indent=2))
        (output_dir / "best_design.csv").write_text("")
    else:
        (output_dir / "best_design.json").write_text(json.dumps(best, indent=2, default=float))
        with (output_dir / "best_design.csv").open("w", newline="") as stream:
            writer = csv.DictWriter(stream, fieldnames=HEADERS); writer.writeheader(); writer.writerow(best)
        summary = _summary(best)
    (output_dir / "summary.txt").write_text(summary)
    print(summary); print(f"Results: {output_dir}")


if __name__ == "__main__":
    main()
