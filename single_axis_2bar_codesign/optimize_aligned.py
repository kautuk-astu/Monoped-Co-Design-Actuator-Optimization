"""CMA-ES study for controller-enforced vertical toe alignment.

The fixed 0.20 m links retain the repository's existing actuator co-design.
Initial joint angles are derived from one fixed base height, so every actuator
candidate starts from exactly the same vertically aligned pose and payload.
"""
from __future__ import annotations

import argparse
import csv
from dataclasses import asdict
from datetime import datetime
import json
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile
import uuid

import numpy as np
import pandas as pd

from kinematics import vertical_leg_ik
from model import Actuator, CARRIAGE_BASE_MASS_KG, Design, actuator_for, write_xml
from simulate import (
    CONTACT_DEBOUNCE_S, INFEASIBILITY_COST, MIN_FLIGHT_S, JumpResult, run_jump,
)

HERE = Path(__file__).resolve().parent
RESULTS = HERE / "results" / "aligned"
FIXED_L1_M = FIXED_L2_M = 0.20
FOOT_RADIUS_M = 0.025
FIXED_INITIAL_BASE_HEIGHT_M = 0.305
FIXED_CROUCH_LENGTH_M = FIXED_INITIAL_BASE_HEIGHT_M - FOOT_RADIUS_M
FIXED_PAYLOAD_MASS_KG = 1.0
FIXED_HIP_MOTOR = "MAD_M6C12_KV170"
FIXED_KNEE_MOTOR = "MAD_M6C12_KV170"
GEARBOX_TYPES = ("SSPG", "CPG", "WPG", "DSPG")
FIXED_ORI_L_M = 0.38
FIXED_THRUST_TIME_S = 0.15
FIXED_KX_N_M = 300.0
FIXED_DX_N_S_M = 10.0
FLIGHT_LEG_LENGTH_M = 0.24
LANDING_LEG_LENGTH_M = 0.31
PREPARE_TIME_S = 0.05
JOINT_KP = 100.0
JOINT_KD = 10.0

PARAMETER_NAMES = (
    "hip_gear_ratio", "knee_gear_ratio", "K", "C", "T_gain",
)
BOUNDS = np.array([
    [4.4, 6.1], [4.4, 6.1], [100, 1000], [0, 20], [0, 20],
], dtype=float)
DEFAULT_LAMBDA_X = 200.0
DEFAULT_LAMBDA_LANDING = 400.0
DEFAULT_LAMBDA_ENERGY = 0.002
CMA_INITIAL_SIGMA = 0.04


def _git_commit() -> str | None:
    try:
        return subprocess.check_output(
            ["git", "rev-parse", "HEAD"], cwd=HERE.parent, text=True, stderr=subprocess.DEVNULL,
        ).strip()
    except Exception:
        return None


def bounds_for_catalogue(catalogue: Path, gearbox: str) -> np.ndarray:
    """Use the requested-ratio span containing feasible rows for one topology."""
    table = pd.read_csv(catalogue)
    required = {"gearbox_type", "requested_ratio", "feasibility_status"}
    missing = required.difference(table.columns)
    if missing:
        raise ValueError(f"Catalogue {catalogue} is missing columns: {sorted(missing)}")
    rows = table[
        (table["gearbox_type"].str.upper() == gearbox.upper())
        & (table["feasibility_status"].str.lower() == "feasible")
    ].copy()
    ratios = pd.to_numeric(rows["requested_ratio"], errors="coerce").dropna()
    if ratios.empty:
        raise ValueError(f"Catalogue contains no feasible {gearbox} ratios: {catalogue}")
    bounds = BOUNDS.copy()
    bounds[0] = bounds[1] = [float(ratios.min()), float(ratios.max())]
    return bounds


def denormalize(vector: np.ndarray, bounds: np.ndarray = BOUNDS) -> np.ndarray:
    vector = np.asarray(vector, dtype=float)
    if vector.shape != (len(PARAMETER_NAMES),) or not np.isfinite(vector).all():
        raise ValueError(f"normalized vector must be finite and length {len(PARAMETER_NAMES)}")
    if np.any(vector < 0.0) or np.any(vector > 1.0):
        raise ValueError("normalized vector must lie in [0, 1]")
    return bounds[:, 0] + vector * (bounds[:, 1] - bounds[:, 0])


def design_from_vector(
    vector: np.ndarray,
    *,
    gearbox: str = "SSPG",
    actuator_catalogue: Path | None = None,
    motor_config: Path | None = None,
    bounds: np.ndarray = BOUNDS,
) -> Design:
    p = denormalize(vector, bounds)
    q = vertical_leg_ik(FIXED_CROUCH_LENGTH_M, FIXED_L1_M, FIXED_L2_M)
    actuator_kwargs = {"gearbox": gearbox}
    if actuator_catalogue is not None:
        actuator_kwargs["actuator_table"] = actuator_catalogue
    if motor_config is not None:
        actuator_kwargs["motor_config"] = motor_config
    return Design(
        thigh=FIXED_L1_M, shank=FIXED_L2_M, payload_mass=FIXED_PAYLOAD_MASS_KG,
        hip=actuator_for(FIXED_HIP_MOTOR, float(p[0]), **actuator_kwargs),
        knee=actuator_for(FIXED_KNEE_MOTOR, float(p[1]), **actuator_kwargs),
        theta1=float(q[0]), theta2=float(q[1]), ori_l=FIXED_ORI_L_M, ori_theta=0.0,
        K=float(p[2]), C=float(p[3]), T_gain=float(p[4]), thrust_time=FIXED_THRUST_TIME_S,
        Kx=FIXED_KX_N_M, Dx=FIXED_DX_N_S_M, prepare_time=PREPARE_TIME_S,
        flight_leg_length=FLIGHT_LEG_LENGTH_M, landing_leg_length=LANDING_LEG_LENGTH_M,
        joint_kp=JOINT_KP, joint_kd=JOINT_KD,
    )


def design_from_record(record: dict[str, object]) -> Design:
    def actuator(prefix: str) -> Actuator:
        efficiency = float(record[f"{prefix}_efficiency"])
        return Actuator(
            motor=str(record[f"{prefix}_motor"]), ratio=float(record[f"{prefix}_gear_ratio"]),
            gearbox=str(record[f"{prefix}_gearbox"]), mass=float(record[f"{prefix}_actuator_mass"]),
            efficiency=efficiency,
            continuous_output_torque=float(record[f"{prefix}_torque_limit_nm"]) / efficiency,
            kv_rpm_per_volt=float(record[f"{prefix}_kv_rpm_per_volt"]),
            rated_voltage=float(record[f"{prefix}_rated_voltage_v"]),
            resistance_ohm=float(record[f"{prefix}_resistance_ohm"]),
            max_continuous_current=float(record[f"{prefix}_max_continuous_current_a"]),
        )
    return Design(
        thigh=float(record["l1_m"]), shank=float(record["l2_m"]),
        payload_mass=float(record["payload_mass_kg"]), hip=actuator("hip"), knee=actuator("knee"),
        theta1=float(record["theta1_rad"]), theta2=float(record["theta2_rad"]),
        ori_l=float(record["ori_l_m"]), ori_theta=0.0, K=float(record["K_n_m"]),
        C=float(record["C_n_s_m"]), T_gain=float(record.get("T_gain_nm_rad", 0.0)),
        thrust_time=float(record["thrust_time_s"]),
        Kx=float(record["Kx_n_m"]), Dx=float(record["Dx_n_s_m"]),
        prepare_time=float(record["prepare_time_s"]),
        flight_leg_length=float(record["flight_leg_length_m"]),
        landing_leg_length=float(record["landing_leg_length_m"]),
        joint_kp=float(record["joint_kp_nm_rad"]), joint_kd=float(record["joint_kd_nm_s_rad"]),
    )


def objective(
    result: JumpResult,
    lambda_x: float = DEFAULT_LAMBDA_X,
    lambda_landing: float = DEFAULT_LAMBDA_LANDING,
    lambda_energy: float = DEFAULT_LAMBDA_ENERGY,
) -> float:
    """Ballistic height plus alignment and positive-mechanical-work penalties."""
    landing_error = float(result.prelanding_alignment_error or 0.0)
    return float(
        -result.ballistic_height
        + lambda_x * result.alignment_mse
        + lambda_landing * landing_error * landing_error
        + lambda_energy * result.positive_work
    )


def _record(
    candidate_id: str, generation: int, evaluation_index: int, vector: np.ndarray,
    design: Design, result: JumpResult, cost: float, violation: str | None,
) -> dict[str, object]:
    return {
        "candidate_id": candidate_id, "generation": generation, "evaluation_index": evaluation_index,
        "normalized_vector_json": json.dumps(np.asarray(vector, dtype=float).tolist()),
        "cost": cost, "valid": violation is None, "constraint_violation_reason": violation or "",
        "ballistic_height_m": result.ballistic_height,
        "crouch_to_apex_height_m": result.crouch_to_apex_height,
        "takeoff_com_height_m": result.takeoff_com_height,
        "apex_com_height_m": result.apex_com_height,
        "takeoff_time_s": result.takeoff_time, "landing_time_s": result.landing_time,
        "flight_duration_s": result.flight_duration, "qualified_flight": result.qualified_flight,
        "alignment_mse_m2": result.alignment_mse, "alignment_rms_m": result.alignment_rms,
        "max_abs_alignment_error_m": result.maximum_alignment_error,
        "prelanding_alignment_error_m": result.prelanding_alignment_error,
        "positive_mechanical_work_j": result.positive_work,
        "joule_loss_proxy_j": result.joule_energy, "legacy_total_energy_j": result.total_energy,
        "peak_power_w": result.peak_power, "peak_ground_force_n": result.peak_force,
        "hip_saturation_fraction": result.hip_saturation_fraction,
        "knee_saturation_fraction": result.knee_saturation_fraction,
        "peak_hip_motor_current_a": result.peak_hip_motor_current,
        "peak_knee_motor_current_a": result.peak_knee_motor_current,
        "peak_hip_motor_speed_rad_s": result.peak_hip_motor_speed,
        "peak_knee_motor_speed_rad_s": result.peak_knee_motor_speed,
        "minimum_toe_to_rail_clearance_m": result.minimum_toe_to_pole_clearance,
        "minimum_toe_to_hip_clearance_m": result.minimum_hip_to_toe_clearance,
        "minimum_link_to_rail_clearance_m": result.minimum_link_to_body_clearance,
        "l1_m": design.thigh, "l2_m": design.shank,
        "crouch_length_m": float(np.linalg.norm([0.0, -design.thigh * np.cos(design.theta1) - design.shank * np.cos(design.theta1 + design.theta2)])),
        "theta1_rad": design.theta1, "theta2_rad": design.theta2,
        "payload_mass_kg": design.payload_mass,
        "hip_motor": design.hip.motor, "hip_gear_ratio": design.hip.ratio,
        "hip_gearbox": design.hip.gearbox, "hip_efficiency": design.hip.efficiency,
        "hip_actuator_mass": design.hip.mass,
        "hip_torque_limit_nm": design.hip.efficiency * design.hip.continuous_output_torque,
        "hip_kv_rpm_per_volt": design.hip.kv_rpm_per_volt,
        "hip_rated_voltage_v": design.hip.rated_voltage,
        "hip_resistance_ohm": design.hip.resistance_ohm,
        "hip_max_continuous_current_a": design.hip.max_continuous_current,
        "knee_motor": design.knee.motor, "knee_gear_ratio": design.knee.ratio,
        "knee_gearbox": design.knee.gearbox, "knee_efficiency": design.knee.efficiency,
        "knee_actuator_mass": design.knee.mass,
        "knee_torque_limit_nm": design.knee.efficiency * design.knee.continuous_output_torque,
        "knee_kv_rpm_per_volt": design.knee.kv_rpm_per_volt,
        "knee_rated_voltage_v": design.knee.rated_voltage,
        "knee_resistance_ohm": design.knee.resistance_ohm,
        "knee_max_continuous_current_a": design.knee.max_continuous_current,
        "moving_carriage_mass_kg": CARRIAGE_BASE_MASS_KG + design.payload_mass + design.hip.mass + design.knee.mass,
        "ori_l_m": design.ori_l, "K_n_m": design.K, "C_n_s_m": design.C,
        "T_gain_nm_rad": design.T_gain,
        "thrust_time_s": design.thrust_time, "Kx_n_m": design.Kx, "Dx_n_s_m": design.Dx,
        "prepare_time_s": design.prepare_time, "flight_leg_length_m": design.flight_leg_length,
        "landing_leg_length_m": design.landing_leg_length,
        "joint_kp_nm_rad": design.joint_kp, "joint_kd_nm_s_rad": design.joint_kd,
    }


def write_telemetry(output_dir: Path, result: JumpResult) -> None:
    """Write full controller telemetry and the required alignment/phase plot."""
    if not result.telemetry:
        return
    path = output_dir / "best_jump_telemetry.csv"
    headers = list(asdict(result.telemetry[0]))
    with path.open("w", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=headers)
        writer.writeheader()
        for sample in result.telemetry:
            writer.writerow(asdict(sample))

    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    times = np.array([sample.time for sample in result.telemetry])
    errors = np.array([sample.alignment_error for sample in result.telemetry])
    phases = [sample.phase for sample in result.telemetry]
    colors = {"PREPARE": "#d9d9d9", "STANCE": "#ffd27f", "FLIGHT": "#9ecae1", "LANDING": "#a1d99b"}
    figure, axis = plt.subplots(figsize=(10, 4), layout="constrained")
    start = 0
    for index in range(1, len(phases) + 1):
        if index == len(phases) or phases[index] != phases[start]:
            axis.axvspan(times[start], times[min(index, len(times) - 1)], color=colors[phases[start]], alpha=0.35, label=phases[start])
            start = index
    axis.plot(times, errors * 1000.0, color="black", linewidth=1.2, label="$x_{toe}-x_{hip}$")
    axis.axhline(0.0, color="black", linewidth=0.6)
    for sample in result.telemetry:
        if sample.takeoff_event or sample.apex_event or sample.landing_event:
            label = "takeoff" if sample.takeoff_event else "apex" if sample.apex_event else "landing"
            axis.axvline(sample.time, linestyle="--", linewidth=0.8, label=label)
    axis.set(xlabel="Time [s]", ylabel="Toe–hip horizontal error [mm]")
    handles, labels = axis.get_legend_handles_labels()
    unique = dict(zip(labels, handles))
    axis.legend(unique.values(), unique.keys(), ncol=4, fontsize=8)
    axis.grid(alpha=0.25)
    figure.savefig(output_dir / "alignment_error.png", dpi=180)
    plt.close(figure)

    figure, axes = plt.subplots(1, 2, figsize=(11, 4), layout="constrained")
    hip_z = np.array([sample.hip_z for sample in result.telemetry])
    toe_z = np.array([sample.toe_z for sample in result.telemetry])
    com_z = np.array([sample.com_z for sample in result.telemetry])
    axes[0].plot(times, hip_z, label="hip")
    axes[0].plot(times, toe_z, label="toe")
    axes[0].plot(times, com_z, label="COM")
    axes[0].set(xlabel="Time [s]", ylabel="World z [m]", title="First-jump trajectory")
    axes[0].legend()
    axes[1].plot(errors * 1000.0, (toe_z - hip_z) * 1000.0, linewidth=1.2)
    axes[1].scatter([errors[0] * 1000.0], [(toe_z[0] - hip_z[0]) * 1000.0], label="start", s=25)
    axes[1].set(xlabel="Toe−hip x [mm]", ylabel="Toe−hip z [mm]", title="Toe path relative to hip")
    axes[1].legend()
    for axis in axes:
        axis.grid(alpha=0.25)
    figure.savefig(output_dir / "jump_trajectory.png", dpi=180)
    plt.close(figure)


def replay_record(record: dict[str, object], output_dir: Path, duration: float) -> JumpResult:
    design = design_from_record(record)
    xml_path = output_dir / "best_design.xml"
    write_xml(design, xml_path)
    result = run_jump(xml_path, design, duration=duration, controller_mode="aligned_jump", record=True)
    write_telemetry(output_dir, result)
    metrics = {key: value for key, value in asdict(result).items() if key != "telemetry"}
    (output_dir / "best_validation.json").write_text(json.dumps(metrics, indent=2, default=float))
    return result


def write_compact_log(
    output_dir: Path, best: dict[str, object] | None, result: JumpResult | None,
    *, seed: int, popsize: int, maxiter: int, simulation_duration: float,
    lambda_x: float, lambda_landing: float, lambda_energy: float,
    bounds: np.ndarray = BOUNDS,
) -> None:
    """Write the small review-facing log; detailed diagnostics stay separate."""
    variables = None if best is None else {
        "hip_gear_ratio": float(best["hip_gear_ratio"]),
        "knee_gear_ratio": float(best["knee_gear_ratio"]),
        "K_n_per_m": float(best["K_n_m"]),
        "C_n_s_per_m": float(best["C_n_s_m"]),
        "T_gain_n_m_per_rad": float(best["T_gain_nm_rad"]),
    }
    key_results: dict[str, object] = {"valid_jump": False}
    if best is not None and result is not None:
        takeoff_sample = next((sample for sample in result.telemetry if sample.takeoff_event), None)
        takeoff_base_height = None if takeoff_sample is None else float(takeoff_sample.hip_z)
        maximum_base_height = (
            max(float(sample.hip_z) for sample in result.telemetry)
            if result.telemetry else float(result.apex_height)
        )
        key_results = {
            "valid_jump": bool(result.qualified_flight and result.constraint_violation is None),
            "cost": float(best["cost"]),
            "initial_base_height_m": float(result.initial_height),
            "takeoff_base_height_m": takeoff_base_height,
            "maximum_base_height_m": maximum_base_height,
            "total_base_rise_m": maximum_base_height - float(result.initial_height),
            "base_rise_after_toe_leaves_ground_m": (
                None if takeoff_base_height is None else maximum_base_height - takeoff_base_height
            ),
            "toe_off_ground_duration_s": float(result.flight_duration),
            "toe_horizontal_error_rms_m": float(result.alignment_rms),
            "toe_horizontal_error_max_m": float(result.maximum_alignment_error),
            "toe_horizontal_error_before_landing_m": (
                None if result.prelanding_alignment_error is None
                else float(result.prelanding_alignment_error)
            ),
            "positive_mechanical_work_j": float(result.positive_work),
            "hip_torque_saturation_fraction": float(result.hip_saturation_fraction),
            "knee_torque_saturation_fraction": float(result.knee_saturation_fraction),
        }
    compact_log = {
        "optimization_variables": variables,
        "hyperparameters": {
            "bounds": dict(zip(PARAMETER_NAMES, bounds.tolist())),
            "seed": seed,
            "population_size": popsize,
            "generations": maxiter,
            "total_candidate_evaluations": popsize * maxiter,
            "simulation_duration_s": simulation_duration,
            "cma_initial_sigma_normalized": CMA_INITIAL_SIGMA,
            "cost_weights": {
                "toe_horizontal_error": lambda_x,
                "toe_error_before_landing": lambda_landing,
                "positive_mechanical_work": lambda_energy,
            },
        },
        "key_results": key_results,
    }
    (output_dir / "compact_optimization_log.json").write_text(
        json.dumps(compact_log, indent=2, allow_nan=False) + "\n"
    )


def main() -> None:
    import cma
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--maxiter", "--iterations", dest="maxiter", type=int, default=100)
    parser.add_argument("--popsize", "--population", dest="popsize", type=int, default=8)
    parser.add_argument("--seed", type=int, default=5)
    parser.add_argument("--simulation-duration", type=float, default=1.0)
    parser.add_argument("--lambda-x", type=float, default=DEFAULT_LAMBDA_X)
    parser.add_argument("--lambda-landing", type=float, default=DEFAULT_LAMBDA_LANDING)
    parser.add_argument("--lambda-energy", type=float, default=DEFAULT_LAMBDA_ENERGY)
    parser.add_argument("--gearbox", required=True, choices=GEARBOX_TYPES)
    parser.add_argument("--actuator-catalogue", required=True, type=Path)
    parser.add_argument("--motor-config", required=True, type=Path,
                        help="COMPAct config_files/motors.json")
    parser.add_argument("--output-dir", type=Path)
    args = parser.parse_args()
    if not args.actuator_catalogue.is_file():
        parser.error(f"actuator catalogue does not exist: {args.actuator_catalogue}")
    if not args.motor_config.is_file():
        parser.error(f"motor configuration does not exist: {args.motor_config}")
    bounds = bounds_for_catalogue(args.actuator_catalogue, args.gearbox)
    output_dir = args.output_dir or RESULTS / (datetime.now().strftime("run_%Y-%m-%d_%H-%M-%S") + f"_seed{args.seed}")
    output_dir.mkdir(parents=True, exist_ok=True)
    metadata = {
        "timestamp": datetime.now().isoformat(), "git_commit": _git_commit(), "python_version": sys.version,
        "controller": "aligned_jump", "seed": args.seed, "popsize": args.popsize,
        "maxiter": args.maxiter, "simulation_duration_s": args.simulation_duration,
        "parameter_names": PARAMETER_NAMES, "bounds": dict(zip(PARAMETER_NAMES, bounds.tolist())),
        "gearbox": args.gearbox,
        "actuator_catalogue": str(args.actuator_catalogue.resolve()),
        "motor_config": str(args.motor_config.resolve()),
        "fixed_parameters": {"l1_m": FIXED_L1_M, "l2_m": FIXED_L2_M,
                             "initial_base_height_m": FIXED_INITIAL_BASE_HEIGHT_M,
                             "crouch_length_m": FIXED_CROUCH_LENGTH_M,
                             "payload_mass_kg": FIXED_PAYLOAD_MASS_KG,
                             "hip_motor": FIXED_HIP_MOTOR, "knee_motor": FIXED_KNEE_MOTOR,
                             "hip_gearbox": args.gearbox, "knee_gearbox": args.gearbox,
                             "ori_l_m": FIXED_ORI_L_M, "thrust_time_s": FIXED_THRUST_TIME_S,
                             "Kx_n_m": FIXED_KX_N_M, "Dx_n_s_m": FIXED_DX_N_S_M,
                             "flight_leg_length_m": FLIGHT_LEG_LENGTH_M,
                             "landing_leg_length_m": LANDING_LEG_LENGTH_M,
                             "prepare_time_s": PREPARE_TIME_S, "joint_kp": JOINT_KP, "joint_kd": JOINT_KD,
                             "contact_debounce_s": CONTACT_DEBOUNCE_S, "minimum_flight_s": MIN_FLIGHT_S},
        "objective": "-ballistic_height + lambda_x*MSE(ex) + lambda_landing*ex_prelanding^2 + lambda_energy*positive_mechanical_work",
        "penalties": {"lambda_x": args.lambda_x, "lambda_landing": args.lambda_landing, "lambda_energy": args.lambda_energy},
        "energy_note": "Objective E is existing non-regenerative positive joint mechanical work [J]. joule_loss_proxy_j remains the legacy unverified proxy.",
    }
    try:
        import mujoco
        metadata["mujoco_version"] = mujoco.__version__
        metadata["cma_version"] = cma.__version__
    except Exception:
        metadata["mujoco_version"] = metadata["cma_version"] = None
    (output_dir / "metadata.json").write_text(json.dumps(metadata, indent=2))

    initial_ratio = float(np.mean(bounds[0]))
    x0_physical = np.array([initial_ratio, initial_ratio, 800, 8, 10])
    x0 = (x0_physical - bounds[:, 0]) / (bounds[:, 1] - bounds[:, 0])
    es = cma.CMAEvolutionStrategy(x0, CMA_INITIAL_SIGMA, {
        "bounds": [np.zeros(len(PARAMETER_NAMES)), np.ones(len(PARAMETER_NAMES))],
        "maxiter": args.maxiter, "popsize": args.popsize, "seed": args.seed,
        "verb_disp": 1, "tolflatfitness": args.maxiter + 1,
    })
    best: dict[str, object] | None = None
    evaluation_index = 0
    with tempfile.TemporaryDirectory(prefix="aligned_monoped_", dir=output_dir) as temporary, \
            (output_dir / "all_candidates.csv").open("w", newline="") as all_stream, \
            (output_dir / "generation_best.csv").open("w", newline="") as generation_stream:
        candidate_xml = Path(temporary) / "candidate.xml"
        all_writer = generation_writer = None
        for generation in range(args.maxiter):
            samples, costs, records = es.ask(), [], []
            for sample in samples:
                evaluation_index += 1
                candidate_id = uuid.uuid4().hex[:10]
                design = design_from_vector(
                    np.asarray(sample), gearbox=args.gearbox,
                    actuator_catalogue=args.actuator_catalogue,
                    motor_config=args.motor_config, bounds=bounds,
                )
                try:
                    write_xml(design, candidate_xml)
                    result = run_jump(candidate_xml, design, duration=args.simulation_duration, controller_mode="aligned_jump")
                    violation = result.constraint_violation
                    if violation is None and not result.qualified_flight:
                        violation = "no completed qualified first flight"
                except Exception as exc:
                    result = JumpResult(0, 0, 0, False, None, 0, 0, 0, 0, 0, float("nan"), float("nan"), float("nan"), f"evaluation error: {exc}")
                    violation = result.constraint_violation
                cost = objective(result, args.lambda_x, args.lambda_landing, args.lambda_energy) if violation is None else INFEASIBILITY_COST
                record = _record(candidate_id, generation, evaluation_index, np.asarray(sample), design, result, float(cost), violation)
                if all_writer is None:
                    all_writer = csv.DictWriter(all_stream, fieldnames=list(record)); all_writer.writeheader()
                    generation_writer = csv.DictWriter(generation_stream, fieldnames=list(record)); generation_writer.writeheader()
                all_writer.writerow(record); all_stream.flush()
                records.append(record); costs.append(float(cost))
                if violation is None and (best is None or float(cost) < float(best["cost"])):
                    best = record.copy()
                    shutil.copy2(candidate_xml, output_dir / "best_design.xml")
                    (output_dir / "best_design.json").write_text(json.dumps(best, indent=2, default=float))
                    print(
                        f"new global best: cost={cost:.6f}, "
                        f"total base rise={result.apex_gain:.4f} m, "
                        f"toe alignment rms={result.alignment_rms*1000:.2f} mm"
                    )
            valid_records = [record for record in records if record["valid"]]
            if valid_records:
                generation_writer.writerow(min(valid_records, key=lambda item: float(item["cost"])))
                generation_stream.flush()
            es.tell(samples, costs)

    if best is None:
        (output_dir / "best_design.json").write_text(json.dumps({"best_valid_candidate": None}, indent=2))
        summary = "No completed qualified first flight was found; see all_candidates.csv.\n"
        replay = None
    else:
        replay = replay_record(best, output_dir, args.simulation_duration)
        maximum_base_height = (
            max(float(sample.hip_z) for sample in replay.telemetry)
            if replay.telemetry else replay.apex_height
        )
        summary = (
            f"Initial base height: {replay.initial_height:.6f} m\n"
            f"Maximum base height: {maximum_base_height:.6f} m\n"
            f"Total base rise: {maximum_base_height - replay.initial_height:.6f} m\n"
            f"Toe alignment RMS / maximum error: {replay.alignment_rms:.6f} / {replay.maximum_alignment_error:.6f} m\n"
            f"Toe alignment error before landing: {replay.prelanding_alignment_error:.6f} m\n"
            f"Positive mechanical work: {replay.positive_work:.6f} J\n"
            f"Valid jump: {replay.qualified_flight}; toe off-ground duration: {replay.flight_duration:.6f} s\n"
            f"Cost: {float(best['cost']):.6f}\n"
        )
    write_compact_log(
        output_dir, best, replay, seed=args.seed, popsize=args.popsize, maxiter=args.maxiter,
        simulation_duration=args.simulation_duration, lambda_x=args.lambda_x,
        lambda_landing=args.lambda_landing, lambda_energy=args.lambda_energy,
        bounds=bounds,
    )
    (output_dir / "summary.txt").write_text(summary)
    print(summary)
    print(f"Results: {output_dir}")


if __name__ == "__main__":
    main()
