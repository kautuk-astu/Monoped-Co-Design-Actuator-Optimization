#!/usr/bin/env python3

"""
STAGE 4: COMPAct + MuJoCo fixed-gearbox PD optimisation.

Configurations:
B: WPG hip + WPG knee
C: DSPG hip + WPG knee

Gearbox ratios, masses and efficiencies are loaded
from actual COMPAct CSV outputs.

Optimisation variables:
    hip_Kp, hip_Kd, knee_Kp, knee_Kd
"""

import argparse
import csv
import json
from pathlib import Path

import numpy as np
import cma

from jump_engine import evaluate, CSV_COLUMNS


ROOT = Path(__file__).resolve().parent
CSV_DIR = ROOT / "results_stage3_compact" / "final_csv"

GAIN_NAMES = [
    "hip_Kp",
    "hip_Kd",
    "knee_Kp",
    "knee_Kd",
]

LOW = np.array([20.0, 0.1, 20.0, 0.1])
HIGH = np.array([900.0, 30.0, 900.0, 30.0])

EXPECTED = {
    "B": {
        "hip": 24.0,
        "knee": 16.96969696969697,
    },
    "C": {
        "hip": 18.81818181818182,
        "knee": 20.0,
    },
}


def load_compact_csv(path):
    """Read the final gearbox-design row from COMPAct."""

    with open(path, newline="") as f:
        rows = list(csv.reader(f))

    for i, row in enumerate(rows):
        if row and row[0].strip() == "iter":

            headers = [x.strip() for x in row]

            if i + 1 >= len(rows):
                raise ValueError(f"No design row in {path}")

            values = [x.strip() for x in rows[i + 1]]

            if len(values) != len(headers):
                raise ValueError(
                    f"Column mismatch in {path}"
                )

            result = dict(zip(headers, values))

            return {
                "ratio": float(result["gearRatio"]),
                "mass": float(result["mass"]),
                "efficiency": float(result["eff"]),
                "width_mm": float(result["Actuator_width"]),
                "peak_torque_Nm": float(result["peakTorque"]),
            }

    raise ValueError(f"COMPAct design header not found: {path}")


def load_configuration(name):

    hip = load_compact_csv(CSV_DIR / f"{name}_hip.csv")
    knee = load_compact_csv(CSV_DIR / f"{name}_knee.csv")

    for joint, gearbox in [("hip", hip), ("knee", knee)]:

        ratio = gearbox["ratio"]

        if not 4.0 <= ratio <= 25.0:
            raise ValueError(
                f"{name} {joint}: ratio outside 4:1–25:1"
            )

        expected = EXPECTED[name][joint]

        if abs(ratio - expected) > 0.01:
            raise ValueError(
                f"{name} {joint}: unexpected ratio {ratio}. "
                f"Expected approximately {expected}. "
                "Check whether the CSV was overwritten."
            )

        if not 0.0 < gearbox["efficiency"] <= 1.0:
            raise ValueError("Invalid gearbox efficiency")

        if gearbox["mass"] <= 0:
            raise ValueError("Invalid actuator mass")

    motor = json.loads(
        (ROOT / "motor_config.json").read_text()
    )

    cfg = dict(motor)

    cfg["gearbox_efficiencies"] = [
        hip["efficiency"],
        knee["efficiency"],
    ]

    cfg["actuator_masses_kg"] = [
        hip["mass"],
        knee["mass"],
    ]

    ratios = [hip["ratio"], knee["ratio"]]

    return cfg, ratios, hip, knee


def feasible(metrics, cfg):

    return (
        metrics.get("valid_jump", False)
        and metrics.get("overspeed_time_s", 1e9) <= 1e-6
        and metrics.get("peak_motor_speed_rad_s", 1e9)
            <= cfg["max_speed_rad_s"] + 1e-6
        and metrics.get("peak_current_A", 1e9)
            <= cfg["max_current_A"] + 1e-6
        and metrics.get("joint_limit_duration_s", 1e9)
            <= 1e-6
    )


def objective(metrics, cfg):

    height = metrics.get("com_rise_m", 0.0)
    energy = metrics.get("energy_proxy_J", 1e6)

    score = height - 0.00005 * energy

    if feasible(metrics, cfg):
        return 10.0 + score

    speed = metrics.get("peak_motor_speed_rad_s", 1e6)

    overspeed = max(
        0.0,
        speed - cfg["max_speed_rad_s"]
    )

    return (
        score
        - 0.3
        - 0.03 * overspeed / cfg["max_speed_rad_s"]
        - 0.5 * metrics.get("overspeed_time_s", 0.0)
        - metrics.get("joint_limit_duration_s", 1.0)
    )


def optimise(name, args):

    cfg, ratios, hip, knee = load_configuration(name)

    print(f"\n{'=' * 60}")
    print(f"CONFIGURATION {name}")
    print(f"{'=' * 60}")

    print("Hip ratio:", ratios[0])
    print("Knee ratio:", ratios[1])

    print("Hip efficiency:", hip["efficiency"])
    print("Knee efficiency:", knee["efficiency"])

    total_actuator_mass = hip["mass"] + knee["mass"]

    print("Actuator mass:", total_actuator_mass, "kg")
    print(
        "Estimated robot mass:",
        0.710 + total_actuator_mass,
        "kg"
    )

    def decode(y):
        return LOW + np.clip(y, 0, 1) * (HIGH - LOW)

    def simulate(gains, record=False):

        x = np.array([
            ratios[0],
            ratios[1],
            gains[0],
            gains[1],
            gains[2],
            gains[3],
        ])

        return evaluate(
            x,
            cfg,
            duration=args.duration,
            record=record
        )

    initial_gains = np.array([
        400.0,
        18.0,
        600.0,
        12.0,
    ])

    center = np.clip(
        (initial_gains - LOW) / (HIGH - LOW),
        0.02,
        0.98
    )

    es = cma.CMAEvolutionStrategy(
        center,
        0.17,
        {
            "bounds": [0, 1],
            "seed": args.seed,
            "popsize": args.population,
            "verbose": -9,
        }
    )

    best_valid = None
    best_any = None
    history = []

    def consider(gains, metrics):

        nonlocal best_valid, best_any

        rank = objective(metrics, cfg)

        item = (
            rank,
            gains.copy(),
            metrics,
        )

        if best_any is None or rank > best_any[0]:
            best_any = item

        if feasible(metrics, cfg):
            if best_valid is None or rank > best_valid[0]:
                best_valid = item

        return rank

    _, metrics, _ = simulate(initial_gains)
    consider(initial_gains, metrics)

    for generation in range(1, args.generations + 1):

        solutions = es.ask()
        losses = []

        for solution in solutions:

            gains = decode(np.asarray(solution))

            _, metrics, _ = simulate(gains)

            rank = consider(gains, metrics)

            losses.append(-rank)

        es.tell(solutions, losses)

        if best_valid is not None:
            height = best_valid[2]["com_rise_m"]
            status = f"{height * 1000:.2f} mm"
        else:
            status = "No strictly feasible jump yet"

        print(
            f"Generation {generation}/{args.generations}: "
            f"{status}",
            flush=True
        )

        history.append({
            "generation": generation,
            "best_feasible_rise_m": (
                best_valid[2]["com_rise_m"]
                if best_valid is not None
                else None
            ),
        })

    selected = best_valid if best_valid is not None else best_any

    _, final_metrics, trajectory = simulate(
        selected[1],
        record=True
    )

    result = {
        "configuration": name,
        "strictly_feasible": feasible(final_metrics, cfg),
        "gearboxes": {
            "hip": hip,
            "knee": knee,
        },
        "actuator_mass_total_kg": total_actuator_mass,
        "estimated_robot_mass_kg": (
            0.710 + total_actuator_mass
        ),
        "pd_gains": dict(
            zip(
                GAIN_NAMES,
                map(float, selected[1])
            )
        ),
        "metrics": final_metrics,
        "limitations": [
            "Motor electrical constants not independently verified",
            "Gearbox rotational inertia not fully modelled",
            "Remote knee transmission not explicitly modelled",
            "Actuator mass assumed to include motor",
            "Energy result is a proxy",
            "Optimisation is local and not proof of global optimum",
        ],
    }

    output = ROOT / args.output
    output.mkdir(parents=True, exist_ok=True)

    (output / f"{name}_best.json").write_text(
        json.dumps(result, indent=2)
    )

    with (output / f"{name}_trajectory.csv").open(
        "w", newline=""
    ) as f:
        writer = csv.writer(f)
        writer.writerow(CSV_COLUMNS)
        writer.writerows(trajectory)

    with (output / f"{name}_convergence.csv").open(
        "w", newline=""
    ) as f:
        writer = csv.DictWriter(
            f,
            fieldnames=[
                "generation",
                "best_feasible_rise_m",
            ]
        )
        writer.writeheader()
        writer.writerows(history)

    print("\nFINAL RESULT")
    print("Feasible:", result["strictly_feasible"])
    print("PD gains:", result["pd_gains"])
    print(
        "COM rise:",
        final_metrics.get("com_rise_m", 0) * 1000,
        "mm"
    )
    print(
        "Peak current:",
        final_metrics.get("peak_current_A"),
        "A"
    )
    print(
        "Peak motor speed:",
        final_metrics.get("peak_motor_speed_rad_s"),
        "rad/s"
    )

    return result


def main():

    parser = argparse.ArgumentParser()

    parser.add_argument(
        "--candidate",
        choices=["B", "C", "all"],
        default="all"
    )

    parser.add_argument(
        "--generations",
        type=int,
        default=25
    )

    parser.add_argument(
        "--population",
        type=int,
        default=12
    )

    parser.add_argument(
        "--duration",
        type=float,
        default=1.4
    )

    parser.add_argument(
        "--seed",
        type=int,
        default=42
    )

    parser.add_argument(
        "--output",
        default="results_stage4"
    )

    args = parser.parse_args()

    if args.generations < 1 or args.population < 4:
        parser.error("Use generations >= 1 and population >= 4")

    candidates = (
        ["B", "C"]
        if args.candidate == "all"
        else [args.candidate]
    )

    results = []

    for name in candidates:
        results.append(optimise(name, args))

    print("\n" + "=" * 60)
    print("STAGE 4 COMPARISON")
    print("=" * 60)

    for result in results:

        metrics = result["metrics"]

        print(
            result["configuration"],
            "feasible:",
            result["strictly_feasible"],
            "COM rise:",
            round(metrics.get("com_rise_m", 0) * 1000, 2),
            "mm",
            "robot mass:",
            result["estimated_robot_mass_kg"],
            "kg"
        )


if __name__ == "__main__":
    main()
