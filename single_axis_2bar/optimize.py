"""CMA-ES search for the vertical, single-leg two-bar jumper."""

from __future__ import annotations

import argparse
import csv
from datetime import datetime
from pathlib import Path
import uuid

import cma
import numpy as np

from model import Design, MOTORS, actuator_for, write_xml
from simulate import run_jump

HERE = Path(__file__).resolve().parent
GENERATED = HERE / "generated"
RESULTS = HERE / "results"

# thigh, shank, payload, hip motor ID, knee motor ID, hip ratio, knee ratio,
# crouch fraction, thrust time, hip Kp, knee Kp
BOUNDS = np.array([
    [0.15, 0.35], [0.15, 0.35], [0.25, 3.0], [1.0, 6.0], [1.0, 6.0],
    [4.0, 25.0], [4.0, 25.0], [0.50, 0.85], [0.04, 0.25], [30.0, 800.0], [30.0, 800.0],
])


def denormalize(vector: np.ndarray) -> np.ndarray:
    vector = np.clip(vector, 0.0, 1.0)
    return BOUNDS[:, 0] + vector * (BOUNDS[:, 1] - BOUNDS[:, 0])


def motor_name(value: float) -> str:
    return MOTORS[int(np.clip(round(value), 1, len(MOTORS))) - 1]


def design_from_vector(vector: np.ndarray) -> Design:
    p = denormalize(vector)
    return Design(
        thigh=float(p[0]), shank=float(p[1]), payload_mass=float(p[2]),
        hip=actuator_for(motor_name(p[3]), float(p[5])),
        knee=actuator_for(motor_name(p[4]), float(p[6])),
        crouch_fraction=float(p[7]), thrust_time=float(p[8]), hip_kp=float(p[9]), knee_kp=float(p[10]),
    )


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--iterations", type=int, default=100)
    parser.add_argument("--population", type=int, default=8)
    parser.add_argument("--seed", type=int, default=5)
    parser.add_argument("--energy-weight", type=float, default=0.015)
    parser.add_argument("--mass-weight", type=float, default=0.05)
    args = parser.parse_args()
    GENERATED.mkdir(parents=True, exist_ok=True)
    RESULTS.mkdir(parents=True, exist_ok=True)
    stamp = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
    output = RESULTS / f"samples_{stamp}_seed{args.seed}.csv"

    headers = [
        "id", "cost", "apex_gain_m", "apex_height_m", "takeoff", "positive_work_j", "peak_power_w", "peak_force_n",
        "thigh_m", "shank_m", "payload_kg", "hip_motor", "knee_motor", "hip_ratio", "knee_ratio",
        "hip_gearbox", "knee_gearbox", "actuator_mass_kg", "crouch_fraction", "thrust_time_s", "hip_kp", "knee_kp",
    ]
    with output.open("w", newline="") as stream:
        csv.writer(stream).writerow(headers)

    x0_physical = np.array([0.25, 0.25, 1.0, 1, 1, 10, 10, 0.65, 0.12, 300, 300])
    x0 = (x0_physical - BOUNDS[:, 0]) / (BOUNDS[:, 1] - BOUNDS[:, 0])
    es = cma.CMAEvolutionStrategy(x0, 0.18, {
        "bounds": [np.zeros(len(BOUNDS)), np.ones(len(BOUNDS))],
        "maxiter": args.iterations, "popsize": args.population, "seed": args.seed, "verb_disp": 1,
    })
    best_cost = float("inf")

    while not es.stop():
        samples = es.ask()
        costs: list[float] = []
        rows = []
        for sample in samples:
            uid = uuid.uuid4().hex[:8]
            design = design_from_vector(np.asarray(sample))
            xml_path = GENERATED / f"{uid}.xml"
            try:
                write_xml(design, xml_path)
                result = run_jump(xml_path, design)
                mass = design.hip.mass + design.knee.mass
                cost = -result.apex_gain + args.energy_weight * result.positive_work + args.mass_weight * mass
                if not result.takeoff:
                    cost += 100.0
            except Exception as exc:
                print(f"Candidate {uid} failed: {exc}")
                result = None
                mass = design.hip.mass + design.knee.mass
                cost = 1e6
            costs.append(float(cost))
            if result is not None:
                rows.append([
                    uid, cost, result.apex_gain, result.apex_height, result.takeoff, result.positive_work,
                    result.peak_power, result.peak_force, design.thigh, design.shank, design.payload_mass,
                    design.hip.motor, design.knee.motor, design.hip.ratio, design.knee.ratio,
                    design.hip.gearbox, design.knee.gearbox, mass, design.crouch_fraction, design.thrust_time,
                    design.hip_kp, design.knee_kp,
                ])
            if cost < best_cost:
                best_cost = cost
                print(f"new best: cost={cost:.4f}, apex gain={result.apex_gain if result else 0:.3f} m, xml={xml_path}")
        with output.open("a", newline="") as stream:
            csv.writer(stream).writerows(rows)
        es.tell(samples, costs)

    print(f"Finished. Samples: {output}")
    print(f"Best cost: {best_cost:.6f}")


if __name__ == "__main__":
    main()
