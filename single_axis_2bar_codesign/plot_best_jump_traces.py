"""Replay retained best designs and plot their jump telemetry.

The optimization output stores the best design but not a per-step state log.
This utility reconstructs the original Authors-style controller, replays the
saved XML/design pair, and writes a CSV trace plus a six-panel PNG for each
requested run directory.
"""

from __future__ import annotations

import argparse
import csv
from dataclasses import dataclass
import json
from pathlib import Path

import matplotlib.pyplot as plt
import mujoco as mj
import numpy as np

from kinematics import saturated_applied_torque
from model import Actuator, Design
from simulate import foot_contact_force, vmc_joint_torques


@dataclass(frozen=True)
class Trace:
    time: np.ndarray
    hip_torque: np.ndarray
    knee_torque: np.ndarray
    hip_angle: np.ndarray
    knee_angle: np.ndarray
    base_z: np.ndarray
    hip_velocity: np.ndarray
    knee_velocity: np.ndarray
    base_velocity_z: np.ndarray
    contact: np.ndarray


def design_from_record(record: dict[str, object]) -> Design:
    """Build the exact controller design stored with an optimization result."""
    hip_limit = float(record["hip_torque_limit"])
    knee_limit = float(record["knee_torque_limit"])
    hip_efficiency = float(record["hip_efficiency"])
    knee_efficiency = float(record["knee_efficiency"])
    return Design(
        thigh=float(record["l1"]), shank=float(record["l2"]),
        payload_mass=float(record["payload_mass"]),
        hip=Actuator(
            motor=str(record["hip_motor"]), ratio=float(record["hip_gear_ratio"]),
            gearbox=str(record["hip_gearbox"]), mass=float(record["hip_actuator_mass"]),
            efficiency=hip_efficiency, continuous_output_torque=hip_limit / hip_efficiency,
        ),
        knee=Actuator(
            motor=str(record["knee_motor"]), ratio=float(record["knee_gear_ratio"]),
            gearbox=str(record["knee_gearbox"]), mass=float(record["knee_actuator_mass"]),
            efficiency=knee_efficiency, continuous_output_torque=knee_limit / knee_efficiency,
        ),
        theta1=float(record["theta1_rad"]), theta2=float(record["theta2_rad"]),
        ori_l=float(record["ori_l"]), ori_theta=float(record["ori_theta_rad"]),
        K=float(record["K"]), C=float(record["C"]), T_gain=float(record["T_gain"]),
        thrust_time=float(record["thrust_time"]),
    )


def replay(run_dir: Path, duration: float) -> Trace:
    record = json.loads((run_dir / "best_design.json").read_text())
    metadata = json.loads((run_dir / "metadata.json").read_text())
    design = design_from_record(record)
    controller = metadata["controller"]
    if controller not in {"authors_single_jump", "authors_contact_vmc"}:
        raise ValueError(f"{run_dir}: unsupported controller {controller!r}")

    model = mj.MjModel.from_xml_path(str(run_dir / "best_design.xml"))
    data = mj.MjData(model)
    hip = model.joint("hip")
    knee = model.joint("knee")
    slide = model.joint("pole_slide")
    carriage = model.body("carriage").id
    hip_motor = model.actuator("hip_motor").id
    knee_motor = model.actuator("knee_motor").id
    data.qpos[hip.qposadr[0]] = design.theta1
    data.qpos[knee.qposadr[0]] = design.theta2
    mj.mj_forward(model, data)

    samples: list[tuple[float, ...]] = []
    takeoff = False
    single_jump_complete = False
    had_contact = foot_contact_force(model, data) > 1e-4
    while data.time < duration:
        normal_force = foot_contact_force(model, data)
        in_contact = normal_force > 1e-4
        if controller == "authors_single_jump" and takeoff and in_contact:
            single_jump_complete = True
        if controller == "authors_single_jump" and single_jump_complete:
            raw_torque = np.zeros(2)
        elif in_contact:
            raw_torque = vmc_joint_torques(
                design, float(data.qpos[hip.qposadr[0]]), float(data.qpos[knee.qposadr[0]]),
                float(data.qvel[hip.dofadr[0]]), float(data.qvel[knee.dofadr[0]]),
            )
        else:
            raw_torque = np.array([
                100.0 * (design.theta1 - data.qpos[hip.qposadr[0]]) - 10.0 * data.qvel[hip.dofadr[0]],
                100.0 * (design.theta2 - data.qpos[knee.qposadr[0]]) - 10.0 * data.qvel[knee.dofadr[0]],
            ])
        hip_torque = saturated_applied_torque(
            raw_torque[0], design.hip.continuous_output_torque, design.hip.efficiency,
        )
        knee_torque = saturated_applied_torque(
            raw_torque[1], design.knee.continuous_output_torque, design.knee.efficiency,
        )
        samples.append((
            float(data.time), hip_torque, knee_torque,
            float(data.qpos[hip.qposadr[0]]), float(data.qpos[knee.qposadr[0]]),
            float(data.xpos[carriage, 2]), float(data.qvel[hip.dofadr[0]]),
            float(data.qvel[knee.dofadr[0]]), float(data.qvel[slide.dofadr[0]]), float(in_contact),
        ))
        data.ctrl[hip_motor] = hip_torque
        data.ctrl[knee_motor] = knee_torque
        if had_contact and not in_contact and not takeoff:
            takeoff = True
        had_contact = had_contact or in_contact
        mj.mj_step(model, data)

    values = np.asarray(samples)
    return Trace(*[values[:, index] for index in range(values.shape[1])])


def write_trace(path: Path, trace: Trace) -> None:
    headers = (
        "time_s", "hip_torque_nm", "knee_torque_nm", "hip_angle_rad", "knee_angle_rad",
        "base_z_m", "hip_angular_velocity_rad_s", "knee_angular_velocity_rad_s",
        "base_velocity_z_m_s", "foot_in_contact",
    )
    with path.open("w", newline="") as stream:
        writer = csv.writer(stream)
        writer.writerow(headers)
        writer.writerows(zip(*[getattr(trace, field) for field in Trace.__dataclass_fields__]))


def plot_trace(path: Path, label: str, trace: Trace) -> None:
    figure, axes = plt.subplots(3, 2, figsize=(13, 10), sharex=True, layout="constrained")
    figure.suptitle(f"{label}: replayed best-design jump telemetry")
    panels = (
        (axes[0, 0], ((trace.hip_torque, "Hip"), (trace.knee_torque, "Knee")), "Applied torque [N m]"),
        (axes[0, 1], ((trace.hip_angle, "Hip"), (trace.knee_angle, "Knee")), "Joint angle [rad]"),
        (axes[1, 0], ((trace.base_z, "Base z"),), "Base position z [m]"),
        (axes[1, 1], ((trace.hip_velocity, "Hip"), (trace.knee_velocity, "Knee")), "Angular velocity [rad/s]"),
        (axes[2, 0], ((trace.base_velocity_z, "Base z"),), "Base velocity z [m/s]"),
    )
    for axis, curves, ylabel in panels:
        for values, curve_label in curves:
            axis.plot(trace.time, values, label=curve_label, linewidth=1.25)
        axis.set_ylabel(ylabel)
        axis.grid(alpha=0.3)
        axis.legend(loc="best")
    axes[2, 1].step(trace.time, trace.contact, where="post", color="tab:green", linewidth=1.1)
    axes[2, 1].set_ylabel("Foot contact")
    axes[2, 1].set_yticks((0, 1), ("flight", "contact"))
    axes[2, 1].grid(alpha=0.3)
    for axis in axes[2, :]:
        axis.set_xlabel("Time [s]")
    figure.savefig(path, dpi=180)
    plt.close(figure)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("run_dirs", nargs="+", type=Path)
    parser.add_argument("--duration", type=float, default=None, help="Override the saved simulation duration [s].")
    args = parser.parse_args()
    for run_dir in args.run_dirs:
        metadata = json.loads((run_dir / "metadata.json").read_text())
        duration = args.duration if args.duration is not None else float(metadata["simulation_duration_s"])
        trace = replay(run_dir, duration)
        write_trace(run_dir / "best_jump_telemetry.csv", trace)
        plot_trace(run_dir / "best_jump_telemetry.png", run_dir.name, trace)
        print(f"Wrote {run_dir / 'best_jump_telemetry.csv'}")
        print(f"Wrote {run_dir / 'best_jump_telemetry.png'}")


if __name__ == "__main__":
    main()
