#!/usr/bin/env python3
"""Run a saved monoped design without starting a CMA-ES optimization.

The four paper cases use the selected rows in summary_*.json.  The
controller-fixed ablation has no summary JSON, so its selected row is the
minimum-cost row saved by cmaes_ctrlfixed.py in its all_*.csv outputs.
"""

from __future__ import annotations

import argparse
import csv
import contextlib
import json
import os
import sys
import xml.etree.ElementTree as ET
from dataclasses import dataclass
from pathlib import Path
from typing import Any

# MuJoCo must see this before it is imported by opt_codesign_5bar.  EGL is the
# normal headless backend; an explicitly selected user backend is preserved.
os.environ.setdefault("MUJOCO_GL", "egl")

ROOT = Path(__file__).resolve().parents[1]
RESULTS = ROOT / "results"
PARAMS = RESULTS / "Opt_design_control_parameters"
XMLS = ROOT / "xmls"


@dataclass(frozen=True)
class CaseSpec:
    key: str
    label: str
    source: Path | None
    xml_directory: Path
    optimizer: str


CASES: dict[str, CaseSpec] = {
    "nominal": CaseSpec(
        "nominal", "Nominal", PARAMS / "summary_nominal.json", XMLS / "Nominal_xmls",
        "components/cmaes_baseline.py",
    ),
    "casea": CaseSpec(
        "caseA", "Case A", PARAMS / "summary_CaseA.json", XMLS / "Case_A_xmls",
        "components/cmaes_ll.py",
    ),
    "caseb": CaseSpec(
        "caseB", "Case B", PARAMS / "summary_CaseB.json", XMLS / "Case_B_xmls",
        "components/cmaes_gear.py",
    ),
    "casec": CaseSpec(
        "caseC", "Case C", PARAMS / "summary_CaseC.json", XMLS / "design_xmls",
        "components/cmaes.py",
    ),
    "controller-fixed": CaseSpec(
        "controller-fixed", "Controller fixed", None, XMLS / "only_design_xmls",
        "components/cmaes_ctrlfixed.py",
    ),
}
PAPER_CASES = ("nominal", "casea", "caseb", "casec")
ALL_CASES = PAPER_CASES + ("controller-fixed",)


def require_packages(include_cma: bool = False) -> None:
    names = ["numpy", "pandas", "scipy", "mujoco", "imageio"]
    if include_cma:
        names.append("cma")
    missing: list[str] = []
    for name in names:
        try:
            __import__(name)
        except ImportError:
            missing.append(name)
    if missing:
        raise RuntimeError(
            "The active Python environment is missing: " + ", ".join(missing)
            + ". Activate the repository's simulation environment and rerun; "
            "this launcher does not create or modify environments."
        )


def value(row: dict[str, Any], *names: str) -> Any:
    for name in names:
        if name in row and row[name] not in (None, ""):
            return row[name]
    raise KeyError(f"Missing one of {names} in saved configuration")


def paper_row(source: Path) -> dict[str, Any]:
    with source.open() as stream:
        saved = json.load(stream)
    for name in ("secondary", "all_row_for_best_min", "all_min_row"):
        row = saved.get(name)
        if row is not None:
            return dict(row)
    raise ValueError(f"{source} does not contain a selected saved configuration")


def controller_fixed_row() -> tuple[dict[str, Any], Path]:
    """Select the exact minimum-cost sampled configuration from the ablation."""
    candidates: list[tuple[float, dict[str, str], Path]] = []
    for source in sorted((RESULTS / "CMAES_output" / "only_design").glob("all_*.csv")):
        with source.open(newline="") as stream:
            for row in csv.DictReader(stream):
                try:
                    candidates.append((float(row["Cost"]), row, source))
                except (KeyError, TypeError, ValueError):
                    continue
    if not candidates:
        raise FileNotFoundError("No saved controller-fixed rows found in results/CMAES_output/only_design")
    _, row, source = min(candidates, key=lambda item: item[0])
    return dict(row), source


def xml_filename(row: dict[str, Any]) -> str:
    return f"{value(row, 'Unique id', 'unique_id', 'Unique Id')}.xml"


def load_configuration(spec: CaseSpec) -> tuple[dict[str, Any], str]:
    if spec.source is not None:
        return paper_row(spec.source), str(spec.source.relative_to(ROOT))
    row, source = controller_fixed_row()
    return row, str(source.relative_to(ROOT))


def continuous_torque(motor: str) -> float:
    import numpy as np

    with (ROOT / "actuator_optimization" / "config_files" / "config.json").open() as stream:
        config = json.load(stream)
    details = config["Motors"][f"Motor{motor}_framed"]
    return 60.0 / (2.0 * np.pi * float(details["Kv"])) * float(details["maxContinuousCurrent"])


def actuator_mass(motor: str, ratio: float) -> float:
    import pandas as pd

    table = pd.read_csv(RESULTS / "optimal_gearbox_selection.csv")
    choices = table[table["motor"] == motor]
    if choices.empty:
        raise ValueError(f"Saved motor {motor!r} is absent from optimal_gearbox_selection.csv")
    # This is the same round-and-nearest lookup used by every CMA-ES script.
    selected = choices.loc[(choices["target_ratio"] - round(ratio, 1)).abs().idxmin()]
    return float(selected["mass"])


def rebuild_xml_if_missing(spec: CaseSpec, row: dict[str, Any]) -> Path:
    output = spec.xml_directory / xml_filename(row)
    if output.is_file():
        print(f"XML: {output.relative_to(ROOT)} (saved)")
        return output

    thigh = float(value(row, "Thigh", "thigh_length"))
    calf = float(value(row, "Calf", "calf_length"))
    width = float(value(row, "Torso distance", "torso_distance"))
    height = float(value(row, "ik_height", "IK height", "ik_height (m)"))
    left_motor = str(value(row, "Hip left motor", "motor_left_name", "motor_left"))
    right_motor = str(value(row, "Hip right motor", "motor_right_name", "motor_right"))
    left_ratio = float(value(row, "Hip left ratio", "gear_left_ratio", "gear_ratio_left"))
    right_ratio = float(value(row, "Hip right ratio", "gear_right_ratio", "gear_ratio_right"))
    left_efficiency = float(value(row, "Efficiency left", "efficiency_left"))
    right_efficiency = float(value(row, "Efficiency right", "efficiency_right"))

    import numpy as np
    import pandas as pd
    from scipy.interpolate import interp1d

    link_data = pd.read_csv(ROOT / "components" / "calf_15_35.csv").sort_values("Link Length (mm)")
    link_mass = interp1d(
        link_data["Link Length (mm)"].to_numpy() / 1000.0,
        link_data["Calculated Mass (kg)"].to_numpy(),
        kind="linear", bounds_error=False, fill_value="extrapolate",
    )
    left_torque = continuous_torque(left_motor) * left_ratio
    right_torque = continuous_torque(right_motor) * right_ratio

    tree = ET.parse(XMLS / "5bar_base.xml")
    root = tree.getroot()
    half_width = width / 2.0

    for body in root.findall(".//body[@name='root']"):
        position = body.get("pos", "").split()
        position[2] = str(height)
        body.set("pos", " ".join(position))
    for torso in root.findall(".//geom[@name='torso']"):
        size = torso.get("size", "").split()
        size[0] = str(width)
        torso.set("size", " ".join(size))
        torso.set("mass", str(actuator_mass(left_motor, left_ratio) + actuator_mass(right_motor, right_ratio)))
    for side, sign in (("left", -1), ("right", 1)):
        for body in root.findall(f".//body[@name='l1_{side}']"):
            body.set("pos", f"{sign * half_width} 0 0")
        for geom in root.findall(f".//geom[@name='thigh_{side}']"):
            geom.set("fromto", f"0 0 0 {thigh} 0 0")
            geom.set("mass", str(float(link_mass(thigh))))
        for body in root.findall(f".//body[@name='l2_{side}']"):
            body.set("pos", f"{thigh} 0 0")
        for geom in root.findall(f".//geom[@name='shank_{side}']"):
            geom.set("fromto", f"0 0 0 {calf} 0 0")
            geom.set("mass", str(float(link_mass(calf))))
        for geom in root.findall(f".//geom[@name='foot_{side}']"):
            geom.set("pos", f"{calf} 0 0")
        for site in root.findall(f".//site[@name='{side}_tip']"):
            site.set("pos", f"{calf} 0 0")
    for motor, torque, efficiency in (
        ("motor_left", left_torque, left_efficiency),
        ("motor_right", right_torque, right_efficiency),
    ):
        limit = efficiency * torque
        for actuator in root.findall(f".//motor[@name='{motor}']"):
            actuator.set("ctrlrange", f"-{limit} {limit}")

    output.parent.mkdir(parents=True, exist_ok=True)
    tree.write(output)
    print(f"XML: {output.relative_to(ROOT)} (rebuilt deterministically from saved parameters)")
    return output


def record_video(sim: Any, enabled: bool, path: Path):
    """Capture frames around the existing simulation function without changing it."""
    if not enabled:
        return None, lambda: None
    import imageio.v2 as imageio

    path.parent.mkdir(parents=True, exist_ok=True)
    original_step = sim.mj.mj_step
    state: dict[str, Any] = {"renderer": None, "writer": None, "failed": None, "count": 0}

    def wrapped_step(model: Any, data: Any) -> None:
        original_step(model, data)
        if state["failed"] is not None:
            return
        state["count"] += 1
        # The model timestep is 1 ms.  Sampling every 20 steps yields 50 fps
        # without retaining thousands of full-HD frames in memory.
        if state["count"] % 20:
            return
        try:
            if state["renderer"] is None:
                state["renderer"] = sim.mj.Renderer(model, width=640, height=480)
                state["writer"] = imageio.get_writer(path, fps=50, macro_block_size=1)
            state["renderer"].update_scene(data)
            state["writer"].append_data(state["renderer"].render())
        except Exception as exc:  # Rendering should not invalidate a physics run.
            state["failed"] = exc

    sim.mj.mj_step = wrapped_step

    def finish() -> None:
        sim.mj.mj_step = original_step
        if state["writer"] is not None:
            state["writer"].close()
        if state["renderer"] is not None:
            state["renderer"].close()
        if state["failed"] is not None:
            print(f"Video unavailable: {state['failed']}")
        elif state["writer"] is None:
            print("Video unavailable: simulation ended before a frame could be captured")
        else:
            print(f"Video: {path.relative_to(ROOT)}")

    return path, finish


def run_one(key: str, video: bool) -> dict[str, Any]:
    import numpy as np
    import opt_codesign_5bar as simulation

    spec = CASES[key]
    row, source = load_configuration(spec)
    xml_path = rebuild_xml_if_missing(spec, row)
    left_motor = str(value(row, "Hip left motor", "motor_left_name", "motor_left"))
    right_motor = str(value(row, "Hip right motor", "motor_right_name", "motor_right"))
    left_ratio = float(value(row, "Hip left ratio", "gear_left_ratio", "gear_ratio_left"))
    right_ratio = float(value(row, "Hip right ratio", "gear_right_ratio", "gear_ratio_right"))
    height = float(value(row, "ik_height", "IK height", "ik_height (m)"))
    thigh = float(value(row, "Thigh", "thigh_length"))
    calf = float(value(row, "Calf", "calf_length"))
    width = float(value(row, "Torso distance", "torso_distance"))
    left_efficiency = float(value(row, "Efficiency left", "efficiency_left"))
    right_efficiency = float(value(row, "Efficiency right", "efficiency_right"))
    action = np.array([float(value(row, "ac1")), float(value(row, "ac2")), float(value(row, "ac3"))])
    video_path = RESULTS / "videos" / f"{spec.key}.mp4"
    log_path = RESULTS / "run_cases" / f"{spec.key}.log"

    print(f"\n== {spec.label} ==")
    print(f"Saved parameters: {source}")
    _, finish_video = record_video(simulation, video, video_path)
    try:
        log_path.parent.mkdir(parents=True, exist_ok=True)
        with log_path.open("w") as log, contextlib.redirect_stdout(log):
            print(f"Case: {spec.label}")
            print(f"Saved parameters: {source}")
            result = simulation.run(
                str(xml_path), action, -height,
                continuous_torque(left_motor) * left_ratio,
                continuous_torque(right_motor) * right_ratio,
                thigh, calf, width * 0.5, left_efficiency, right_efficiency,
                float(value(row, "ori_l")), float(value(row, "ori_theta")),
            )
            print(f"Raw signed jump distance (m): {result[2]}")
            print(f"Jump height (m): {result[0]}")
            print(f"Energy (J): {result[3]}")
            print(f"Duration (s): {result[4]}")
            print(f"Velocity (m/s): {result[1]}")
    finally:
        finish_video()

    jump_height, velocity, signed_distance, energy, duration, jump_results = result
    report = {
        "case": spec.key,
        "label": spec.label,
        "source": source,
        "xml": str(xml_path.relative_to(ROOT)),
        "video": str(video_path.relative_to(ROOT)) if video and video_path.is_file() else None,
        "log": str(log_path.relative_to(ROOT)),
        "height_m": float(jump_height),
        "signed_jump_distance_m": float(signed_distance),
        "jump_distance_m": abs(float(signed_distance)),
        "energy_j": float(energy),
        "duration_s": float(duration),
        "velocity_m_s": float(velocity),
        "detected_jumps": len(jump_results),
    }
    output = RESULTS / "run_cases" / f"{spec.key}.json"
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(report, indent=2) + "\n")
    print(f"Height: {report['height_m']:.4f} m")
    print(f"Jump distance (signed): {report['signed_jump_distance_m']:.4f} m")
    print(f"Jump distance (absolute): {report['jump_distance_m']:.4f} m")
    print(f"Energy: {report['energy_j']:.4f} J")
    print(f"Duration: {report['duration_s']:.4f} s")
    print(f"Velocity: {report['velocity_m_s']:.4f} m/s")
    print(f"Metrics: {output.relative_to(ROOT)}")
    print(f"Detailed log: {log_path.relative_to(ROOT)}")
    if not video:
        print("Video: disabled (--no-video)")
    elif report["video"] is None:
        print("Video: unavailable (see message above)")
    return report


def print_table(reports: list[dict[str, Any]]) -> None:
    print("\nCase                Height(m)  Distance(m)  Energy(J)  Duration(s)")
    for report in reports:
        print(
            f"{report['label']:<19} {report['height_m']:>9.4f} "
            f"{report['jump_distance_m']:>11.4f} {report['energy_j']:>10.4f} "
            f"{report['duration_s']:>11.4f}"
        )


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("case", nargs="?", choices=(*CASES, "all"))
    parser.add_argument("--no-video", action="store_true", help="run physics without writing an MP4")
    parser.add_argument("--check-deps", action="store_true", help="verify the active Python environment and exit")
    args = parser.parse_args()
    try:
        require_packages()
        if args.check_deps:
            print("Required simulation packages are available in the active Python environment.")
            return 0
        if args.case is None:
            parser.error("a case is required")
        keys = ALL_CASES if args.case == "all" else (args.case,)
        reports = [run_one(key, not args.no_video) for key in keys]
        if args.case == "all":
            print_table(reports)
    except Exception as exc:
        print(f"run_saved_case.py: {exc}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
