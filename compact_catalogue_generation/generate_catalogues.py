#!/usr/bin/env python3
"""Generate isolated MAD_M6C12_KV170 COMPAct catalogues with process workers.

COMPAct writes a fixed-name result CSV in the current working directory and, by
default, CAD equation files beside its source modules. To keep the source
repository unchanged and avoid parallel write collisions, every multiprocessing
worker gets its own minimal runtime copy of the required COMPAct source and
configuration. CAD equation writes are disabled in those temporary copies.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import multiprocessing as mp
import os
from pathlib import Path
import shutil
import signal
import subprocess
import sys
import time
import traceback
from typing import Any, Iterable


MOTOR = "MAD_M6C12_KV170"
GEARBOXES = ("SSPG", "CPG", "WPG", "DSPG")
PARAM_FILES = {
    "SSPG": ("sspg_params.json", "sspg_optimization_parameters"),
    "CPG": ("cpg_params.json", "cpg_optimization_parameters"),
    "WPG": ("wpg_params.json", "wpg_optimization_parameters"),
    "DSPG": ("dspg_params.json", "dspg_optimization_parameters"),
}
RATIO_SPACING = 0.1
DEFAULT_WORKERS = 18

# Keep the delivered catalogues deliberately small. Identification and status
# columns make infeasible/error rows usable; feasible measurements are limited
# to the three values requested by the experiment.
CATALOGUE_FIELDS = [
    "gearbox_type",
    "requested_ratio",
    "actual_ratio",
    "mass_kg",
    "efficiency",
    "feasibility_status",
    "failure_reason",
]

_WORKER_SOURCE: Path | None = None
_COMPACT_SOURCE: Path | None = None
_STAGING_ROOT: Path | None = None


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--compact-repo", required=True, type=Path)
    parser.add_argument("--output-dir", required=True, type=Path)
    parser.add_argument("--workers", type=int, default=DEFAULT_WORKERS)
    return parser.parse_args()


def load_json(path: Path) -> dict[str, Any]:
    with path.open("r", encoding="utf-8") as handle:
        return json.load(handle)


def validate_repository(repo: Path) -> None:
    required = [
        repo / "actOpt.py",
        repo / "ActuatorAndGearbox.py",
        repo / "config_files" / "config.json",
        repo / "config_files" / "motors.json",
    ]
    required.extend(repo / "config_files" / item[0] for item in PARAM_FILES.values())
    missing = [str(path) for path in required if not path.is_file()]
    if missing:
        raise FileNotFoundError("COMPAct is missing required files: " + ", ".join(missing))

    motors = load_json(repo / "config_files" / "motors.json").get("motors", {})
    if MOTOR not in motors:
        raise KeyError(
            f"Motor identifier {MOTOR!r} was not found in config_files/motors.json"
        )
    if motors[MOTOR].get("type") != "framed_outrunner":
        raise ValueError(f"Motor {MOTOR!r} is not configured as a framed_outrunner")


def read_ranges(repo: Path) -> dict[str, dict[str, float]]:
    ranges: dict[str, dict[str, float]] = {}
    for gearbox, (filename, section) in PARAM_FILES.items():
        params = load_json(repo / "config_files" / filename)[section]
        ranges[gearbox] = {
            "minimum": float(params["GEAR_RATIO_MIN"]),
            "maximum": float(params["GEAR_RATIO_MAX"]),
            "spacing": RATIO_SPACING,
            "configured_spacing": float(params["GEAR_RATIO_STEP"]),
        }
    return ranges


def ratio_strings(minimum: float, maximum: float) -> list[str]:
    """Return inclusive tenths without accumulating floating-point error."""
    start = int(round(minimum * 10))
    stop = int(round(maximum * 10))
    if not math.isclose(start / 10, minimum) or not math.isclose(stop / 10, maximum):
        raise ValueError(f"Range endpoints {minimum}, {maximum} are not on a 0.1 grid")
    return [f"{value / 10:.1f}" for value in range(start, stop + 1)]


def make_jobs(ranges: dict[str, dict[str, float]], raw_root: Path) -> list[dict[str, str]]:
    jobs: list[dict[str, str]] = []
    for gearbox in GEARBOXES:
        bounds = ranges[gearbox]
        for ratio in ratio_strings(bounds["minimum"], bounds["maximum"]):
            jobs.append(
                {
                    "gearbox": gearbox,
                    "ratio": ratio,
                    "raw_dir": str(raw_root / gearbox.lower() / f"ratio_{ratio}"),
                }
            )
    return jobs


def worker_initializer(compact_source: str, staging_root: str) -> None:
    """Create a private minimal COMPAct source tree for this process worker."""
    global _WORKER_SOURCE, _COMPACT_SOURCE, _STAGING_ROOT
    signal.signal(signal.SIGINT, signal.SIG_IGN)
    _COMPACT_SOURCE = Path(compact_source)
    _STAGING_ROOT = Path(staging_root)
    _WORKER_SOURCE = _STAGING_ROOT / f"worker_{os.getpid()}"
    _WORKER_SOURCE.mkdir(parents=True, exist_ok=False)

    for source_file in _COMPACT_SOURCE.glob("*.py"):
        shutil.copy2(source_file, _WORKER_SOURCE / source_file.name)
    shutil.copytree(_COMPACT_SOURCE / "config_files", _WORKER_SOURCE / "config_files")

    # The original CPG/WPG/DSPG configs use 1.0 ratio bins. The experiment asks
    # for a 0.1 grid, so change only the private configuration snapshots.
    for filename, section in PARAM_FILES.values():
        private_path = _WORKER_SOURCE / "config_files" / filename
        private_params = load_json(private_path)
        private_params[section]["GEAR_RATIO_STEP"] = RATIO_SPACING
        private_path.write_text(
            json.dumps(private_params, indent=4) + "\n",
            encoding="utf-8",
        )

    # Suppress the four CAD-only side effects in the private source. This does
    # not touch optimizer costs, constraints, or returned catalogue values.
    core_path = _WORKER_SOURCE / "ActuatorAndGearbox.py"
    core_text = core_path.read_text(encoding="utf-8")
    cad_call = (
        "Actuator.genEquationFile(motor_name=Actuator.motor.motorName, "
        "gearRatioLL=round(self.gearRatioIter, 1), gearRatioUL = "
        "(round(self.gearRatioIter + self.GEAR_RATIO_STEP,1)))"
    )
    if core_text.count(cad_call) != 4:
        raise RuntimeError(
            "The COMPAct source no longer matches the four expected CAD-write sites"
        )
    core_text = core_text.replace(cad_call, "pass  # CAD equation output disabled for catalogue run")
    core_path.write_text(core_text, encoding="utf-8")


def parse_compact_csv(path: Path) -> tuple[dict[str, str] | None, float | None]:
    if not path.is_file():
        return None, None

    lines = path.read_text(encoding="utf-8", errors="replace").splitlines()
    header_index = next((i for i, line in enumerate(lines) if line.startswith("iter,")), None)
    if header_index is None:
        return None, _read_internal_time(lines)

    reader = csv.reader(lines[header_index:])
    headers = [name.strip() for name in next(reader)]
    rows: list[dict[str, str]] = []
    for values in reader:
        if not values or not any(value.strip() for value in values):
            break
        if values[0].strip().startswith("Running Time"):
            break
        if len(values) == len(headers):
            rows.append({key: value.strip() for key, value in zip(headers, values)})
    return (rows[-1] if rows else None), _read_internal_time(lines)


def _read_internal_time(lines: list[str]) -> float | None:
    for index, line in enumerate(lines):
        if line.strip() == "Running Time (sec)" and index + 1 < len(lines):
            try:
                return float(lines[index + 1].strip())
            except ValueError:
                return None
    return None


def normalize_row(
    gearbox: str,
    requested_ratio: str,
    raw: dict[str, str],
    return_code: int,
    wall_time: float,
    internal_time: float | None,
    raw_csv: Path,
    console_log: Path,
) -> dict[str, Any]:
    result: dict[str, Any] = {field: "" for field in CATALOGUE_FIELDS}
    result.update(
        {
            "gearbox_type": gearbox,
            "requested_ratio": requested_ratio,
            "actual_ratio": raw.get("gearRatio", ""),
            "mass_kg": raw.get("mass", ""),
            "failure_reason": "",
            "efficiency": raw.get("eff", ""),
            "feasibility_status": "feasible",
            # Internal-only values are ignored by the CSV writer and used for
            # progress and metadata runtime statistics.
            "feasible": True,
            "status": "feasible",
            "wall_time_seconds": f"{wall_time:.6f}",
        }
    )
    return result


def failure_row(
    gearbox: str,
    ratio: str,
    status: str,
    reason: str,
    return_code: int | str,
    wall_time: float,
    internal_time: float | None,
    raw_csv: Path,
    console_log: Path,
) -> dict[str, Any]:
    result: dict[str, Any] = {field: "" for field in CATALOGUE_FIELDS}
    result.update(
        {
            "gearbox_type": gearbox,
            "requested_ratio": ratio,
            "feasibility_status": status,
            "failure_reason": reason,
            "feasible": False,
            "status": status,
            "wall_time_seconds": f"{wall_time:.6f}",
        }
    )
    return result


def evaluate(job: dict[str, str]) -> dict[str, Any]:
    if _WORKER_SOURCE is None:
        raise RuntimeError("Worker source was not initialized")

    gearbox = job["gearbox"]
    ratio = job["ratio"]
    raw_dir = Path(job["raw_dir"])
    raw_dir.mkdir(parents=True, exist_ok=False)
    result_dir = raw_dir / "results" / f"results_BruteForce_{MOTOR}"
    result_dir.mkdir(parents=True)
    raw_csv = result_dir / f"{gearbox}_BRUTEFORCE_MIT_{MOTOR}.csv"
    console_log = raw_dir / "console.log"

    command = [
        sys.executable,
        str(_WORKER_SOURCE / "actOpt.py"),
        "--motor", MOTOR,
        "--gb_type", gearbox.lower(),
        "--gr", ratio,
        "--material", "3dp",
    ]
    started = time.perf_counter()
    process: subprocess.Popen[str] | None = None
    previous_sigterm = signal.getsignal(signal.SIGTERM)

    def terminate_child(_signum: int, _frame: Any) -> None:
        if process is not None and process.poll() is None:
            try:
                os.killpg(process.pid, signal.SIGTERM)
            except ProcessLookupError:
                pass
        raise SystemExit(143)

    try:
        signal.signal(signal.SIGTERM, terminate_child)
        process = subprocess.Popen(
            command,
            cwd=raw_dir,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            start_new_session=True,
        )
        output, _ = process.communicate()
        return_code = process.returncode
        console_log.write_text(output, encoding="utf-8")
        wall_time = time.perf_counter() - started
        compact_row, internal_time = parse_compact_csv(raw_csv)

        if return_code != 0:
            reason = output.strip().splitlines()[-1] if output.strip() else "COMPAct exited with an error"
            return failure_row(
                gearbox, ratio, "error", reason, return_code, wall_time, internal_time,
                raw_csv, console_log,
            )
        if compact_row is None:
            reason = "No feasible design was emitted for the requested ratio"
            return failure_row(
                gearbox, ratio, "infeasible", reason, return_code, wall_time, internal_time,
                raw_csv, console_log,
            )
        return normalize_row(
            gearbox, ratio, compact_row, return_code, wall_time, internal_time,
            raw_csv, console_log,
        )
    except Exception as exc:
        wall_time = time.perf_counter() - started
        diagnostic = "".join(traceback.format_exception_only(type(exc), exc)).strip()
        try:
            console_log.write_text(diagnostic + "\n", encoding="utf-8")
        except OSError:
            pass
        return failure_row(
            gearbox, ratio, "worker_error", diagnostic, "", wall_time, None,
            raw_csv, console_log,
        )
    finally:
        signal.signal(signal.SIGTERM, previous_sigterm)


def atomic_write_csv(path: Path, rows: Iterable[dict[str, Any]]) -> None:
    temporary = path.with_suffix(path.suffix + ".tmp")
    with temporary.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=CATALOGUE_FIELDS, extrasaction="ignore")
        writer.writeheader()
        writer.writerows(rows)
    temporary.replace(path)


def sorted_rows(rows: Iterable[dict[str, Any]]) -> list[dict[str, Any]]:
    order = {gearbox: index for index, gearbox in enumerate(GEARBOXES)}
    return sorted(rows, key=lambda row: (order[row["gearbox_type"]], float(row["requested_ratio"])))


def write_catalogues(output_dir: Path, rows: list[dict[str, Any]]) -> None:
    ordered = sorted_rows(rows)
    for gearbox in GEARBOXES:
        atomic_write_csv(
            output_dir / f"{gearbox.lower()}_{MOTOR}_catalogue.csv",
            (row for row in ordered if row["gearbox_type"] == gearbox),
        )
    atomic_write_csv(output_dir / f"combined_{MOTOR}_catalogue.csv", ordered)


def git_details(repo: Path) -> dict[str, Any]:
    def git(*arguments: str) -> str:
        completed = subprocess.run(
            ["git", "-C", str(repo), *arguments],
            check=True,
            text=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
        )
        return completed.stdout.strip()

    try:
        status = git("status", "--porcelain")
        return {"commit": git("rev-parse", "HEAD"), "dirty": bool(status), "status": status}
    except (OSError, subprocess.CalledProcessError) as exc:
        return {"commit": None, "dirty": None, "error": str(exc)}


def configuration_snapshot(repo: Path) -> dict[str, Any]:
    names = ["config.json", "cost.json", "motors.json"] + [
        PARAM_FILES[item][0] for item in GEARBOXES
    ]
    return {name: load_json(repo / "config_files" / name) for name in names}


def runtime_statistics(rows: list[dict[str, Any]], total_jobs: int, elapsed: float) -> dict[str, Any]:
    stats: dict[str, Any] = {
        "wall_time_seconds": round(elapsed, 6),
        "planned_evaluations": total_jobs,
        "completed_evaluations": len(rows),
        "feasible": sum(str(row["feasible"]).lower() == "true" for row in rows),
        "infeasible_or_failed": sum(str(row["feasible"]).lower() != "true" for row in rows),
        "by_gearbox": {},
    }
    for gearbox in GEARBOXES:
        subset = [row for row in rows if row["gearbox_type"] == gearbox]
        durations = [float(row["wall_time_seconds"]) for row in subset]
        stats["by_gearbox"][gearbox] = {
            "completed": len(subset),
            "feasible": sum(str(row["feasible"]).lower() == "true" for row in subset),
            "failed_or_infeasible": sum(str(row["feasible"]).lower() != "true" for row in subset),
            "evaluation_wall_time_sum_seconds": round(sum(durations), 6),
            "evaluation_wall_time_mean_seconds": round(sum(durations) / len(durations), 6)
            if durations else None,
        }
    return stats


def write_metadata(
    path: Path,
    repo: Path,
    output_dir: Path,
    ranges: dict[str, dict[str, float]],
    rows: list[dict[str, Any]],
    total_jobs: int,
    workers: int,
    start_epoch: float,
    run_status: str,
) -> None:
    elapsed = time.time() - start_epoch
    metadata = {
        "schema_version": 1,
        "run_status": run_status,
        "motor": {
            "runtime_identifier": MOTOR,
            "configuration_file": "config_files/motors.json",
            "configuration": load_json(repo / "config_files" / "motors.json")["motors"][MOTOR],
        },
        "gearboxes": list(GEARBOXES),
        "ratio_ranges": ranges,
        "ratio_spacing": RATIO_SPACING,
        "temporary_runtime_adjustments": {
            "gear_ratio_step": "Set to 0.1 in private worker configuration copies",
            "cad_equation_output": "Disabled in private worker copies",
        },
        "workers": workers,
        "compact_repository": str(repo),
        "output_directory": str(output_dir),
        "git": git_details(repo),
        "compact_configuration": configuration_snapshot(repo),
        "runtime_statistics": runtime_statistics(rows, total_jobs, elapsed),
        "started_unix_time": start_epoch,
        "updated_unix_time": time.time(),
    }
    temporary = path.with_suffix(path.suffix + ".tmp")
    temporary.write_text(json.dumps(metadata, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    temporary.replace(path)


def main() -> int:
    args = parse_args()
    repo = args.compact_repo.resolve()
    output_dir = args.output_dir.resolve()
    if args.workers != DEFAULT_WORKERS:
        raise ValueError(f"This experiment is fixed to {DEFAULT_WORKERS} workers")
    validate_repository(repo)

    output_dir.mkdir(parents=True, exist_ok=False)
    staging_root = output_dir / ".worker_sources"
    staging_root.mkdir()
    evaluation_root = staging_root / "evaluations"
    evaluation_root.mkdir()

    ranges = read_ranges(repo)
    jobs = make_jobs(ranges, evaluation_root)
    rows: list[dict[str, Any]] = []
    start_epoch = time.time()
    metadata_path = output_dir / "metadata.json"
    run_status = "running"

    print(f"COMPAct repository: {repo}", flush=True)
    print(f"Output directory:   {output_dir}", flush=True)
    print(f"Motor:              {MOTOR}", flush=True)
    print(f"Workers:            {args.workers}", flush=True)
    print(f"Evaluations:        {len(jobs)}", flush=True)

    context = mp.get_context("spawn")
    pool = context.Pool(
        processes=args.workers,
        initializer=worker_initializer,
        initargs=(str(repo), str(staging_root)),
    )
    try:
        for completed, row in enumerate(pool.imap_unordered(evaluate, jobs, chunksize=1), start=1):
            rows.append(row)
            marker = "OK" if str(row["feasible"]).lower() == "true" else row["status"].upper()
            print(
                f"[{completed:4d}/{len(jobs)}] {row['gearbox_type']:4s} "
                f"requested={row['requested_ratio']:>4s} {marker}",
                flush=True,
            )
            if completed % 10 == 0:
                write_catalogues(output_dir, rows)
                write_metadata(
                    metadata_path, repo, output_dir, ranges, rows, len(jobs),
                    args.workers, start_epoch, run_status,
                )
        pool.close()
        pool.join()
        run_status = "completed"
    except KeyboardInterrupt:
        run_status = "interrupted"
        print("\nInterrupt received; terminating active COMPAct evaluations...", file=sys.stderr, flush=True)
        pool.terminate()
        pool.join()
    except BaseException:
        run_status = "failed"
        pool.terminate()
        pool.join()
        raise
    finally:
        write_catalogues(output_dir, rows)
        write_metadata(
            metadata_path, repo, output_dir, ranges, rows, len(jobs),
            args.workers, start_epoch, run_status,
        )
        shutil.rmtree(staging_root, ignore_errors=True)

    print(f"Run status: {run_status}", flush=True)
    print(f"Combined catalogue: {output_dir / f'combined_{MOTOR}_catalogue.csv'}", flush=True)
    print(f"Metadata:           {metadata_path}", flush=True)
    return 0 if run_status == "completed" else 130


if __name__ == "__main__":
    raise SystemExit(main())
