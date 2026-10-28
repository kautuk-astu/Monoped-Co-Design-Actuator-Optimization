#!/usr/bin/env python3

import argparse
import json
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent
COMPACT = ROOT / "COMPAct"

TARGETS = {
    "B_hip": (
        "Opt_wolfromPlanetaryGBOptimization",
        24.0,
    ),
    "B_knee": (
        "Opt_wolfromPlanetaryGBOptimization",
        16.916162279837167,
    ),
    "C_hip": (
        "Opt_doubleStagePlanetaryGBOptimization",
        18.838091104279815,
    ),
    "C_knee": (
        "Opt_wolfromPlanetaryGBOptimization",
        19.877795866912138,
    ),
}


def main():
    parser = argparse.ArgumentParser()

    parser.add_argument(
        "--candidate",
        choices=[*TARGETS, "all"],
        required=True,
    )

    args = parser.parse_args()

    if not (COMPACT / "ActuatorAndGearbox.py").exists():
        parser.error("COMPAct repository not found")

    output = ROOT / "results_stage3_compact"
    output.mkdir(exist_ok=True)

    names = (
        list(TARGETS)
        if args.candidate == "all"
        else [args.candidate]
    )

    for name in names:

        module, ratio = TARGETS[name]

        if not 4 <= ratio <= 25:
            raise ValueError("Gear ratio outside project bounds")

        logfile = output / f"{name}.log"

        metadata = {
            "candidate": name,
            "module": module,
            "requested_ratio": ratio,
            "motor": "MAD_M6C12",
            "status": "requested",
        }

        (output / f"{name}_request.json").write_text(
            json.dumps(metadata, indent=2)
        )

        code = """
import importlib
import sys

module = importlib.import_module(sys.argv[1])

result = module.run(
    "MAD_M6C12",
    float(sys.argv[2])
)

print("COMPAct returned:", repr(result))
"""

        print(
            f"Running {name}, target ratio {ratio:.6f}",
            flush=True,
        )

        with logfile.open("w") as log:

            process = subprocess.run(
                [
                    sys.executable,
                    "-u",
                    "-c",
                    code,
                    module,
                    str(ratio),
                ],
                cwd=COMPACT,
                stdout=log,
                stderr=subprocess.STDOUT,
                check=False,
            )

        print(
            f"{name}: exit code {process.returncode}",
            flush=True,
        )

        print(f"Log saved: {logfile}", flush=True)

        if process.returncode != 0:
            print("Optimisation failed. Check the log.")
            sys.exit(process.returncode)

    print("COMPAct optimisation completed.")
    print("Verify actual gear ratios and geometry from COMPAct outputs.")


if __name__ == "__main__":
    main()
