#!/usr/bin/env bash
# Non-interactive launcher for fixed L1=L2=0.20 m CMA-ES co-design.
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd -- "${SCRIPT_DIR}/.." && pwd)"
PYTHON_BIN="${PYTHON_BIN:-python}"
SEED=5
MAXITER=100
POPSIZE=8
ENERGY_WEIGHT=0.015

while [[ $# -gt 0 ]]; do
    case "$1" in
        --seed) SEED="$2"; shift 2 ;;
        --maxiter|--iterations) MAXITER="$2"; shift 2 ;;
        --popsize|--population) POPSIZE="$2"; shift 2 ;;
        --energy-weight) ENERGY_WEIGHT="$2"; shift 2 ;;
        *) echo "error: unknown or incomplete option: $1" >&2; exit 2 ;;
    esac
done

if [[ -f "${REPO_ROOT}/.venv/bin/activate" ]]; then
    # shellcheck disable=SC1091
    source "${REPO_ROOT}/.venv/bin/activate"
    PYTHON_BIN=python
fi

for required in \
    "${REPO_ROOT}/results/optimal_gearbox_selection.csv" \
    "${REPO_ROOT}/actuator_optimization/config_files/config.json" \
    "${SCRIPT_DIR}/optimize_fixed_link.py" \
    "${SCRIPT_DIR}/model.py" \
    "${SCRIPT_DIR}/simulate.py"; do
    [[ -f "${required}" ]] || { echo "error: required input missing: ${required}" >&2; exit 1; }
done

"${PYTHON_BIN}" -c 'import cma, mujoco, numpy, pandas' || {
    echo "error: Python environment needs cma, mujoco, numpy, and pandas." >&2
    exit 1
}

STAMP="$(date +%Y-%m-%d_%H-%M-%S)"
RUN_DIR="${SCRIPT_DIR}/results/fixed_link_run_${STAMP}_seed${SEED}"
mkdir -p "${RUN_DIR}"

echo "Starting fixed-link CMA-ES run (L1=L2=0.20 m): ${RUN_DIR}"
"${PYTHON_BIN}" "${SCRIPT_DIR}/optimize_fixed_link.py" \
    --seed "${SEED}" --maxiter "${MAXITER}" --popsize "${POPSIZE}" \
    --energy-weight "${ENERGY_WEIGHT}" --output-dir "${RUN_DIR}" \
    2>&1 | tee "${RUN_DIR}/run.log"

echo "Complete. Best design: ${RUN_DIR}/best_design.json"
echo "Summary: ${RUN_DIR}/summary.txt"

if "${PYTHON_BIN}" - "${RUN_DIR}/best_design.json" "${SCRIPT_DIR}/render_video.py" "${RUN_DIR}" <<'PY' | tee -a "${RUN_DIR}/run.log"
import json
import subprocess
import sys
from pathlib import Path

best_path, renderer, run_dir = map(Path, sys.argv[1:])
best = json.loads(best_path.read_text())
if not best.get("valid", False):
    raise SystemExit("No valid best candidate; skipping best-design video.")

command = [
    sys.executable, str(renderer), "--output", str(run_dir / "best_jump.mp4"),
    "--xml", str(run_dir / "best_design.xml"), "--thigh", str(best["l1"]),
    "--shank", str(best["l2"]), "--payload", str(best["payload_mass"]),
    "--theta1", str(best["theta1_rad"]), "--theta2", str(best["theta2_rad"]),
    "--hip-motor", str(best["hip_motor"]), "--knee-motor", str(best["knee_motor"]),
    "--hip-ratio", str(best["hip_gear_ratio"]), "--knee-ratio", str(best["knee_gear_ratio"]),
    "--ori-l", str(best["ori_l"]), "--ori-theta", str(best["ori_theta_rad"]),
    "--K", str(best["K"]), "--C", str(best["C"]), "--T-gain", str(best["T_gain"]),
    "--thrust-time", str(best["thrust_time"]),
]
print("Rendering global best fixed-link candidate:", " ".join(command))
subprocess.run(command, check=True)
PY
then
    echo "Best-design video: ${RUN_DIR}/best_jump.mp4"
    echo "Best-design XML: ${RUN_DIR}/best_design.xml"
else
    echo "No best-design video was created; no valid candidate or rendering failed." >&2
fi
