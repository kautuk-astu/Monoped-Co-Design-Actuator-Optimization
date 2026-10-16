#!/usr/bin/env bash
# Non-interactive launcher: fixed L1=L2=0.20 m and MAD_M6C12 at both joints.
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd -- "${SCRIPT_DIR}/.." && pwd)"
PYTHON_BIN="${PYTHON_BIN:-python}"

# Default experiment configuration. Edit these values here when starting a
# new study; command-line options below are only optional overrides.
export MUJOCO_GL="${MUJOCO_GL:-egl}"
SEED=5
MAXITER=1000
POPSIZE=8
SIMULATION_DURATION=2.0
SINGLE_JUMP=false

while [[ $# -gt 0 ]]; do
    case "$1" in
        --seed) SEED="$2"; shift 2 ;;
        --maxiter|--iterations) MAXITER="$2"; shift 2 ;;
        --popsize|--population) POPSIZE="$2"; shift 2 ;;
        --simulation-duration) SIMULATION_DURATION="$2"; shift 2 ;;
        --single-jump) SINGLE_JUMP=true; shift ;;
        *) echo "error: unknown or incomplete option: $1" >&2; exit 2 ;;
    esac
done

if [[ -f "${REPO_ROOT}/.venv/bin/activate" ]]; then
    # shellcheck disable=SC1091
    source "${REPO_ROOT}/.venv/bin/activate"
    PYTHON_BIN=python
fi

SINGLE_JUMP_ARGS=()
RUN_TYPE=multi_jump
if [[ "${SINGLE_JUMP}" == true ]]; then
    SINGLE_JUMP_ARGS=(--single-jump)
    RUN_TYPE=single_jump
fi

for required in \
    "${SCRIPT_DIR}/data/sspg_actuator_selection.csv" \
    "${SCRIPT_DIR}/data/motor_config.json" \
    "${SCRIPT_DIR}/optimize_fixed_link_motor.py" \
    "${SCRIPT_DIR}/model.py" \
    "${SCRIPT_DIR}/simulate.py"; do
    [[ -f "${required}" ]] || { echo "error: required input missing: ${required}" >&2; exit 1; }
done

"${PYTHON_BIN}" -c 'import cma, imageio, mujoco, numpy, pandas' || {
    echo "error: Python environment needs cma, imageio, mujoco, numpy, and pandas." >&2
    exit 1
}

STAMP="$(date +%Y-%m-%d_%H-%M-%S)"
RUN_DIR="${SCRIPT_DIR}/results/fixed_link_MAD_M6C12_${RUN_TYPE}_run_${STAMP}_seed${SEED}"
mkdir -p "${RUN_DIR}"

echo "Starting fixed-link/MAD_M6C12 CMA-ES run: ${RUN_DIR}"
echo "MuJoCo backend: ${MUJOCO_GL}; study: ${RUN_TYPE}; seed: ${SEED}; iterations: ${MAXITER}; population: ${POPSIZE}; duration: ${SIMULATION_DURATION} s"
"${PYTHON_BIN}" "${SCRIPT_DIR}/optimize_fixed_link_motor.py" \
    --seed "${SEED}" --maxiter "${MAXITER}" --popsize "${POPSIZE}" \
    --simulation-duration "${SIMULATION_DURATION}" "${SINGLE_JUMP_ARGS[@]}" --output-dir "${RUN_DIR}" \
    2>&1 | tee "${RUN_DIR}/run.log"

echo "Complete. Best design: ${RUN_DIR}/best_design.json"
echo "Summary: ${RUN_DIR}/summary.txt"

if "${PYTHON_BIN}" "${SCRIPT_DIR}/render_best.py" \
    --best-json "${RUN_DIR}/best_design.json" \
    --renderer "${SCRIPT_DIR}/render_video.py" \
    --output-dir "${RUN_DIR}" \
    --duration "${SIMULATION_DURATION}" \
    "${SINGLE_JUMP_ARGS[@]}" \
    2>&1 | tee -a "${RUN_DIR}/run.log"
then
    echo "Best-design videos: ${RUN_DIR}/best_jump_{front,left_oblique,right_oblique}.mp4"
    echo "Three-panel collage: ${RUN_DIR}/best_jump_collage.mp4"
    echo "Best-design XML: ${RUN_DIR}/best_design.xml"
else
    echo "No best-design videos were created; no valid candidate or rendering failed." >&2
fi
