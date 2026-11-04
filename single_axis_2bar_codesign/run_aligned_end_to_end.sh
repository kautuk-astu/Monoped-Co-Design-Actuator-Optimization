#!/usr/bin/env bash
# End-to-end aligned-monoped study: tests, CMA-ES, and exact best replay.
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd -- "${SCRIPT_DIR}/.." && pwd)"
PYTHON_BIN="${PYTHON_BIN:-python}"

# Requested fixed experiment configuration.
readonly SEED=5
readonly MAXITER=1000
readonly POPSIZE=8
readonly SIMULATION_DURATION_S=10

RENDER_VIDEO=false
if [[ $# -gt 1 ]] || { [[ $# -eq 1 ]] && [[ "$1" != "--video" ]]; }; then
    echo "usage: $0 [--video]" >&2
    exit 2
elif [[ $# -eq 1 ]]; then
    RENDER_VIDEO=true
fi

if [[ -f "${REPO_ROOT}/.venv/bin/activate" ]]; then
    # shellcheck disable=SC1091
    source "${REPO_ROOT}/.venv/bin/activate"
    PYTHON_BIN=python
fi

for required in \
    "${SCRIPT_DIR}/data/sspg_actuator_selection.csv" \
    "${SCRIPT_DIR}/data/motor_config.json" \
    "${SCRIPT_DIR}/optimize_aligned.py" \
    "${SCRIPT_DIR}/replay_aligned.py" \
    "${SCRIPT_DIR}/model.py" \
    "${SCRIPT_DIR}/simulate.py"; do
    [[ -f "${required}" ]] || {
        echo "error: required input missing: ${required}" >&2
        exit 1
    }
done

"${PYTHON_BIN}" -c 'import cma, matplotlib, mujoco, numpy, pandas' || {
    echo "error: required Python packages are missing." >&2
    echo "run: ${PYTHON_BIN} -m pip install -r ${SCRIPT_DIR}/requirements-aligned.txt" >&2
    exit 1
}

STAMP="$(date +%Y-%m-%d_%H-%M-%S)"
RUN_DIR="${SCRIPT_DIR}/results/aligned/run_${STAMP}_seed${SEED}"
mkdir -p "${RUN_DIR}/.matplotlib"
export MPLCONFIGDIR="${MPLCONFIGDIR:-${RUN_DIR}/.matplotlib}"

echo "Aligned-monoped end-to-end run"
echo "  output:              ${RUN_DIR}"
echo "  CMA-ES generations:  ${MAXITER}"
echo "  population:          ${POPSIZE}"
echo "  seed:                ${SEED}"
echo "  simulation duration: ${SIMULATION_DURATION_S} s per candidate"
echo "  total evaluations:   $((MAXITER * POPSIZE))"
echo "This is a long run: 8,000 candidates at a 1 ms MuJoCo timestep."

{
    echo "Running test suite..."
    cd "${SCRIPT_DIR}"
    "${PYTHON_BIN}" -m unittest discover -s tests -v

    echo "Starting CMA-ES optimization..."
    "${PYTHON_BIN}" "${SCRIPT_DIR}/optimize_aligned.py" \
        --seed "${SEED}" \
        --maxiter "${MAXITER}" \
        --popsize "${POPSIZE}" \
        --simulation-duration "${SIMULATION_DURATION_S}" \
        --output-dir "${RUN_DIR}"

    if ! "${PYTHON_BIN}" -c \
        'import json,sys; data=json.load(open(sys.argv[1])); raise SystemExit(data.get("best_valid_candidate", True) is None)' \
        "${RUN_DIR}/best_design.json"; then
        echo "error: CMA-ES completed without a qualified feasible candidate; see ${RUN_DIR}/all_candidates.csv" >&2
        exit 1
    fi

    echo "Replaying the exact saved winner..."
    "${PYTHON_BIN}" "${SCRIPT_DIR}/replay_aligned.py" "${RUN_DIR}"

    if [[ "${RENDER_VIDEO}" == true ]]; then
        echo "Rendering the exact saved winner..."
        "${PYTHON_BIN}" "${SCRIPT_DIR}/replay_aligned.py" "${RUN_DIR}" --video
    fi
} 2>&1 | tee "${RUN_DIR}/run.log"

echo "Complete."
echo "  Summary:    ${RUN_DIR}/summary.txt"
echo "  Compact log: ${RUN_DIR}/compact_optimization_log.json"
echo "  Best design: ${RUN_DIR}/best_design.json"
echo "  Validation:  ${RUN_DIR}/best_validation.json"
echo "  Telemetry:   ${RUN_DIR}/best_jump_telemetry.csv"
echo "  Plots:       ${RUN_DIR}/alignment_error.png and ${RUN_DIR}/jump_trajectory.png"
if [[ "${RENDER_VIDEO}" == true ]]; then
    echo "  Video:       ${RUN_DIR}/best_jump.mp4"
fi
