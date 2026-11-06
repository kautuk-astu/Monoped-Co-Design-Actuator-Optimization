#!/usr/bin/env bash
# Separate end-to-end launcher for data/actuator_selection_nishant.csv.
set -Eeuo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd -- "${SCRIPT_DIR}/.." && pwd)"
COMPACT_REPO="${COMPACT_REPO:-$(cd -- "${REPO_ROOT}/.." && pwd)/COMPAct}"
PYTHON_BIN="${PYTHON_BIN:-python}"

SEED="${SEED:-5}"
MAXITER="${MAXITER:-1000}"
POPSIZE="${POPSIZE:-8}"
SIMULATION_DURATION_S="${SIMULATION_DURATION_S:-10}"
CATALOGUE="${NISHANT_CATALOGUE:-${SCRIPT_DIR}/data/actuator_selection_nishant.csv}"
MOTOR_CONFIG="${COMPACT_REPO}/config_files/motors.json"

if [[ $# -ne 0 ]]; then
    echo "usage: $0" >&2
    echo "configure with SEED, MAXITER, POPSIZE, and SIMULATION_DURATION_S" >&2
    exit 2
fi

if [[ -f "${REPO_ROOT}/.venv/bin/activate" ]]; then
    # shellcheck disable=SC1091
    source "${REPO_ROOT}/.venv/bin/activate"
    PYTHON_BIN=python
fi

for required in \
    "${CATALOGUE}" \
    "${MOTOR_CONFIG}" \
    "${SCRIPT_DIR}/optimize_aligned_nishant.py"; do
    [[ -f "${required}" ]] || {
        echo "error: required file missing: ${required}" >&2
        exit 1
    }
done

"${PYTHON_BIN}" -c 'import cma, matplotlib, mujoco, numpy, pandas' || {
    echo "error: required Python packages are missing" >&2
    exit 1
}

STAMP="$(date +%Y-%m-%d_%H-%M-%S)"
RUN_ROOT="${SCRIPT_DIR}/results/aligned_nishant/run_${STAMP}_seed${SEED}"
MPL_DIR="${SCRIPT_DIR}/results/aligned_nishant/.matplotlib"
mkdir -p "${MPL_DIR}"
export MPLCONFIGDIR="${MPLCONFIGDIR:-${MPL_DIR}}"

echo "Nishant-catalogue aligned optimization"
echo "  source catalogue:    ${CATALOGUE}"
echo "  output:              ${RUN_ROOT}"
echo "  motor simulation:    MAD_M6C12_KV170"
echo "  generations:         ${MAXITER}"
echo "  population:          ${POPSIZE}"
echo "  seed:                ${SEED}"
echo "  simulation duration: ${SIMULATION_DURATION_S} s per candidate"
echo "The catalogue determines which gearbox types and ratios are available."

"${PYTHON_BIN}" "${SCRIPT_DIR}/optimize_aligned_nishant.py" \
    --catalogue "${CATALOGUE}" \
    --motor-config "${MOTOR_CONFIG}" \
    --output-dir "${RUN_ROOT}" \
    --seed "${SEED}" \
    --maxiter "${MAXITER}" \
    --popsize "${POPSIZE}" \
    --simulation-duration "${SIMULATION_DURATION_S}"

echo "Complete."
echo "  Comparison: ${RUN_ROOT}/gearbox_comparison.csv"
echo "  Best result: ${RUN_ROOT}/best_overall.json"
