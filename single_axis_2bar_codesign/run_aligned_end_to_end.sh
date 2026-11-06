#!/usr/bin/env bash
# End-to-end aligned study for MAD_M6C12_KV170 and all four COMPAct gearboxes.
set -Eeuo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd -- "${SCRIPT_DIR}/.." && pwd)"
DEFAULT_COMPACT_REPO="$(cd -- "${REPO_ROOT}/.." && pwd)/COMPAct"
COMPACT_REPO="${COMPACT_REPO:-${DEFAULT_COMPACT_REPO}}"
PYTHON_BIN="${PYTHON_BIN:-python}"

SEED="${SEED:-5}"
MAXITER="${MAXITER:-1000}"
POPSIZE="${POPSIZE:-8}"
SIMULATION_DURATION_S="${SIMULATION_DURATION_S:-10}"
GEARBOXES=(SSPG CPG WPG DSPG)

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

CATALOGUE_GENERATOR="${REPO_ROOT}/compact_catalogue_generation/generate_catalogues.py"
MOTOR_CONFIG="${COMPACT_REPO}/config_files/motors.json"
for required in \
    "${CATALOGUE_GENERATOR}" \
    "${MOTOR_CONFIG}" \
    "${SCRIPT_DIR}/optimize_aligned.py" \
    "${SCRIPT_DIR}/replay_aligned.py" \
    "${SCRIPT_DIR}/compare_aligned_gearboxes.py" \
    "${SCRIPT_DIR}/model.py" \
    "${SCRIPT_DIR}/simulate.py"; do
    [[ -f "${required}" ]] || {
        echo "error: required file missing: ${required}" >&2
        exit 1
    }
done

for value_name in SEED MAXITER POPSIZE; do
    value="${!value_name}"
    [[ "${value}" =~ ^[0-9]+$ ]] || {
        echo "error: ${value_name} must be a non-negative integer, got: ${value}" >&2
        exit 2
    }
done
(( MAXITER > 0 && POPSIZE > 0 )) || {
    echo "error: MAXITER and POPSIZE must be greater than zero" >&2
    exit 2
}
"${PYTHON_BIN}" - "${SIMULATION_DURATION_S}" <<'PY'
import sys
try:
    value = float(sys.argv[1])
except ValueError:
    raise SystemExit("error: SIMULATION_DURATION_S must be numeric")
if value <= 0:
    raise SystemExit("error: SIMULATION_DURATION_S must be greater than zero")
PY

"${PYTHON_BIN}" -c 'import cma, matplotlib, mujoco, numpy, pandas' || {
    echo "error: required Python packages are missing from ${PYTHON_BIN}." >&2
    echo "Activate the environment used for this project, then run the script again." >&2
    exit 1
}

STAMP="$(date +%Y-%m-%d_%H-%M-%S)"
RUN_ROOT="${SCRIPT_DIR}/results/aligned_four_gearboxes/run_${STAMP}_seed${SEED}"
mkdir -p "${RUN_ROOT}/.matplotlib"
export MPLCONFIGDIR="${MPLCONFIGDIR:-${RUN_ROOT}/.matplotlib}"

echo "Running test suite..."
(
    cd "${SCRIPT_DIR}"
    "${PYTHON_BIN}" -m unittest discover -s tests -v
)

if [[ -n "${ALIGNED_CATALOGUE_CSV:-}" ]]; then
    CATALOGUE_CSV="${ALIGNED_CATALOGUE_CSV}"
    [[ -f "${CATALOGUE_CSV}" ]] || {
        echo "error: ALIGNED_CATALOGUE_CSV does not exist: ${CATALOGUE_CSV}" >&2
        exit 1
    }
    echo "Using existing combined catalogue: ${CATALOGUE_CSV}"
else
    CATALOGUE_DIR="${RUN_ROOT}/catalogues"
    echo "Generating one MAD_M6C12_KV170 catalogue for all four gearbox runs..."
    "${PYTHON_BIN}" "${CATALOGUE_GENERATOR}" \
        --compact-repo "${COMPACT_REPO}" \
        --output-dir "${CATALOGUE_DIR}" \
        --workers 18
    CATALOGUE_CSV="${CATALOGUE_DIR}/combined_MAD_M6C12_KV170_catalogue.csv"
fi

"${PYTHON_BIN}" - "${CATALOGUE_CSV}" <<'PY'
import pandas as pd
import sys

path = sys.argv[1]
table = pd.read_csv(path)
required = {
    "gearbox_type", "requested_ratio", "actual_ratio", "mass_kg",
    "efficiency", "feasibility_status",
}
missing = sorted(required - set(table.columns))
if missing:
    raise SystemExit(f"error: incompatible catalogue {path}; missing columns: {missing}")
feasible = table[table["feasibility_status"].astype(str).str.lower() == "feasible"]
available = set(feasible["gearbox_type"].astype(str).str.upper())
missing_types = sorted({"SSPG", "CPG", "WPG", "DSPG"} - available)
if missing_types:
    raise SystemExit(f"error: catalogue has no feasible rows for: {missing_types}")
for gearbox in sorted(available):
    ratios = pd.to_numeric(
        feasible.loc[
            feasible["gearbox_type"].astype(str).str.upper() == gearbox,
            "requested_ratio",
        ],
        errors="coerce",
    ).dropna()
    if ratios.empty:
        raise SystemExit(f"error: catalogue has no numeric requested ratio for {gearbox}")
PY

echo "Aligned-monoped four-gearbox study"
echo "  output:              ${RUN_ROOT}"
echo "  catalogue:           ${CATALOGUE_CSV}"
echo "  motor:               MAD_M6C12_KV170"
echo "  gearboxes:           ${GEARBOXES[*]}"
echo "  CMA-ES generations:  ${MAXITER}"
echo "  population:          ${POPSIZE}"
echo "  seed:                ${SEED}"
echo "  simulation duration: ${SIMULATION_DURATION_S} s per candidate"
echo "  evaluations:         $((MAXITER * POPSIZE)) per gearbox"
echo "  total evaluations:   $((MAXITER * POPSIZE * ${#GEARBOXES[@]}))"

for gearbox in "${GEARBOXES[@]}"; do
    gearbox_lower="${gearbox,,}"
    RUN_DIR="${RUN_ROOT}/${gearbox_lower}"
    mkdir -p "${RUN_DIR}"

    echo "Starting ${gearbox} CMA-ES optimization..."
    "${PYTHON_BIN}" "${SCRIPT_DIR}/optimize_aligned.py" \
        --gearbox "${gearbox}" \
        --actuator-catalogue "${CATALOGUE_CSV}" \
        --motor-config "${MOTOR_CONFIG}" \
        --seed "${SEED}" \
        --maxiter "${MAXITER}" \
        --popsize "${POPSIZE}" \
        --simulation-duration "${SIMULATION_DURATION_S}" \
        --output-dir "${RUN_DIR}" \
        2>&1 | tee "${RUN_DIR}/run.log"

    if ! "${PYTHON_BIN}" - "${RUN_DIR}/best_design.json" <<'PY'
import json
import sys

with open(sys.argv[1]) as stream:
    result = json.load(stream)
raise SystemExit(0 if "cost" in result else 1)
PY
    then
        echo "warning: ${gearbox} produced no valid candidate; continuing." >&2
        continue
    fi

    if [[ "${RENDER_VIDEO}" == true ]]; then
        echo "Rendering the saved ${gearbox} result..."
        "${PYTHON_BIN}" "${SCRIPT_DIR}/replay_aligned.py" "${RUN_DIR}" --video
    fi
done

"${PYTHON_BIN}" "${SCRIPT_DIR}/compare_aligned_gearboxes.py" "${RUN_ROOT}"

echo "Complete."
echo "  Comparison: ${RUN_ROOT}/gearbox_comparison.csv"
echo "  Best result: ${RUN_ROOT}/best_overall.json"
