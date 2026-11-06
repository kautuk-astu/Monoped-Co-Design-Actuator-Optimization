#!/usr/bin/env bash
# Run the four-gearbox end-to-end study for five seeds concurrently.
set -Eeuo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd -- "${SCRIPT_DIR}/.." && pwd)"
PYTHON_BIN="${PYTHON_BIN:-python}"
TARGET="${SCRIPT_DIR}/run_aligned_end_to_end.sh"
PLOTTER="${SCRIPT_DIR}/plot_aligned_seed_heights.py"

# Override with, for example: SEEDS="1 2 3 4 5"
read -r -a SEED_VALUES <<< "${SEEDS:-5 6 7 8 9}"

[[ ${#SEED_VALUES[@]} -eq 5 ]] || {
    echo "error: SEEDS must contain exactly five integers" >&2
    exit 2
}
declare -A SEEN_SEEDS=()
for seed in "${SEED_VALUES[@]}"; do
    [[ "${seed}" =~ ^[0-9]+$ ]] || {
        echo "error: invalid seed: ${seed}" >&2
        exit 2
    }
    [[ -z "${SEEN_SEEDS[${seed}]:-}" ]] || {
        echo "error: duplicate seed: ${seed}" >&2
        exit 2
    }
    SEEN_SEEDS["${seed}"]=1
done

[[ -x "${TARGET}" ]] || {
    echo "error: launcher is missing or not executable: ${TARGET}" >&2
    exit 1
}
[[ -f "${PLOTTER}" ]] || {
    echo "error: result plotter is missing: ${PLOTTER}" >&2
    exit 1
}
command -v setsid >/dev/null 2>&1 || {
    echo "error: setsid is required for clean parallel-process shutdown" >&2
    exit 1
}

if [[ -n "${ALIGNED_CATALOGUE_CSV:-}" ]]; then
    CATALOGUE_CSV="${ALIGNED_CATALOGUE_CSV}"
else
    CATALOGUE_CSV="$("${PYTHON_BIN}" - "${REPO_ROOT}/compact_catalogue_generation/outputs" <<'PY'
from pathlib import Path
import sys

root = Path(sys.argv[1])
candidates = list(root.glob("run_*/combined_MAD_M6C12_KV170_catalogue.csv"))
if candidates:
    print(max(candidates, key=lambda path: path.stat().st_mtime).resolve())
PY
)"
fi
[[ -n "${CATALOGUE_CSV}" && -f "${CATALOGUE_CSV}" ]] || {
    echo "error: no existing combined MAD_M6C12_KV170 catalogue was found" >&2
    echo "Set ALIGNED_CATALOGUE_CSV=/absolute/path/to/catalogue.csv." >&2
    exit 1
}

STAMP="$(date +%Y-%m-%d_%H-%M-%S)"
LOG_ROOT="${SCRIPT_DIR}/results/aligned_parallel_launches/run_${STAMP}"
mkdir -p "${LOG_ROOT}"

declare -a PIDS=()
declare -A PID_TO_SEED=()

terminate_children() {
    trap - INT TERM
    echo "Stopping parallel seed runs..." >&2
    for pid in "${PIDS[@]}"; do
        if kill -0 "${pid}" 2>/dev/null; then
            kill -TERM -- "-${pid}" 2>/dev/null || kill -TERM "${pid}" 2>/dev/null || true
        fi
    done
    for pid in "${PIDS[@]}"; do
        wait "${pid}" 2>/dev/null || true
    done
    exit 130
}
trap terminate_children INT TERM

echo "Parallel aligned study"
echo "  seeds:               ${SEED_VALUES[*]}"
echo "  concurrent runs:     ${#SEED_VALUES[@]}"
echo "  gearboxes per seed:  SSPG CPG WPG DSPG"
echo "  catalogue:           ${CATALOGUE_CSV}"
echo "  launcher logs:       ${LOG_ROOT}"
echo "  generations:         ${MAXITER:-1000}"
echo "  population:          ${POPSIZE:-8}"
echo "  simulation duration: ${SIMULATION_DURATION_S:-10} s per candidate"

for seed in "${SEED_VALUES[@]}"; do
    log_file="${LOG_ROOT}/seed_${seed}.log"
    echo "Starting seed ${seed}; log: ${log_file}"
    setsid env \
        SEED="${seed}" \
        MAXITER="${MAXITER:-1000}" \
        POPSIZE="${POPSIZE:-8}" \
        SIMULATION_DURATION_S="${SIMULATION_DURATION_S:-10}" \
        ALIGNED_CATALOGUE_CSV="${CATALOGUE_CSV}" \
        PYTHON_BIN="${PYTHON_BIN}" \
        PYTHONDONTWRITEBYTECODE=1 \
        OMP_NUM_THREADS=1 \
        OPENBLAS_NUM_THREADS=1 \
        MKL_NUM_THREADS=1 \
        NUMEXPR_NUM_THREADS=1 \
        bash "${TARGET}" >"${log_file}" 2>&1 &
    pid=$!
    PIDS+=("${pid}")
    PID_TO_SEED["${pid}"]="${seed}"
done

failed=0
for pid in "${PIDS[@]}"; do
    seed="${PID_TO_SEED[${pid}]}"
    if wait "${pid}"; then
        echo "Seed ${seed} completed successfully."
    else
        status=$?
        echo "Seed ${seed} failed with exit status ${status}; see ${LOG_ROOT}/seed_${seed}.log" >&2
        failed=1
    fi
done
trap - INT TERM

if (( failed )); then
    echo "One or more seed runs failed. Successful seed results were preserved." >&2
    exit 1
fi

"${PYTHON_BIN}" "${PLOTTER}" "${LOG_ROOT}"
echo "All five seed runs completed."
echo "  Logs:    ${LOG_ROOT}"
echo "  Results: ${SCRIPT_DIR}/results/aligned_four_gearboxes"
