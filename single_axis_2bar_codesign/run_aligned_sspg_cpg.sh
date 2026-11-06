#!/usr/bin/env bash
# End-to-end aligned study restricted to SSPG and CPG.
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
GEARBOXES=(SSPG CPG)

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

MOTOR_CONFIG="${COMPACT_REPO}/config_files/motors.json"
for required in \
    "${MOTOR_CONFIG}" \
    "${SCRIPT_DIR}/optimize_aligned.py" \
    "${SCRIPT_DIR}/plot_aligned_seed_heights.py" \
    "${SCRIPT_DIR}/replay_aligned.py" \
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

if [[ "${ALIGNED_SSPG_CPG_SINGLE_SEED:-0}" != "1" ]]; then
    read -r -a SEED_VALUES <<< "${SEEDS:-5 6 7 8 9}"
    [[ ${#SEED_VALUES[@]} -eq 5 ]] || {
        echo "error: SEEDS must contain exactly five integers" >&2
        exit 2
    }
    declare -A SEEN_SEEDS=()
    for parallel_seed in "${SEED_VALUES[@]}"; do
        [[ "${parallel_seed}" =~ ^[0-9]+$ ]] || {
            echo "error: invalid seed: ${parallel_seed}" >&2
            exit 2
        }
        [[ -z "${SEEN_SEEDS[${parallel_seed}]:-}" ]] || {
            echo "error: duplicate seed: ${parallel_seed}" >&2
            exit 2
        }
        SEEN_SEEDS["${parallel_seed}"]=1
    done
    command -v setsid >/dev/null 2>&1 || {
        echo "error: setsid is required for clean parallel-process shutdown" >&2
        exit 1
    }

    if [[ -n "${ALIGNED_CATALOGUE_CSV:-}" ]]; then
        SHARED_CATALOGUE_CSV="${ALIGNED_CATALOGUE_CSV}"
    else
        SHARED_CATALOGUE_CSV="$("${PYTHON_BIN}" - "${REPO_ROOT}/compact_catalogue_generation/outputs" <<'PY'
from pathlib import Path
import sys

root = Path(sys.argv[1])
candidates = list(root.glob("run_*/combined_MAD_M6C12_KV170_catalogue.csv"))
if candidates:
    print(max(candidates, key=lambda path: path.stat().st_mtime).resolve())
PY
)"
    fi
    [[ -n "${SHARED_CATALOGUE_CSV}" && -f "${SHARED_CATALOGUE_CSV}" ]] || {
        echo "error: no existing combined MAD_M6C12_KV170 catalogue was found" >&2
        echo "Set ALIGNED_CATALOGUE_CSV=/absolute/path/to/catalogue.csv." >&2
        exit 1
    }

    echo "Running test suite once before launching the five seeds..."
    (
        cd "${SCRIPT_DIR}"
        "${PYTHON_BIN}" -m unittest discover -s tests -v
    )

    PARALLEL_STAMP="$(date +%Y-%m-%d_%H-%M-%S)"
    PARALLEL_LOG_ROOT="${SCRIPT_DIR}/results/aligned_sspg_cpg_parallel/run_${PARALLEL_STAMP}"
    mkdir -p "${PARALLEL_LOG_ROOT}"
    declare -a PARALLEL_PIDS=()
    declare -A PID_TO_SEED=()

    stop_parallel_runs() {
        trap - INT TERM
        echo "Stopping parallel SSPG/CPG seed runs..." >&2
        for parallel_pid in "${PARALLEL_PIDS[@]}"; do
            if kill -0 "${parallel_pid}" 2>/dev/null; then
                kill -TERM -- "-${parallel_pid}" 2>/dev/null || \
                    kill -TERM "${parallel_pid}" 2>/dev/null || true
            fi
        done
        for parallel_pid in "${PARALLEL_PIDS[@]}"; do
            wait "${parallel_pid}" 2>/dev/null || true
        done
        exit 130
    }
    trap stop_parallel_runs INT TERM

    echo "Parallel SSPG/CPG study"
    echo "  seeds:               ${SEED_VALUES[*]}"
    echo "  concurrent runs:     ${#SEED_VALUES[@]}"
    echo "  gearboxes per seed:  SSPG CPG"
    echo "  catalogue:           ${SHARED_CATALOGUE_CSV}"
    echo "  launcher logs:       ${PARALLEL_LOG_ROOT}"
    echo "  generations:         ${MAXITER}"
    echo "  population:          ${POPSIZE}"
    echo "  simulation duration: ${SIMULATION_DURATION_S} s per candidate"

    CHILD_ARGS=()
    if [[ "${RENDER_VIDEO}" == true ]]; then
        echo "warning: parallel video rendering may contend for GPU resources" >&2
        CHILD_ARGS+=(--video)
    fi

    for parallel_seed in "${SEED_VALUES[@]}"; do
        seed_log="${PARALLEL_LOG_ROOT}/seed_${parallel_seed}.log"
        echo "Starting seed ${parallel_seed}; log: ${seed_log}"
        setsid env \
            ALIGNED_SSPG_CPG_SINGLE_SEED=1 \
            ALIGNED_CATALOGUE_CSV="${SHARED_CATALOGUE_CSV}" \
            SEED="${parallel_seed}" \
            MAXITER="${MAXITER}" \
            POPSIZE="${POPSIZE}" \
            SIMULATION_DURATION_S="${SIMULATION_DURATION_S}" \
            PYTHON_BIN="${PYTHON_BIN}" \
            PYTHONDONTWRITEBYTECODE=1 \
            OMP_NUM_THREADS=1 \
            OPENBLAS_NUM_THREADS=1 \
            MKL_NUM_THREADS=1 \
            NUMEXPR_NUM_THREADS=1 \
            bash "${SCRIPT_DIR}/run_aligned_sspg_cpg.sh" "${CHILD_ARGS[@]}" >"${seed_log}" 2>&1 &
        parallel_pid=$!
        PARALLEL_PIDS+=("${parallel_pid}")
        PID_TO_SEED["${parallel_pid}"]="${parallel_seed}"
    done

    parallel_failed=0
    for parallel_pid in "${PARALLEL_PIDS[@]}"; do
        parallel_seed="${PID_TO_SEED[${parallel_pid}]}"
        if wait "${parallel_pid}"; then
            echo "Seed ${parallel_seed} completed successfully."
        else
            parallel_status=$?
            echo "Seed ${parallel_seed} failed with exit status ${parallel_status}; see ${PARALLEL_LOG_ROOT}/seed_${parallel_seed}.log" >&2
            parallel_failed=1
        fi
    done
    trap - INT TERM

    if (( parallel_failed )); then
        echo "One or more seed runs failed. Successful results were preserved." >&2
        exit 1
    fi
    "${PYTHON_BIN}" "${SCRIPT_DIR}/plot_aligned_seed_heights.py" "${PARALLEL_LOG_ROOT}"
    echo "All five SSPG/CPG seed runs completed."
    echo "  Logs:    ${PARALLEL_LOG_ROOT}"
    echo "  Results: ${SCRIPT_DIR}/results/aligned_sspg_cpg"
    exit 0
fi

STAMP="$(date +%Y-%m-%d_%H-%M-%S)"
RUN_ROOT="${SCRIPT_DIR}/results/aligned_sspg_cpg/run_${STAMP}_seed${SEED}"
mkdir -p "${RUN_ROOT}/.matplotlib"
export MPLCONFIGDIR="${MPLCONFIGDIR:-${RUN_ROOT}/.matplotlib}"

if [[ -n "${ALIGNED_CATALOGUE_CSV:-}" ]]; then
    CATALOGUE_CSV="${ALIGNED_CATALOGUE_CSV}"
    [[ -f "${CATALOGUE_CSV}" ]] || {
        echo "error: ALIGNED_CATALOGUE_CSV does not exist: ${CATALOGUE_CSV}" >&2
        exit 1
    }
    echo "Using existing combined catalogue: ${CATALOGUE_CSV}"
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
    [[ -n "${CATALOGUE_CSV}" && -f "${CATALOGUE_CSV}" ]] || {
        echo "error: no existing combined MAD_M6C12_KV170 catalogue was found" >&2
        echo "Set ALIGNED_CATALOGUE_CSV=/absolute/path/to/catalogue.csv." >&2
        exit 1
    }
    echo "Using newest existing combined catalogue: ${CATALOGUE_CSV}"
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
for gearbox in ("SSPG", "CPG"):
    rows = feasible[
        feasible["gearbox_type"].astype(str).str.upper() == gearbox
    ]
    ratios = pd.to_numeric(rows["requested_ratio"], errors="coerce").dropna()
    if ratios.empty:
        raise SystemExit(f"error: catalogue has no feasible numeric ratio for {gearbox}")
PY

echo "Aligned-monoped SSPG/CPG study"
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

    if [[ "${RENDER_VIDEO}" == true ]] && \
            "${PYTHON_BIN}" -c \
            'import json,sys; raise SystemExit(0 if "cost" in json.load(open(sys.argv[1])) else 1)' \
            "${RUN_DIR}/best_design.json"; then
        echo "Rendering the saved ${gearbox} result..."
        "${PYTHON_BIN}" "${SCRIPT_DIR}/replay_aligned.py" "${RUN_DIR}" --video
    fi
done

"${PYTHON_BIN}" - "${RUN_ROOT}" <<'PY'
import csv
import json
from pathlib import Path
import sys

root = Path(sys.argv[1])
rows = []
for gearbox in ("SSPG", "CPG"):
    run_dir = root / gearbox.lower()
    path = run_dir / "best_design.json"
    result = json.loads(path.read_text()) if path.is_file() else {}
    rows.append({
        "gearbox": gearbox,
        "valid_candidate": "cost" in result,
        "cost": result.get("cost", ""),
        "hip_actual_ratio": result.get("hip_gear_ratio", ""),
        "knee_actual_ratio": result.get("knee_gear_ratio", ""),
        "run_directory": str(run_dir.resolve()),
    })

with (root / "gearbox_comparison.csv").open("w", newline="") as stream:
    writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
    writer.writeheader()
    writer.writerows(rows)
valid = [row for row in rows if row["valid_candidate"]]
winner = min(valid, key=lambda row: float(row["cost"])) if valid else None
(root / "best_overall.json").write_text(
    json.dumps({"best_overall": winner, "all_gearboxes": rows}, indent=2) + "\n"
)
if winner:
    print(f"Best gearbox: {winner['gearbox']} (cost={float(winner['cost']):.6f})")
else:
    print("Neither SSPG nor CPG produced a valid candidate.")
PY

echo "Complete."
echo "  Comparison: ${RUN_ROOT}/gearbox_comparison.csv"
echo "  Best result: ${RUN_ROOT}/best_overall.json"
