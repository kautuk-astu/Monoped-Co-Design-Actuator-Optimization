#!/usr/bin/env bash
# Run saved monoped paper cases (or the saved controller-fixed ablation).
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
PYTHON_BIN="${PYTHON_BIN:-python3}"

usage() {
    cat <<'EOF'
Usage: ./run_case.sh {nominal|caseA|caseB|caseC|controller-fixed|all} [--no-video]
       ./run_case.sh {nominal|caseA|caseB|caseC|controller-fixed} --optimize

Default operation replays the repository's saved parameters.  --optimize is
the only mode that starts the original CMA-ES scripts.
EOF
}

if [[ $# -eq 0 || "$1" == "-h" || "$1" == "--help" ]]; then
    usage
    exit 0
fi

case_name="$1"
shift
optimize=false
helper_args=()
for argument in "$@"; do
    case "$argument" in
        --optimize) optimize=true ;;
        --no-video) helper_args+=("$argument") ;;
        *) echo "Unknown option: $argument" >&2; usage >&2; exit 2 ;;
    esac
done

case "$case_name" in
    nominal) helper_case="nominal"; optimizer="cmaes_baseline.py" ;;
    caseA|casea) helper_case="casea"; optimizer="cmaes_ll.py" ;;
    caseB|caseb) helper_case="caseb"; optimizer="cmaes_gear.py" ;;
    caseC|casec) helper_case="casec"; optimizer="cmaes.py" ;;
    controller-fixed|controller_fixed|ctrlfixed) helper_case="controller-fixed"; optimizer="cmaes_ctrlfixed.py" ;;
    all) helper_case="all"; optimizer="" ;;
    *) echo "Unknown case: $case_name" >&2; usage >&2; exit 2 ;;
esac

command -v "$PYTHON_BIN" >/dev/null || {
    echo "Python interpreter not found: $PYTHON_BIN" >&2
    exit 1
}

mkdir -p "$SCRIPT_DIR/results/videos" "$SCRIPT_DIR/results/run_cases"

if [[ "$optimize" == true ]]; then
    if [[ "$helper_case" == "all" ]]; then
        echo "--optimize requires one case; it intentionally does not start every CMA-ES study." >&2
        exit 2
    fi
    "$PYTHON_BIN" -c 'import cma, mujoco, numpy, pandas, scipy'
    echo "Starting the original CMA-ES optimizer for $case_name (this is not saved-parameter playback)."
    cd "$SCRIPT_DIR/components"
    exec "$PYTHON_BIN" "$optimizer"
fi

"$PYTHON_BIN" "$SCRIPT_DIR/components/run_saved_case.py" --check-deps
exec "$PYTHON_BIN" "$SCRIPT_DIR/components/run_saved_case.py" "$helper_case" "${helper_args[@]}"
