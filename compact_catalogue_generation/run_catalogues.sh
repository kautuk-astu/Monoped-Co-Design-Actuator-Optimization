#!/usr/bin/env bash
set -Eeuo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
DEFAULT_COMPACT_REPO="$(cd -- "${SCRIPT_DIR}/../.." && pwd)/COMPAct"
COMPACT_REPO="${COMPACT_REPO:-${DEFAULT_COMPACT_REPO}}"
PYTHON_BIN="${PYTHON_BIN:-python3}"
OUTPUT_ROOT="${SCRIPT_DIR}/outputs"
RUN_ID="run_$(date -u +%Y-%m-%d_%H-%M-%S_UTC)"
OUTPUT_DIR="${OUTPUT_ROOT}/${RUN_ID}"

if [[ ! -f "${COMPACT_REPO}/actOpt.py" ]]; then
    echo "COMPAct was not found at: ${COMPACT_REPO}" >&2
    echo "Set COMPACT_REPO=/absolute/path/to/COMPAct and run again." >&2
    exit 2
fi

if ! command -v "${PYTHON_BIN}" >/dev/null 2>&1; then
    echo "Python executable not found: ${PYTHON_BIN}" >&2
    exit 2
fi

if ! "${PYTHON_BIN}" -c 'import numpy' >/dev/null 2>&1; then
    echo "The selected Python environment needs NumPy for COMPAct." >&2
    echo "Install NumPy or set PYTHON_BIN to a compatible Python executable." >&2
    exit 2
fi

mkdir -p -- "${OUTPUT_ROOT}"

echo "Starting COMPAct catalogue generation"
echo "  COMPAct: ${COMPACT_REPO}"
echo "  Output:  ${OUTPUT_DIR}"
echo "  Workers: 18"

"${PYTHON_BIN}" "${SCRIPT_DIR}/generate_catalogues.py" \
    --compact-repo "${COMPACT_REPO}" \
    --output-dir "${OUTPUT_DIR}" \
    --workers 18

echo "Catalogue generation and consolidation completed: ${OUTPUT_DIR}"
