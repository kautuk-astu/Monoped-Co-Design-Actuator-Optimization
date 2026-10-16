#!/usr/bin/env bash
# Fixed 0.20 m/MAD_M6C12 study with exactly one powered jump.
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
exec "${SCRIPT_DIR}/run_cmaes_fixed_link_motor.sh" --single-jump "$@"
