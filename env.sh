#!/usr/bin/env bash
# Project-local Python environment helper.
#
# Usage:
#   source ./env.sh              # activate .venv in the current shell
#   ./env.sh pytest              # run pytest with project-safe settings
#   ./env.sh python script.py    # run any command inside .venv

set -euo pipefail

PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
VENV_DIR="${PROJECT_ROOT}/.venv"
ACTIVATE="${VENV_DIR}/bin/activate"

if [[ ! -f "${ACTIVATE}" ]]; then
    echo "[env] Missing virtual environment: ${VENV_DIR}" >&2
    echo "[env] Create it first:" >&2
    echo "      python3 -m venv .venv" >&2
    echo "      source .venv/bin/activate" >&2
    echo "      python -m pip install -U pip" >&2
    echo "      python -m pip install -e . \"pytest>=8,<9\"" >&2
    return 1 2>/dev/null || exit 1
fi

cd "${PROJECT_ROOT}"
source "${ACTIVATE}"

# Keep project tests isolated from ROS/MoveIt/global pytest plugins and avoid
# refreshing tracked __pycache__ files during normal test runs.
export PYTEST_DISABLE_PLUGIN_AUTOLOAD="${PYTEST_DISABLE_PLUGIN_AUTOLOAD:-1}"
export PYTHONDONTWRITEBYTECODE="${PYTHONDONTWRITEBYTECODE:-1}"
export PYTHONPATH="${PROJECT_ROOT}/src${PYTHONPATH:+:${PYTHONPATH}}"

echo "[env] Activated: ${VIRTUAL_ENV}"
echo "[env] Python: $(command -v python)"
echo "[env] PYTEST_DISABLE_PLUGIN_AUTOLOAD=${PYTEST_DISABLE_PLUGIN_AUTOLOAD}"
echo "[env] PYTHONDONTWRITEBYTECODE=${PYTHONDONTWRITEBYTECODE}"
echo "[env] PYTHONPATH=${PYTHONPATH}"

if [[ "${BASH_SOURCE[0]}" == "${0}" ]]; then
    if [[ "$#" -gt 0 ]]; then
        exec "$@"
    fi
    echo
    echo "[env] Tip: use 'source ./env.sh' to keep this venv active in your current shell."
fi
