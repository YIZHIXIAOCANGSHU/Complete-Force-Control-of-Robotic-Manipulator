#!/bin/bash
set -euo pipefail

cd "$(dirname "${BASH_SOURCE[0]}")/.."

PYTHON_BASE=${AM_D02_VENV_PYTHON:-python3}
VENV_DIR=${AM_D02_VENV_DIR:-.venv}

if ! command -v "$PYTHON_BASE" >/dev/null 2>&1 && [ ! -x "$PYTHON_BASE" ]; then
    echo "错误: 未找到用于创建 venv 的 Python: $PYTHON_BASE" >&2
    exit 1
fi

"$PYTHON_BASE" -m venv --system-site-packages "$VENV_DIR"
"$VENV_DIR/bin/python" -m pip install --upgrade pip
"$VENV_DIR/bin/python" -m pip install -r python/requirements.txt

echo "VENV ready: $PWD/$VENV_DIR"
echo "Python: $("$VENV_DIR/bin/python" -c 'import sys; print(sys.executable)')"
