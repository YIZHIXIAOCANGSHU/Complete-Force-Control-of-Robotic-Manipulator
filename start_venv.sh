#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
VENV_DIR="$SCRIPT_DIR/.venv"
ACTIVATE_FILE="$VENV_DIR/bin/activate"

if [ ! -f "$ACTIVATE_FILE" ]; then
    echo "未找到虚拟环境: $VENV_DIR"
    echo "正在创建 .venv 并安装项目依赖..."
    python3 -m venv "$VENV_DIR"
    "$VENV_DIR/bin/python" -m pip install --upgrade pip setuptools wheel

    if [ -f "$SCRIPT_DIR/python/requirements.txt" ]; then
        "$VENV_DIR/bin/python" -m pip install -r "$SCRIPT_DIR/python/requirements.txt"
    fi
fi

export AM_D02_PYTHON="$VENV_DIR/bin/python"

if [[ "${BASH_SOURCE[0]}" != "$0" ]]; then
    # Being sourced: activate the caller's current shell.
    # shellcheck disable=SC1090
    source "$ACTIVATE_FILE"
    echo "已激活虚拟环境: $VIRTUAL_ENV"
    python --version
else
    # Being executed: open a new interactive shell with the venv already active.
    TMP_RC="$(mktemp)"
    trap 'rm -f "$TMP_RC"' EXIT

    {
        printf 'source %q\n' "$ACTIVATE_FILE"
        printf 'export AM_D02_PYTHON=%q\n' "$AM_D02_PYTHON"
        printf 'echo "已激活虚拟环境: $VIRTUAL_ENV"\n'
        printf 'python --version\n'
        printf 'echo "退出虚拟环境 shell: exit 或 Ctrl-D"\n'
    } > "$TMP_RC"

    bash --rcfile "$TMP_RC" -i
fi
