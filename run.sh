#!/bin/bash
# AM-D02 Pinocchio 参数辨识启动脚本
set -e
cd "$(dirname "${BASH_SOURCE[0]}")"

PRINT_BLUE() { echo -e "\033[34m$1\033[0m"; }
PRINT_GREEN() { echo -e "\033[32m$1\033[0m"; }
PRINT_RED() { echo -e "\033[31m$1\033[0m"; }

PYTHON_BIN=${AM_D02_PYTHON:-}

python_has_mujoco() {
    "$1" - <<'PY' >/dev/null 2>&1
import mujoco
PY
}

select_python() {
    if [ -n "$PYTHON_BIN" ]; then
        if command -v "$PYTHON_BIN" >/dev/null 2>&1; then
            PYTHON_BIN=$(command -v "$PYTHON_BIN")
        elif [ ! -x "$PYTHON_BIN" ]; then
            PRINT_RED "错误: AM_D02_PYTHON 指向的解释器不可执行: $PYTHON_BIN"
            exit 1
        fi
        return
    fi

    local candidates=()
    candidates+=("python3")
    candidates+=("python")
    candidates+=("$HOME/miniconda3/envs/dial-mpc-py310/bin/python")
    candidates+=("$HOME/miniconda3/bin/python")

    local candidate
    for candidate in "${candidates[@]}"; do
        if ! command -v "$candidate" >/dev/null 2>&1 && [ ! -x "$candidate" ]; then
            continue
        fi
        if python_has_mujoco "$candidate"; then
            PYTHON_BIN="$candidate"
            return
        fi
    done

    PYTHON_BIN=$(command -v python3 || true)
    if [ -z "$PYTHON_BIN" ]; then
        PRINT_RED "错误: 未找到 python3。"
        exit 1
    fi
}

show_main_menu() {
    echo "=========================================================="
    echo "            AM-D02 参数辨识启动状态机"
    echo "=========================================================="
    echo "请选择启动模式："
    echo "  1) param-id-sim  - 全参辨识笛卡尔阻抗闭环仿真"
    echo "  2) param-id-real - 全参辨识实机采集"
    echo "  3) payload-id-sim - 末端载荷辨识仿真"
    echo "  q) 退出"
    echo "----------------------------------------------------------"
}

select_mode_from_menu() {
    local choice
    while true; do
        show_main_menu
        read -r -p "输入数字选择模式: " choice
        case "$choice" in
            1) MODE="param-id-sim"; EXTRA_ARGS=(); break ;;
            2) MODE="param-id-real"; EXTRA_ARGS=(); break ;;
            3) MODE="payload-id-sim"; EXTRA_ARGS=(); break ;;
            q|Q) echo "已退出。"; exit 0 ;;
            *) PRINT_RED "无效选择: $choice" ;;
        esac
    done
}

show_available_modes() {
    echo "可用模式: param-id-sim, param-id-real, payload-id-sim"
}

# —— 参数解析 ——
MODE=${1:-}
EXTRA_ARGS=()
if [ -z "$MODE" ] || [[ "$MODE" == -* ]]; then
    select_mode_from_menu
else
    shift 2>/dev/null || true
    case "$MODE" in
        1|param-id-sim) MODE="param-id-sim" ;;
        2|param-id-real) MODE="param-id-real" ;;
        3|payload-id-sim) MODE="payload-id-sim" ;;
        *) PRINT_RED "错误: 未知模式 '$MODE'。"; show_available_modes; exit 1 ;;
    esac
fi
APP_ARGS=("${EXTRA_ARGS[@]}" "$@")
select_python
export PYTHONPATH="$PWD/src${PYTHONPATH:+:$PYTHONPATH}"

echo "=========================================================="
case "$MODE" in
    param-id-sim)  echo "    AM-D02 全参辨识笛卡尔阻抗闭环仿真               " ;;
    param-id-real) echo "    AM-D02 全参辨识实机采集                         " ;;
    payload-id-sim) echo "    AM-D02 末端载荷辨识仿真                         " ;;
esac
echo "=========================================================="
PRINT_BLUE "[System] Python 解释器: $PYTHON_BIN"

if [ "$MODE" == "param-id-sim" ]; then
    : "${AM_D02_ENABLE_VIEWER:=1}"
    : "${AM_D02_ENABLE_RERUN:=1}"
    export AM_D02_ENABLE_VIEWER AM_D02_ENABLE_RERUN
    PRINT_BLUE "[1/1] 启动全参辨识笛卡尔阻抗闭环仿真模式..."
    PRINT_BLUE "[Config] MuJoCo Viewer=$AM_D02_ENABLE_VIEWER, Rerun=$AM_D02_ENABLE_RERUN"
    echo "----------------------------------------------------------"
    "$PYTHON_BIN" -m robot_control.modes.param_id_sim.main "${APP_ARGS[@]}"

elif [ "$MODE" == "param-id-real" ]; then
    : "${AM_D02_ENABLE_VIEWER:=0}"
    : "${AM_D02_ENABLE_RERUN:=1}"
    : "${AM_D02_CAN_INTERFACE:=can0}"
    export AM_D02_ENABLE_VIEWER AM_D02_ENABLE_RERUN AM_D02_CAN_INTERFACE
    PRINT_BLUE "[1/1] 启动全参辨识实机模式..."
    echo "----------------------------------------------------------"
    "$PYTHON_BIN" -m robot_control.modes.param_id_real.main "${APP_ARGS[@]}"

else  # payload-id-sim
    : "${AM_D02_ENABLE_VIEWER:=0}"
    : "${AM_D02_ENABLE_RERUN:=0}"
    export AM_D02_ENABLE_VIEWER AM_D02_ENABLE_RERUN
    PRINT_BLUE "[1/1] 启动末端载荷辨识仿真模式..."
    echo "----------------------------------------------------------"
    "$PYTHON_BIN" -m robot_control.modes.payload_id_sim.main "${APP_ARGS[@]}"
fi
