#!/bin/bash
# AM-D02 Pinocchio 四模式统一启动脚本
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
    echo "            AM-D02 四模式统一启动状态机"
    echo "=========================================================="
    echo "请选择启动模式："
    echo "  1) sim           - 真机控制仿真 (MuJoCo + Pinocchio)"
    echo "  2) real          - 真实硬件控制 (serial / USB2FDCAN)"
    echo "  3) param-id-sim  - 全参辨识 PD 闭环仿真"
    echo "  4) param-id-real - 全参辨识实机采集"
    echo "  q) 退出"
    echo "----------------------------------------------------------"
}

select_mode_from_menu() {
    local choice
    while true; do
        show_main_menu
        read -r -p "输入数字选择模式: " choice
        case "$choice" in
            1) MODE="sim"; EXTRA_ARGS=(); break ;;
            2) MODE="real"; EXTRA_ARGS=(); break ;;
            3) MODE="param-id-sim"; EXTRA_ARGS=(); break ;;
            4) MODE="param-id-real"; EXTRA_ARGS=(); break ;;
            q|Q) echo "已退出。"; exit 0 ;;
            *) PRINT_RED "无效选择: $choice" ;;
        esac
    done
}

show_available_modes() {
    echo "可用模式: sim, real, param-id-sim, param-id-real"
}

reject_removed_mode() {
    PRINT_RED "错误: 模式 '$1' 已删除。"
    show_available_modes
    exit 1
}

# —— 参数解析 ——
MODE=${1:-}
EXTRA_ARGS=()
if [ -z "$MODE" ] || [[ "$MODE" == -* ]]; then
    select_mode_from_menu
else
    shift 2>/dev/null || true
    case "$MODE" in
        1|sim) MODE="sim" ;;
        2|real) MODE="real" ;;
        3|param-id-sim) MODE="param-id-sim" ;;
        4|param-id-real) MODE="param-id-real" ;;
        mc|monte-carlo|usbfdcan-sim|usb2fdcan-sim|mirror|param_id|param-id|param_id_sim|param_id_real) reject_removed_mode "$MODE" ;;
        *) PRINT_RED "错误: 未知模式 '$MODE'。"; show_available_modes; exit 1 ;;
    esac
fi
APP_ARGS=("${EXTRA_ARGS[@]}" "$@")
select_python
export PYTHONPATH="$PWD/src${PYTHONPATH:+:$PYTHONPATH}"

echo "=========================================================="
case "$MODE" in
    sim)           echo "    AM-D02 真机控制仿真 (Pinocchio SITL)            " ;;
    real)          echo "    AM-D02 真实硬件控制 (Real)                      " ;;
    param-id-sim)  echo "    AM-D02 全参辨识 PD 闭环仿真                     " ;;
    param-id-real) echo "    AM-D02 全参辨识实机采集                         " ;;
esac
echo "=========================================================="
PRINT_BLUE "[System] Python 解释器: $PYTHON_BIN"

if [ "$MODE" == "sim" ]; then
    PRINT_BLUE "[1/2] 启动后台 Python MuJoCo 物理仿真服务器..."
    READY_FILE=$(mktemp /tmp/am_d02_server_ready.XXXXXX)
    "$PYTHON_BIN" -m robot_control.modes.control_sim.main --ready-file "$READY_FILE" "${APP_ARGS[@]}" &
    SERVER_PID=$!

    cleanup() {
        if [ ! -z "${SERVER_PID:-}" ] && kill -0 "$SERVER_PID" 2>/dev/null; then
            kill "$SERVER_PID" 2>/dev/null || true
            wait "$SERVER_PID" 2>/dev/null || true
        fi
        if [ ! -z "${READY_FILE:-}" ]; then rm -f "$READY_FILE"; fi
    }
    on_signal() {
        echo -e "\n[Shutdown] 正在关闭后台仿真服务器..."
        cleanup
        echo "[Shutdown] 仿真会话已结束。"
        exit 0
    }
    trap on_signal SIGINT SIGTERM
    trap cleanup EXIT

    for _ in $(seq 1 200); do
        if [ -f "$READY_FILE" ]; then break; fi
        if ! kill -0 "$SERVER_PID" 2>/dev/null; then
            PRINT_RED "错误: Python 仿真服务器在就绪前已退出。"
            wait "$SERVER_PID" || true
            exit 1
        fi
        sleep 0.1
    done
    if [ ! -f "$READY_FILE" ]; then
        PRINT_RED "错误: 等待 Python 仿真服务器就绪超时。"
        exit 1
    fi

    PRINT_GREEN "[2/2] 启动 Python Pinocchio 仿真控制器..."
    echo "----------------------------------------------------------"
    set +e
    "$PYTHON_BIN" -m robot_control.modes.control_sim.pinocchio_controller "${APP_ARGS[@]}"
    APP_STATUS=$?
    set -e
    exit $APP_STATUS

elif [ "$MODE" == "param-id-sim" ]; then
    : "${AM_D02_ENABLE_VIEWER:=1}"
    : "${AM_D02_ENABLE_RERUN:=0}"
    export AM_D02_ENABLE_VIEWER AM_D02_ENABLE_RERUN
    PRINT_BLUE "[1/1] 启动全参辨识 PD 闭环仿真模式..."
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

else  # real
    : "${AM_D02_CAN_INTERFACE:=can0}"
    : "${AM_D02_RERUN_LOG_STRIDE:=25}"
    : "${AM_D02_REAL_VIEWER_FPS:=30}"
    : "${AM_D02_RERUN_QUEUE_SIZE:=512}"
    export AM_D02_CAN_INTERFACE AM_D02_RERUN_LOG_STRIDE AM_D02_REAL_VIEWER_FPS AM_D02_RERUN_QUEUE_SIZE
    PRINT_BLUE "[System] 检查 SocketCAN 接口 ${AM_D02_CAN_INTERFACE}..."
    if [ -e "/sys/class/net/${AM_D02_CAN_INTERFACE}" ]; then
        CAN_STATE=$(cat "/sys/class/net/${AM_D02_CAN_INTERFACE}/operstate" 2>/dev/null || true)
        if [ "$CAN_STATE" != "up" ]; then
            PRINT_RED "警告: ${AM_D02_CAN_INTERFACE} 状态为 '${CAN_STATE:-unknown}'"
        fi
    else
        PRINT_RED "警告: 未检测到 SocketCAN 接口 ${AM_D02_CAN_INTERFACE}"
    fi
    PRINT_GREEN "[1/1] 启动 Python 真实硬件控制回路..."
    echo "----------------------------------------------------------"
    "$PYTHON_BIN" -m robot_control.modes.control_real.main "${APP_ARGS[@]}"
fi
