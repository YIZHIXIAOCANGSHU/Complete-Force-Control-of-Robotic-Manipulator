#!/bin/bash
# AM-D02 Pinocchio 多模式统一启动脚本
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
    echo "            AM-D02 统一启动状态机"
    echo "=========================================================="
    echo "请选择启动模式："
    echo "  1) sim       - 软硬件联合仿真 (MuJoCo + Pinocchio)"
    echo "  2) real      - 真实硬件控制 (USB2FDCAN)"
    echo "  3) mc        - 蒙特卡罗范围检查 + MuJoCo 窗口"
    echo "  4) usbfdcan-sim - USB2FDCAN 反馈驱动仿真 + 全零发送"
    echo "  5) param_id  - 参数辨识 (质量/质心/惯量)"
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
            3) MODE="mc"; EXTRA_ARGS=(); break ;;
            4) MODE="usbfdcan-sim"; EXTRA_ARGS=(); break ;;
            5) MODE="param_id"; EXTRA_ARGS=(); select_param_id_mode; break ;;
            q|Q) echo "已退出。"; exit 0 ;;
            *) PRINT_RED "无效选择: $choice" ;;
        esac
    done
}

select_param_id_mode() {
    local choice
    while true; do
        echo "----------------------------------------------------------"
        echo "  参数辨识子模式："
        echo "    1) sim     - MuJoCo 开环逆动力学辨识"
        echo "    2) sim-pd  - MuJoCo PD闭环辨识"
        echo "    3) real    - 实机 USB2FDCAN 辨识"
        echo "    q) 返回"
        echo "----------------------------------------------------------"
        read -r -p "输入数字选择: " choice
        case "$choice" in
            1) PARAM_ID_MODE="sim"; break ;;
            2) PARAM_ID_MODE="sim-pd"; break ;;
            3) PARAM_ID_MODE="real"; break ;;
            q|Q) PARAM_ID_MODE=""; MODE=""; break ;;
            *) PRINT_RED "无效选择: $choice" ;;
        esac
    done
    if [ -z "$PARAM_ID_MODE" ]; then
        select_mode_from_menu
    fi
}

# —— 参数解析 ——
MODE=${1:-}
EXTRA_ARGS=()
PARAM_ID_MODE=""
if [ -z "$MODE" ] || [[ "$MODE" == -* ]]; then
    select_mode_from_menu
else
    shift 2>/dev/null || true
    case "$MODE" in
        1|sim) MODE="sim" ;;
        2|real) MODE="real" ;;
        3|mc) MODE="mc" ;;
        4|usbfdcan-sim|usb2fdcan-sim) MODE="usbfdcan-sim" ;;
        5|param_id)
            MODE="param_id"
            PARAM_ID_MODE="${1:-sim}"
            case "$PARAM_ID_MODE" in
                sim|sim-pd|real) shift 2>/dev/null || true ;;
                *) PRINT_RED "错误: param_id 子模式必须是 'sim', 'sim-pd' 或 'real'"; exit 1 ;;
            esac
            ;;
        *) PRINT_RED "错误: 未知模式 '$MODE'。可用: sim, real, mc, usbfdcan-sim, param_id"; exit 1 ;;
    esac
fi
APP_ARGS=("${EXTRA_ARGS[@]}" "$@")
select_python

echo "=========================================================="
case "$MODE" in
    sim)       echo "    AM-D02 软硬件联合仿真 (Pinocchio SITL)        " ;;
    mc)        echo "    AM-D02 蒙特卡罗末端位姿范围检查               " ;;
    usbfdcan-sim) echo "    AM-D02 USB2FDCAN 反馈驱动仿真                  " ;;
    real)      echo "    AM-D02 机械臂 USB2FDCAN 控制系统 (Real)        " ;;
    param_id)  echo "    AM-D02 参数辨识 (质量/质心/惯量)               " ;;
esac
echo "=========================================================="
PRINT_BLUE "[System] Python 解释器: $PYTHON_BIN"

# —— 直接启动 Python 脚本，无需 C 编译 ——
if [ "$MODE" == "sim" ]; then
    PRINT_BLUE "[1/2] 启动后台 Python MuJoCo 物理仿真服务器..."
    READY_FILE=$(mktemp /tmp/am_d02_server_ready.XXXXXX)
    "$PYTHON_BIN" python/sim/main_server.py --ready-file "$READY_FILE" "${APP_ARGS[@]}" &
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
    "$PYTHON_BIN" python/sim/pinocchio_sim_controller.py "${APP_ARGS[@]}"
    APP_STATUS=$?
    set -e
    exit $APP_STATUS

elif [ "$MODE" == "mc" ]; then
    : "${AM_D02_ENABLE_VIEWER:=0}"
    : "${AM_D02_ENABLE_RERUN:=0}"
    export AM_D02_ENABLE_VIEWER AM_D02_ENABLE_RERUN
    PRINT_BLUE "[1/1] 启动 MuJoCo 模型并随机采样关节空间..."
    echo "----------------------------------------------------------"
    "$PYTHON_BIN" python/mc/main.py "${APP_ARGS[@]}"

elif [ "$MODE" == "param_id" ]; then
    if [ "$PARAM_ID_MODE" == "sim" ]; then
        : "${AM_D02_ENABLE_VIEWER:=1}"
        : "${AM_D02_ENABLE_RERUN:=0}"
        export AM_D02_ENABLE_VIEWER AM_D02_ENABLE_RERUN
        PRINT_BLUE "[1/1] 启动参数辨识仿真模式（开环逆动力学）..."
        echo "----------------------------------------------------------"
        "$PYTHON_BIN" python/param_id/sim_main.py "${APP_ARGS[@]}"
    elif [ "$PARAM_ID_MODE" == "sim-pd" ]; then
        : "${AM_D02_ENABLE_VIEWER:=1}"
        : "${AM_D02_ENABLE_RERUN:=0}"
        export AM_D02_ENABLE_VIEWER AM_D02_ENABLE_RERUN
        PRINT_BLUE "[1/1] 启动参数辨识 PD闭环仿真模式..."
        echo "----------------------------------------------------------"
        "$PYTHON_BIN" python/param_id/sim_main_pd.py "${APP_ARGS[@]}"
    else
        : "${AM_D02_ENABLE_VIEWER:=0}"
        : "${AM_D02_ENABLE_RERUN:=1}"
        : "${AM_D02_CAN_INTERFACE:=can0}"
        export AM_D02_ENABLE_VIEWER AM_D02_ENABLE_RERUN AM_D02_CAN_INTERFACE
        PRINT_BLUE "[1/1] 启动参数辨识实机模式..."
        echo "----------------------------------------------------------"
        "$PYTHON_BIN" python/param_id/real_main.py "${APP_ARGS[@]}"
    fi

elif [ "$MODE" == "usbfdcan-sim" ]; then
    : "${AM_D02_CAN_INTERFACE:=can0}"
    : "${AM_D02_RERUN_LOG_STRIDE:=1}"
    : "${AM_D02_REAL_VIEWER_FPS:=30}"
    : "${AM_D02_RERUN_QUEUE_SIZE:=512}"
    export AM_D02_CAN_INTERFACE AM_D02_RERUN_LOG_STRIDE AM_D02_REAL_VIEWER_FPS AM_D02_RERUN_QUEUE_SIZE
    PRINT_BLUE "[1/1] 检查 SocketCAN 接口 ${AM_D02_CAN_INTERFACE}..."
    if [ -e "/sys/class/net/${AM_D02_CAN_INTERFACE}" ]; then
        CAN_STATE=$(cat "/sys/class/net/${AM_D02_CAN_INTERFACE}/operstate" 2>/dev/null || true)
        if [ "$CAN_STATE" != "up" ]; then
            PRINT_RED "警告: ${AM_D02_CAN_INTERFACE} 状态为 '${CAN_STATE:-unknown}'"
        fi
    else
        PRINT_RED "警告: 未检测到 SocketCAN 接口 ${AM_D02_CAN_INTERFACE}"
    fi
    echo "----------------------------------------------------------"
    "$PYTHON_BIN" python/mirror/main.py "${APP_ARGS[@]}"

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
    PRINT_GREEN "[1/1] 启动 Python SocketCAN USB2FDCAN 控制回路..."
    echo "----------------------------------------------------------"
    "$PYTHON_BIN" python/real/usb2fdcan_control.py "${APP_ARGS[@]}"
fi
