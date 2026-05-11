#!/bin/bash
# AM-D02 SITL 软硬件联合仿真与实机控制统一启动脚本
set -e
cd "$(dirname "${BASH_SOURCE[0]}")"

# 颜色定义
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
    echo "  1) sim  - 软硬件联合仿真"
    echo "  2) real - 真实硬件控制"
    echo "  3) mc   - 蒙特卡罗范围检查 + MuJoCo 窗口"
    echo "  4) usbfdcan-sim - USB2FDCAN 反馈驱动仿真 + 全零发送"
    echo "  q) 退出"
    echo "----------------------------------------------------------"
}

show_real_backend_menu() {
    echo "=========================================================="
    echo "            AM-D02 Real 后端选择"
    echo "=========================================================="
    echo "请选择真实硬件链路："
    echo "  1) serial   - 旧串口下位机链路"
    echo "  2) usbfdcan - SocketCAN USB2FDCAN 达妙 MIT torque"
    echo "  q) 返回/退出"
    echo "----------------------------------------------------------"
}

select_mode_from_menu() {
    local choice
    while true; do
        show_main_menu
        read -r -p "输入数字选择模式: " choice
        case "$choice" in
            1)
                MODE="sim"
                EXTRA_ARGS=()
                break
                ;;
            2)
                MODE="real"
                EXTRA_ARGS=()
                select_real_backend_from_menu
                break
                ;;
            3)
                MODE="mc"
                EXTRA_ARGS=()
                break
                ;;
            4)
                MODE="usbfdcan-sim"
                EXTRA_ARGS=()
                break
                ;;
            q|Q)
                echo "已退出。"
                exit 0
                ;;
            *)
                PRINT_RED "无效选择: $choice"
                ;;
        esac
    done
}

resolve_numeric_mode() {
    case "$MODE" in
        1)
            MODE="sim"
            ;;
        2)
            MODE="real"
            ;;
        3)
            MODE="mc"
            ;;
        4)
            MODE="usbfdcan-sim"
            ;;
        usb2fdcan-sim)
            MODE="usbfdcan-sim"
            ;;
    esac
}

select_real_backend_from_menu() {
    local choice
    while true; do
        show_real_backend_menu
        read -r -p "输入数字选择 Real 后端: " choice
        case "$choice" in
            1)
                REAL_BACKEND="serial"
                break
                ;;
            2)
                REAL_BACKEND="usbfdcan"
                break
                ;;
            q|Q)
                echo "已退出。"
                exit 0
                ;;
            *)
                PRINT_RED "无效选择: $choice"
                ;;
        esac
    done
}

normalize_real_backend() {
    case "${REAL_BACKEND:-serial}" in
        ""|serial|uart)
            REAL_BACKEND="serial"
            ;;
        usbfdcan|can|socketcan)
            REAL_BACKEND="usbfdcan"
            ;;
        *)
            PRINT_RED "错误: 未知 real 后端 '$REAL_BACKEND'。请使用 'serial' 或 'usbfdcan'。"
            exit 1
            ;;
    esac
}

# 1. 参数解析与模式选择
MODE=${1:-}
EXTRA_ARGS=()
REAL_BACKEND=${AM_D02_REAL_BACKEND:-serial}
if [ -z "$MODE" ] || [[ "$MODE" == -* ]]; then
    select_mode_from_menu
else
    shift 2>/dev/null || true # 移出第一个参数，剩余参数传递给后端程序
    resolve_numeric_mode
    if [ "$MODE" == "real" ]; then
        case "${1:-}" in
            serial|uart|usbfdcan|can|socketcan)
                REAL_BACKEND="$1"
                shift 2>/dev/null || true
                ;;
        esac
    fi
fi
if [ "$MODE" == "usb2fdcan-sim" ]; then
    MODE="usbfdcan-sim"
fi
if [ "$MODE" == "real" ]; then
    normalize_real_backend
fi
APP_ARGS=("${EXTRA_ARGS[@]}" "$@")
select_python

echo "=========================================================="
if [ "$MODE" == "sim" ]; then
    echo "    AM-D02 软硬件联合仿真启动系统 (SITL 模式)       "
elif [ "$MODE" == "mc" ] || [ "$MODE" == "monte-carlo" ]; then
    echo "    AM-D02 蒙特卡罗末端位姿范围检查 (Monte Carlo)  "
elif [ "$MODE" == "usbfdcan-sim" ]; then
    echo "    AM-D02 USB2FDCAN 反馈驱动仿真 + 全零发送      "
elif [ "$MODE" == "real" ]; then
    if [ "$REAL_BACKEND" == "usbfdcan" ]; then
        echo "    AM-D02 机械臂 SocketCAN USB2FDCAN 控制系统 (Real) "
    else
        echo "    AM-D02 机械臂真实硬件串口控制系统 (Real 模式)     "
    fi
else
    PRINT_RED "错误: 未知模式 '$MODE'。请使用 'sim'、'mc'、'real' 或 'usbfdcan-sim'。"
    exit 1
fi
echo "=========================================================="
PRINT_BLUE "[System] Python 解释器: $PYTHON_BIN"

# 2. 编译必要的 C 程序
PRINT_BLUE "[1/3] 正在编译底层 C 语言组件..."
if [ "$MODE" == "sim" ]; then
    make -C c_interface clean && make -C c_interface c_main serial_gravity_comp
elif [ "$MODE" == "mc" ] || [ "$MODE" == "monte-carlo" ]; then
    make -C c_interface serial_gravity_comp
elif [ "$MODE" == "usbfdcan-sim" ]; then
    PRINT_BLUE "[1/3] USB2FDCAN 反馈镜像模式不需要编译 C 控制回路，跳过。"
else
    make -C c_interface serial_gravity_comp
fi

# 3. 根据模式启动不同的链路
if [ "$MODE" == "sim" ]; then
    # --- 模式 A: 纯仿真 (SITL) ---
    PRINT_BLUE "[2/3] 启动后台 Python MuJoCo 物理仿真服务器..."
    READY_FILE=$(mktemp /tmp/am_d02_server_ready.XXXXXX)
    "$PYTHON_BIN" python/sim/main_server.py --ready-file "$READY_FILE" "${APP_ARGS[@]}" &
    SERVER_PID=$!

    cleanup() {
        if [ ! -z "${SERVER_PID:-}" ] && kill -0 "$SERVER_PID" 2>/dev/null; then
            kill "$SERVER_PID" 2>/dev/null || true
            wait "$SERVER_PID" 2>/dev/null || true
        fi
        if [ ! -z "${READY_FILE:-}" ]; then
            rm -f "$READY_FILE"
        fi
    }

    on_signal() {
        echo -e "\n[Shutdown] 捕捉到退出信号，正在关闭后台仿真服务器..."
        cleanup
        echo "[Shutdown] 联合仿真会话已结束。"
        exit 0
    }
    trap on_signal SIGINT SIGTERM
    trap cleanup EXIT

    for _ in $(seq 1 200); do
        if [ -f "$READY_FILE" ]; then
            break
        fi
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

    PRINT_GREEN "[3/3] 正式启动前台 C 语言实时控制回路..."
    echo "----------------------------------------------------------"
    set +e
    ./c_interface/c_main
    APP_STATUS=$?
    set -e
    exit $APP_STATUS

elif [ "$MODE" == "mc" ] || [ "$MODE" == "monte-carlo" ]; then
    # --- 模式 C: 蒙特卡罗末端位姿范围检查 ---
    : "${AM_D02_ENABLE_VIEWER:=0}"
    : "${AM_D02_ENABLE_RERUN:=0}"
    export AM_D02_ENABLE_VIEWER
    export AM_D02_ENABLE_RERUN

    PRINT_BLUE "[2/3] 启动 MuJoCo 模型并随机采样关节空间..."
    PRINT_GREEN "[3/3] 刷新并输出末端位置和四元数数值范围..."
    echo "----------------------------------------------------------"
    "$PYTHON_BIN" python/mc/main.py "${APP_ARGS[@]}"

elif [ "$MODE" == "usbfdcan-sim" ]; then
    # --- 模式 D: USB2FDCAN 反馈驱动仿真 + 全零高速发送 ---
    : "${AM_D02_CAN_INTERFACE:=can0}"
    : "${AM_D02_RERUN_LOG_STRIDE:=1}"
    : "${AM_D02_REAL_VIEWER_FPS:=30}"
    : "${AM_D02_RERUN_QUEUE_SIZE:=512}"
    export AM_D02_CAN_INTERFACE
    export AM_D02_RERUN_LOG_STRIDE
    export AM_D02_REAL_VIEWER_FPS
    export AM_D02_RERUN_QUEUE_SIZE

    PRINT_BLUE "[2/3] 检查 SocketCAN 接口 ${AM_D02_CAN_INTERFACE}..."
    if [ -e "/sys/class/net/${AM_D02_CAN_INTERFACE}" ]; then
        CAN_STATE=$(cat "/sys/class/net/${AM_D02_CAN_INTERFACE}/operstate" 2>/dev/null || true)
        if [ "$CAN_STATE" != "up" ]; then
            PRINT_RED "警告: ${AM_D02_CAN_INTERFACE} 当前状态为 '${CAN_STATE:-unknown}'，请确认已配置并 up。"
        fi
    else
        PRINT_RED "警告: 未检测到 SocketCAN 接口 ${AM_D02_CAN_INTERFACE}，请确保 USB2FDCAN 已枚举。"
    fi

    PRINT_GREEN "[3/3] 启动 USB2FDCAN 反馈镜像仿真和全零 MIT 高速发送..."
    echo "----------------------------------------------------------"
    "$PYTHON_BIN" python/usbfdcan_sim/main.py "${APP_ARGS[@]}"

else
    # --- 模式 B: 真实硬件 (Real) ---
    if [ "$REAL_BACKEND" == "usbfdcan" ]; then
        : "${AM_D02_CAN_INTERFACE:=can0}"
        export AM_D02_CAN_INTERFACE
        PRINT_BLUE "[2/3] 检查 SocketCAN 接口 ${AM_D02_CAN_INTERFACE}..."
        if [ -e "/sys/class/net/${AM_D02_CAN_INTERFACE}" ]; then
            CAN_STATE=$(cat "/sys/class/net/${AM_D02_CAN_INTERFACE}/operstate" 2>/dev/null || true)
            if [ "$CAN_STATE" != "up" ]; then
                PRINT_RED "警告: ${AM_D02_CAN_INTERFACE} 当前状态为 '${CAN_STATE:-unknown}'，请确认已配置并 up。"
            fi
        else
            PRINT_RED "警告: 未检测到 SocketCAN 接口 ${AM_D02_CAN_INTERFACE}，请确保 USB2FDCAN 已枚举。"
        fi
    else
        PRINT_BLUE "[2/3] 检查串口权限..."
        SERIAL_PORT="/dev/ttyUSB0"
        if [ -e "$SERIAL_PORT" ]; then
            if [ ! -w "$SERIAL_PORT" ]; then
                PRINT_BLUE "[System] 正在尝试获取串口 $SERIAL_PORT 的访问权限..."
                sudo chmod o+rw "$SERIAL_PORT"
            fi
        else
            PRINT_RED "警告: 未检测到串口 $SERIAL_PORT，请确保硬件已连接。"
        fi
    fi

    : "${AM_D02_RERUN_LOG_STRIDE:=25}"
    : "${AM_D02_REAL_VIEWER_FPS:=30}"
    : "${AM_D02_RERUN_QUEUE_SIZE:=512}"
    export AM_D02_RERUN_LOG_STRIDE
    export AM_D02_REAL_VIEWER_FPS
    export AM_D02_RERUN_QUEUE_SIZE

    PRINT_BLUE "[System] Real 模式加速参数: Rerun 每 ${AM_D02_RERUN_LOG_STRIDE} 步记录一次, Viewer ${AM_D02_REAL_VIEWER_FPS} FPS"
    if [ "$REAL_BACKEND" == "usbfdcan" ]; then
        PRINT_GREEN "[3/3] 正式启动 Python SocketCAN USB2FDCAN 控制回路..."
    else
        PRINT_GREEN "[3/3] 正式启动 Python 串口控制回路..."
    fi
    echo "----------------------------------------------------------"
    if [ "$REAL_BACKEND" == "usbfdcan" ]; then
        "$PYTHON_BIN" python/real/usb2fdcan_control.py "${APP_ARGS[@]}"
    else
        "$PYTHON_BIN" python/real/serial_control.py "${APP_ARGS[@]}"
    fi
fi
