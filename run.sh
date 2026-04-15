#!/bin/bash
# AM-D02 SITL 软硬件联合仿真与实机控制统一启动脚本
set -e
cd "$(dirname "${BASH_SOURCE[0]}")"

# 颜色定义
PRINT_BLUE() { echo -e "\033[34m$1\033[0m"; }
PRINT_GREEN() { echo -e "\033[32m$1\033[0m"; }
PRINT_RED() { echo -e "\033[31m$1\033[0m"; }

# 1. 参数解析与模式选择
MODE=${1:-"sim"} # 默认为仿真模式
shift 2>/dev/null || true # 移出第一个参数，剩余参数传递给后端程序

echo "=========================================================="
if [ "$MODE" == "sim" ]; then
    echo "    AM-D02 软硬件联合仿真启动系统 (SITL 模式)       "
elif [ "$MODE" == "real" ]; then
    echo "    AM-D02 机械臂真实硬件串口控制系统 (Real 模式)     "
else
    PRINT_RED "错误: 未知模式 '$MODE'。请使用 'sim' 或 'real'。"
    exit 1
fi
echo "=========================================================="

# 2. 编译必要的 C 程序
PRINT_BLUE "[1/3] 正在编译底层 C 语言组件..."
if [ "$MODE" == "sim" ]; then
    make -C c_interface clean && make -C c_interface c_main serial_gravity_comp
else
    make -C c_interface serial_gravity_comp
fi

# 3. 根据模式启动不同的链路
if [ "$MODE" == "sim" ]; then
    # --- 模式 A: 纯仿真 (SITL) ---
    PRINT_BLUE "[2/3] 启动后台 Python MuJoCo 物理仿真服务器..."
    READY_FILE=$(mktemp /tmp/am_d02_server_ready.XXXXXX)
    python3 python/main_server.py --ready-file "$READY_FILE" "$@" &
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

else
    # --- 模式 B: 真实硬件 (Real) ---
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

    : "${AM_D02_RERUN_LOG_STRIDE:=25}"
    : "${AM_D02_REAL_VIEWER_FPS:=30}"
    : "${AM_D02_RERUN_QUEUE_SIZE:=512}"
    export AM_D02_RERUN_LOG_STRIDE
    export AM_D02_REAL_VIEWER_FPS
    export AM_D02_RERUN_QUEUE_SIZE

    PRINT_BLUE "[System] Real 模式加速参数: Rerun 每 ${AM_D02_RERUN_LOG_STRIDE} 步记录一次, Viewer ${AM_D02_REAL_VIEWER_FPS} FPS"
    PRINT_GREEN "[3/3] 正式启动 Python 串口控制回路..."
    echo "----------------------------------------------------------"
    python3 python/serial_control.py "$@"
fi
