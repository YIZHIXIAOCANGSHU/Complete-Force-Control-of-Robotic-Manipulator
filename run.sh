#!/bin/bash
# AM-DPBSURDF0422 sim/mc unified launcher.
set -e
cd "$(dirname "${BASH_SOURCE[0]}")"

PRINT_BLUE() { echo -e "\033[34m$1\033[0m"; }
PRINT_GREEN() { echo -e "\033[32m$1\033[0m"; }
PRINT_RED() { echo -e "\033[31m$1\033[0m"; }

PYTHON_BIN=${AM_D02_PYTHON:-}
PROJECT_VENV=${AM_D02_VENV_DIR:-.venv}

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

    if [ -x "$PROJECT_VENV/bin/python" ]; then
        PYTHON_BIN="$PROJECT_VENV/bin/python"
        return
    fi

    if [ -d "$PROJECT_VENV" ] && [ ! -x "$PROJECT_VENV/bin/python" ]; then
        PRINT_RED "错误: 检测到 $PROJECT_VENV，但其中没有可执行的 bin/python。"
        PRINT_RED "请删除后重建: rm -rf $PROJECT_VENV && ./scripts/setup_venv.sh"
        exit 1
    fi

    PRINT_RED "错误: 未找到当前项目虚拟环境: $PROJECT_VENV"
    PRINT_RED "请先运行: ./scripts/setup_venv.sh"
    PRINT_RED "或者显式指定: AM_D02_PYTHON=/path/to/python ./run.sh ..."
    exit 1
}

show_main_menu() {
    echo "=========================================================="
        echo "        AM-DPBSURDF0422 Sim 启动状态机"
    echo "=========================================================="
    echo "请选择启动模式："
    echo "  1) sim  - 软硬件联合仿真"
    echo "  2) real - 真实硬件控制"
    echo "  3) mc   - 蒙特卡罗范围检查 + MuJoCo 窗口"
    echo "  q) 退出"
    echo "----------------------------------------------------------"
}

show_real_menu() {
    echo "=========================================================="
    echo "        AM-DPBSURDF0422 Real 真机模式"
    echo "=========================================================="
    echo "请选择真机控制对象："
    echo "  1) left  - 左臂真机 can0"
    echo "  2) right - 右臂真机 can1"
    echo "  3) both  - 双臂真机 can0 + can1"
    echo "  q) 退出"
    echo "----------------------------------------------------------"
}

show_real_backend_menu() {
    echo "=========================================================="
    echo "        AM-DPBSURDF0422 Real 控制方式"
    echo "=========================================================="
    echo "请选择 Real 控制后端："
    echo "  1) c         - 当前 C/STM32 控制核心"
    echo "  2) pinocchio - Python Pinocchio 纯 tau 控制"
    echo "  q) 退出"
    echo "----------------------------------------------------------"
}

select_real_arm_from_menu() {
    local choice
    while true; do
        show_real_menu
        read -r -p "输入数字选择 Real 模式: " choice
        case "$choice" in
            1)
                REAL_ARM="left"
                break
                ;;
            2)
                REAL_ARM="right"
                break
                ;;
            3)
                REAL_ARM="both"
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

select_real_backend_from_menu() {
    local choice
    while true; do
        show_real_backend_menu
        read -r -p "输入数字选择 Real 控制方式: " choice
        case "$choice" in
            1)
                REAL_BACKEND="c"
                break
                ;;
            2)
                REAL_BACKEND="pinocchio"
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

select_mode_from_menu() {
    local choice
    while true; do
        show_main_menu
        read -r -p "输入数字选择模式: " choice
        case "$choice" in
            1)
                MODE="sim"
                break
                ;;
            2)
                MODE="real"
                select_real_arm_from_menu
                select_real_backend_from_menu
                break
                ;;
            3)
                MODE="mc"
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
        real-left)
            MODE="real"
            REAL_ARM="left"
            ;;
        real-right)
            MODE="real"
            REAL_ARM="right"
            ;;
        real-both)
            MODE="real"
            REAL_ARM="both"
            ;;
    esac
}

MODE=${1:-}
REAL_ARM=${AM_D02_REAL_ARM:-}
REAL_BACKEND=${AM_D02_REAL_CONTROL_BACKEND:-c}
if [ -z "$MODE" ] || [[ "$MODE" == -* ]]; then
    select_mode_from_menu
else
    shift 2>/dev/null || true
    resolve_numeric_mode
    if [ "$MODE" == "real" ]; then
        case "${1:-}" in
            left|right|both)
                REAL_ARM="$1"
                shift 2>/dev/null || true
                ;;
        esac
        case "${1:-}" in
            c|pinocchio)
                REAL_BACKEND="$1"
                shift 2>/dev/null || true
                ;;
        esac
        if [ -z "$REAL_ARM" ]; then
            select_real_arm_from_menu
        fi
    fi
fi
APP_ARGS=("$@")
select_python

echo "=========================================================="
if [ "$MODE" == "sim" ]; then
    echo "    AM-DPBSURDF0422 双臂联合仿真启动系统 (SITL)       "
elif [ "$MODE" == "real" ]; then
    echo "    AM-DPBSURDF0422 双臂真机 SocketCAN 控制系统 (Real) "
elif [ "$MODE" == "mc" ] || [ "$MODE" == "monte-carlo" ]; then
    echo "    AM-DPBSURDF0422 双臂蒙特卡罗末端位姿范围检查      "
else
    PRINT_RED "错误: 未知模式 '$MODE'。请使用 'sim'、'real'、'mc' 或 'monte-carlo'。"
    exit 1
fi
echo "=========================================================="
PRINT_BLUE "[System] Python 解释器: $PYTHON_BIN"

if [ "$MODE" == "sim" ]; then
    PRINT_BLUE "[1/3] 正在编译底层 C 语言仿真控制器..."
    make -C c_interface clean
    make -C c_interface c_main

    PRINT_BLUE "[2/3] 启动后台 Python MuJoCo 物理仿真服务器..."
    READY_FILE=$(mktemp /tmp/am_dpbs_server_ready.XXXXXX)
    "$PYTHON_BIN" python/sim/main_server.py --ready-file "$READY_FILE" "${APP_ARGS[@]}" &
    SERVER_PID=$!

    cleanup() {
        if [ -n "${SERVER_PID:-}" ] && kill -0 "$SERVER_PID" 2>/dev/null; then
            kill "$SERVER_PID" 2>/dev/null || true
            wait "$SERVER_PID" 2>/dev/null || true
        fi
        if [ -n "${READY_FILE:-}" ]; then
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
fi

if [ "$MODE" == "real" ]; then
    case "$REAL_ARM" in
        left|right|both)
            ;;
        *)
            PRINT_RED "错误: 未知 Real 控制对象 '$REAL_ARM'。请使用 left、right 或 both。"
            exit 1
            ;;
    esac
    case "$REAL_BACKEND" in
        c|pinocchio)
            ;;
        *)
            PRINT_RED "错误: 未知 Real 控制方式 '$REAL_BACKEND'。请使用 c 或 pinocchio。"
            exit 1
            ;;
    esac

    : "${AM_D02_LEFT_CAN_INTERFACE:=can0}"
    : "${AM_D02_RIGHT_CAN_INTERFACE:=can1}"
    export AM_D02_LEFT_CAN_INTERFACE
    export AM_D02_RIGHT_CAN_INTERFACE

    if [ "$REAL_BACKEND" == "c" ]; then
        PRINT_BLUE "[1/3] 正在编译真机 C 控制桥..."
        make -C c_interface real_controller
    else
        PRINT_BLUE "[1/3] Pinocchio real 后端不需要编译 C 控制桥，跳过。"
    fi

    if [ "$REAL_ARM" == "left" ] || [ "$REAL_ARM" == "both" ]; then
        PRINT_BLUE "[2/3] 检查左臂 SocketCAN 接口 ${AM_D02_LEFT_CAN_INTERFACE}..."
        if [ ! -e "/sys/class/net/${AM_D02_LEFT_CAN_INTERFACE}" ]; then
            PRINT_RED "警告: 未检测到左臂 CAN 接口 ${AM_D02_LEFT_CAN_INTERFACE}。"
        fi
    fi
    if [ "$REAL_ARM" == "right" ] || [ "$REAL_ARM" == "both" ]; then
        PRINT_BLUE "[2/3] 检查右臂 SocketCAN 接口 ${AM_D02_RIGHT_CAN_INTERFACE}..."
        if [ ! -e "/sys/class/net/${AM_D02_RIGHT_CAN_INTERFACE}" ]; then
            PRINT_RED "警告: 未检测到右臂 CAN 接口 ${AM_D02_RIGHT_CAN_INTERFACE}。"
        fi
    fi

    PRINT_GREEN "[3/3] 启动 Real ${REAL_ARM} ${REAL_BACKEND} 控制回路..."
    echo "----------------------------------------------------------"
    if [ "$REAL_BACKEND" == "pinocchio" ]; then
        "$PYTHON_BIN" python/real_pinocchio/main.py --arm "$REAL_ARM" "${APP_ARGS[@]}"
    else
        "$PYTHON_BIN" python/real/main.py --arm "$REAL_ARM" "${APP_ARGS[@]}"
    fi
    exit $?
fi

: "${AM_D02_ENABLE_VIEWER:=0}"
: "${AM_D02_ENABLE_RERUN:=0}"
export AM_D02_ENABLE_VIEWER
export AM_D02_ENABLE_RERUN

PRINT_BLUE "[1/3] Monte Carlo 模式不需要编译 C 控制回路，跳过。"
PRINT_BLUE "[2/3] 启动 MuJoCo 模型并随机采样关节空间..."
PRINT_GREEN "[3/3] 刷新并输出末端位置和四元数数值范围..."
echo "----------------------------------------------------------"
"$PYTHON_BIN" python/sim/main_server.py --monte-carlo "${APP_ARGS[@]}"
