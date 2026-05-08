#!/bin/bash
# AM-DPBSURDF0422 sim/mc unified launcher.
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

    local candidates=("python3" "python" "$HOME/miniconda3/envs/dial-mpc-py310/bin/python" "$HOME/miniconda3/bin/python")
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
    echo "        AM-DPBSURDF0422 Sim 启动状态机"
    echo "=========================================================="
    echo "请选择启动模式："
    echo "  1) sim  - 软硬件联合仿真"
    echo "  2) mc   - 蒙特卡罗范围检查 + MuJoCo 窗口"
    echo "  q) 退出"
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
                break
                ;;
            2)
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
            MODE="mc"
            ;;
    esac
}

MODE=${1:-}
if [ -z "$MODE" ] || [[ "$MODE" == -* ]]; then
    select_mode_from_menu
else
    shift 2>/dev/null || true
    resolve_numeric_mode
fi
APP_ARGS=("$@")
select_python

echo "=========================================================="
if [ "$MODE" == "sim" ]; then
    echo "    AM-DPBSURDF0422 左臂联合仿真启动系统 (SITL)       "
elif [ "$MODE" == "mc" ] || [ "$MODE" == "monte-carlo" ]; then
    echo "    AM-DPBSURDF0422 左臂蒙特卡罗末端位姿范围检查      "
else
    PRINT_RED "错误: 未知模式 '$MODE'。请使用 'sim'、'mc' 或 'monte-carlo'。"
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
    "$PYTHON_BIN" python/main_server.py --ready-file "$READY_FILE" "${APP_ARGS[@]}" &
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

: "${AM_D02_ENABLE_VIEWER:=0}"
: "${AM_D02_ENABLE_RERUN:=0}"
export AM_D02_ENABLE_VIEWER
export AM_D02_ENABLE_RERUN

PRINT_BLUE "[1/3] Monte Carlo 模式不需要编译 C 控制回路，跳过。"
PRINT_BLUE "[2/3] 启动 MuJoCo 模型并随机采样关节空间..."
PRINT_GREEN "[3/3] 刷新并输出末端位置和四元数数值范围..."
echo "----------------------------------------------------------"
"$PYTHON_BIN" python/main_server.py --monte-carlo "${APP_ARGS[@]}"
