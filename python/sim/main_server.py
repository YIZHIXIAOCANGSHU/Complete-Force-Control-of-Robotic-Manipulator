#!/usr/bin/env python3

from __future__ import annotations

import argparse
import os
import sys


PYTHON_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if PYTHON_ROOT not in sys.path:
    sys.path.insert(0, PYTHON_ROOT)

from sim.udp_server import (
    DEFAULT_MC_MAX_VIS_POINTS,
    DEFAULT_MC_MAX_HULL_POINTS,
    DEFAULT_MC_PROGRESS_INTERVAL,
    DEFAULT_MC_SAMPLES,
    DEFAULT_REPORT_PROGRESS_INTERVAL,
    DEFAULT_REPORT_SAMPLES,
    run_monte_carlo_range_check,
    run_sim_report,
    run_udp_server,
)


def _translate_argparse_text(text: str) -> str:
    replacements = {
        "usage:": "用法:",
        "options:": "选项:",
        "optional arguments:": "可选参数:",
        "show this help message and exit": "显示帮助信息并退出",
        "error:": "错误:",
        "unrecognized arguments:": "无法识别的参数:",
        "argument --seed: invalid _arg_seed value:": "参数 --seed 的取值无效:",
        "argument -n/--samples: invalid int value:": "参数 -n/--samples 必须是整数:",
        "argument --progress-interval: invalid int value:": "参数 --progress-interval 必须是整数:",
        "argument --max-visual-points: invalid int value:": "参数 --max-visual-points 必须是整数:",
        "argument --max-hull-points: invalid int value:": "参数 --max-hull-points 必须是整数:",
        "argument --output-dir: expected one argument": "参数 --output-dir 需要一个路径:",
        "argument --control-duration-s: invalid float value:": "参数 --control-duration-s 必须是数字:",
        "argument --control-log-stride: invalid int value:": "参数 --control-log-stride 必须是整数:",
    }
    for source, target in replacements.items():
        text = text.replace(source, target)
    return text


class ChineseArgumentParser(argparse.ArgumentParser):
    def format_usage(self) -> str:
        return _translate_argparse_text(super().format_usage())

    def format_help(self) -> str:
        return _translate_argparse_text(super().format_help())

    def exit(self, status: int = 0, message: str | None = None) -> None:
        if message:
            message = _translate_argparse_text(message)
        super().exit(status, message)


def _arg_seed(value: str) -> int | None:
    if value.strip().lower() in ("none", "random", "-"):
        return None
    return int(value)


def _env_int(name: str, default: int) -> int:
    value = os.getenv(name)
    if value is None:
        return default
    try:
        return int(value)
    except ValueError:
        return default


def _env_seed(name: str) -> int | None:
    value = os.getenv(name)
    if value is None:
        return None
    return _arg_seed(value)


def main() -> None:
    raw_argv = sys.argv[1:]
    seed_arg_present = any(arg == "--seed" or arg.startswith("--seed=") for arg in raw_argv)
    parser = ChineseArgumentParser(description="AM-DPBSURDF0422 MuJoCo 仿真/蒙特卡洛服务")
    parser.add_argument(
        "--ready-file",
        default=None,
        help="UDP 服务就绪后写入的可选标记文件路径",
    )
    parser.add_argument(
        "--monte-carlo",
        action="store_true",
        help="启用蒙特卡洛正向运动学范围检查模式，而不是 UDP 仿真服务模式",
    )
    parser.add_argument(
        "--report",
        action="store_true",
        help="生成 sim-only 报告数据包，包含蒙特卡洛工作空间和闭环控制实验",
    )
    parser.add_argument(
        "-n",
        "--samples",
        type=int,
        default=None,
        help="蒙特卡洛模式的采样数量",
    )
    parser.add_argument(
        "--seed",
        type=_arg_seed,
        default=_env_seed("AM_D02_MC_SEED"),
        help="蒙特卡洛随机种子；使用 'random' 表示非确定性采样",
    )
    parser.add_argument(
        "--progress-interval",
        type=int,
        default=None,
        help="每采样 N 次刷新一次终端进度；使用 0 关闭进度刷新",
    )
    parser.add_argument(
        "--no-viewer",
        action="store_true",
        help="蒙特卡洛采样结束后不打开 MuJoCo 工作空间范围窗口",
    )
    parser.add_argument(
        "--max-visual-points",
        type=int,
        default=max(1, _env_int("AM_D02_MC_MAX_VIS_POINTS", DEFAULT_MC_MAX_VIS_POINTS)),
        help="MuJoCo 窗口中最多显示的末端采样点数量",
    )
    parser.add_argument(
        "--max-hull-points",
        type=int,
        default=max(1, _env_int("AM_D02_MC_MAX_HULL_POINTS", DEFAULT_MC_MAX_HULL_POINTS)),
        help="用于构建工作空间凸包的最大末端采样点数量",
    )
    parser.add_argument(
        "--output-dir",
        default=None,
        help="蒙特卡洛模式的报告数据输出目录；为空时只输出终端报告",
    )
    parser.add_argument(
        "--control-duration-s",
        type=float,
        default=10.0,
        help="报告模式闭环控制实验时长，单位秒",
    )
    parser.add_argument(
        "--control-log-stride",
        type=int,
        default=10,
        help="报告模式闭环控制 CSV 每 N 步记录一次",
    )
    args = parser.parse_args()
    try:
        if args.report:
            report_seed_env = _env_seed("AM_D02_REPORT_SEED")
            report_seed = args.seed if seed_arg_present else (
                report_seed_env if os.getenv("AM_D02_REPORT_SEED") is not None else 42
            )
            run_sim_report(
                samples=max(
                    1,
                    args.samples
                    if args.samples is not None
                    else _env_int("AM_D02_REPORT_SAMPLES", DEFAULT_REPORT_SAMPLES),
                ),
                seed=report_seed,
                progress_interval=max(
                    0,
                    args.progress_interval
                    if args.progress_interval is not None
                    else _env_int("AM_D02_REPORT_PROGRESS_INTERVAL", DEFAULT_REPORT_PROGRESS_INTERVAL),
                ),
                max_hull_points=args.max_hull_points,
                output_dir=args.output_dir,
                control_duration_s=args.control_duration_s,
                control_log_stride=args.control_log_stride,
            )
        elif args.monte_carlo:
            run_monte_carlo_range_check(
                samples=max(
                    1,
                    args.samples
                    if args.samples is not None
                    else _env_int("AM_D02_MC_SAMPLES", DEFAULT_MC_SAMPLES),
                ),
                seed=args.seed,
                progress_interval=max(
                    0,
                    args.progress_interval
                    if args.progress_interval is not None
                    else _env_int("AM_D02_MC_PROGRESS_INTERVAL", DEFAULT_MC_PROGRESS_INTERVAL),
                ),
                show_viewer=not args.no_viewer,
                max_visual_points=args.max_visual_points,
                max_hull_points=args.max_hull_points,
                output_dir=args.output_dir,
            )
        else:
            run_udp_server(ready_file=args.ready_file)
    except (RuntimeError, ValueError) as exc:
        print(f"[Server] 错误: {exc}", file=sys.stderr)
        raise SystemExit(1)


if __name__ == "__main__":
    main()
