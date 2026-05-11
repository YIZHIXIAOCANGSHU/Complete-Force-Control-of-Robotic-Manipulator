#!/usr/bin/env python3
"""Monte Carlo workspace range-check entrypoint."""

from __future__ import annotations

import argparse
import os
import sys


PYTHON_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
PROJECT_ROOT = os.path.dirname(PYTHON_ROOT)
for path in (PYTHON_ROOT, PROJECT_ROOT):
    if path not in sys.path:
        sys.path.insert(0, path)

from mc.workspace import (
    DEFAULT_MC_MAX_HULL_POINTS,
    DEFAULT_MC_MAX_VIS_POINTS,
    DEFAULT_MC_PROGRESS_INTERVAL,
    DEFAULT_MC_SAMPLES,
    run_monte_carlo_range_check,
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
    parser = ChineseArgumentParser(description="AM-D02 蒙特卡洛末端位姿范围检查")
    parser.add_argument(
        "-n",
        "--samples",
        type=int,
        default=max(1, _env_int("AM_D02_MC_SAMPLES", DEFAULT_MC_SAMPLES)),
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
        default=max(0, _env_int("AM_D02_MC_PROGRESS_INTERVAL", DEFAULT_MC_PROGRESS_INTERVAL)),
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
    args = parser.parse_args()
    try:
        run_monte_carlo_range_check(
            samples=args.samples,
            seed=args.seed,
            progress_interval=args.progress_interval,
            show_viewer=not args.no_viewer,
            max_visual_points=args.max_visual_points,
            max_hull_points=args.max_hull_points,
        )
    except (RuntimeError, ValueError) as exc:
        print(f"[MC] 错误: {exc}", file=sys.stderr)
        raise SystemExit(1)


if __name__ == "__main__":
    main()
