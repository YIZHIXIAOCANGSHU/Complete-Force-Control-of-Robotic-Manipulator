#!/usr/bin/env python3
"""SITL simulation server entrypoint."""

from __future__ import annotations

import argparse
import os
import sys


PYTHON_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
PROJECT_ROOT = os.path.dirname(PYTHON_ROOT)
for path in (PYTHON_ROOT, PROJECT_ROOT):
    if path not in sys.path:
        sys.path.insert(0, path)

from sim.udp_server import run_udp_server


def _translate_argparse_text(text: str) -> str:
    replacements = {
        "usage:": "用法:",
        "options:": "选项:",
        "optional arguments:": "可选参数:",
        "show this help message and exit": "显示帮助信息并退出",
        "error:": "错误:",
        "unrecognized arguments:": "无法识别的参数:",
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


def main() -> None:
    parser = ChineseArgumentParser(description="AM-D02 MuJoCo UDP 仿真服务")
    parser.add_argument(
        "--ready-file",
        default=None,
        help="UDP 服务就绪后写入的可选标记文件路径",
    )
    args = parser.parse_args()
    try:
        run_udp_server(ready_file=args.ready_file)
    except (RuntimeError, ValueError) as exc:
        print(f"[Server] 错误: {exc}", file=sys.stderr)
        raise SystemExit(1)


if __name__ == "__main__":
    main()
