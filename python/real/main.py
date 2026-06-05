#!/usr/bin/env python3
"""Real hardware SocketCAN entrypoint."""

from __future__ import annotations

import argparse
import os
import sys


PYTHON_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if PYTHON_ROOT not in sys.path:
    sys.path.insert(0, PYTHON_ROOT)


def main() -> None:
    parser = argparse.ArgumentParser(description="AM-DPBSURDF0422 real CAN control")
    parser.add_argument("--arm", choices=("left", "right", "both"), required=True)
    send_group = parser.add_mutually_exclusive_group()
    send_group.add_argument(
        "--send",
        dest="send_mode",
        action="store_const",
        const="control",
        default="control",
        help="send computed C/STM32 torque commands to CAN (default)",
    )
    send_group.add_argument(
        "--no-send",
        dest="send_mode",
        action="store_const",
        const="zero",
        help="compute and visualize torque commands, but send zero MIT torque on CAN",
    )
    send_group.add_argument(
        "--gravity-only",
        dest="send_mode",
        action="store_const",
        const="gravity",
        help="compute full control, but send pure gravity compensation torque G(q) on CAN",
    )
    args = parser.parse_args()

    from real.runtime import run_real_control

    run_real_control(args.arm, send_mode=args.send_mode)


if __name__ == "__main__":
    main()
