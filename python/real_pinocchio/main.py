#!/usr/bin/env python3
"""Real hardware SocketCAN entrypoint for the Pinocchio tau backend."""

from __future__ import annotations

import argparse
import os
import sys


PYTHON_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if PYTHON_ROOT not in sys.path:
    sys.path.insert(0, PYTHON_ROOT)


def main() -> None:
    parser = argparse.ArgumentParser(description="AM-DPBSURDF0422 Pinocchio real CAN control")
    parser.add_argument("--arm", choices=("left", "right", "both"), required=True)
    args = parser.parse_args()

    from real_pinocchio.runtime import run_real_pinocchio_control

    run_real_pinocchio_control(args.arm)


if __name__ == "__main__":
    main()
