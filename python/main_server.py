#!/usr/bin/env python3

from __future__ import annotations

import argparse
import os
import sys


sys.path.append(os.path.dirname(os.path.abspath(__file__)))

from udp_server import run_udp_server


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="AM-D02 MuJoCo UDP simulation server")
    parser.add_argument(
        "--ready-file",
        default=None,
        help="Optional file path written once the UDP server is ready for clients",
    )
    args = parser.parse_args()
    run_udp_server(ready_file=args.ready_file)
