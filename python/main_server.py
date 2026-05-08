#!/usr/bin/env python3

from __future__ import annotations

import argparse
import os
import sys


sys.path.append(os.path.dirname(os.path.abspath(__file__)))

from udp_server import (
    DEFAULT_MC_MAX_VIS_POINTS,
    DEFAULT_MC_MAX_HULL_POINTS,
    DEFAULT_MC_PROGRESS_INTERVAL,
    DEFAULT_MC_SAMPLES,
    run_monte_carlo_range_check,
    run_udp_server,
)


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


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="AM-D02 MuJoCo UDP simulation server")
    parser.add_argument(
        "--ready-file",
        default=None,
        help="Optional file path written once the UDP server is ready for clients",
    )
    parser.add_argument(
        "--monte-carlo",
        action="store_true",
        help="Run the sim environment in Monte Carlo FK range-check mode instead of UDP server mode",
    )
    parser.add_argument(
        "-n",
        "--samples",
        type=int,
        default=max(1, _env_int("AM_D02_MC_SAMPLES", DEFAULT_MC_SAMPLES)),
        help="Monte Carlo sample count when --monte-carlo is enabled",
    )
    parser.add_argument(
        "--seed",
        type=_arg_seed,
        default=_env_seed("AM_D02_MC_SEED"),
        help="Monte Carlo random seed. Use 'random' for non-deterministic sampling",
    )
    parser.add_argument(
        "--progress-interval",
        type=int,
        default=max(0, _env_int("AM_D02_MC_PROGRESS_INTERVAL", DEFAULT_MC_PROGRESS_INTERVAL)),
        help="Refresh terminal every N Monte Carlo samples. Use 0 to disable progress refresh",
    )
    parser.add_argument(
        "--no-viewer",
        action="store_true",
        help="Do not open the MuJoCo workspace range viewer after Monte Carlo sampling",
    )
    parser.add_argument(
        "--max-visual-points",
        type=int,
        default=max(1, _env_int("AM_D02_MC_MAX_VIS_POINTS", DEFAULT_MC_MAX_VIS_POINTS)),
        help="Maximum end-effector sample points shown in the MuJoCo viewer",
    )
    parser.add_argument(
        "--max-hull-points",
        type=int,
        default=max(1, _env_int("AM_D02_MC_MAX_HULL_POINTS", DEFAULT_MC_MAX_HULL_POINTS)),
        help="Maximum end-effector sample points used to build the MuJoCo workspace hull",
    )
    args = parser.parse_args()
    try:
        if args.monte_carlo:
            run_monte_carlo_range_check(
                samples=args.samples,
                seed=args.seed,
                progress_interval=args.progress_interval,
                show_viewer=not args.no_viewer,
                max_visual_points=args.max_visual_points,
                max_hull_points=args.max_hull_points,
            )
        else:
            run_udp_server(ready_file=args.ready_file)
    except (RuntimeError, ValueError) as exc:
        print(f"[Server] 错误: {exc}", file=sys.stderr)
        raise SystemExit(1)
