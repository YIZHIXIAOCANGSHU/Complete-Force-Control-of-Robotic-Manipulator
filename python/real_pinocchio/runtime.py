"""Runtime entry for the Pinocchio pure-tau real backend."""

from __future__ import annotations

import subprocess
import sys

from real.runtime import run_real_control_with_bridge
from real_pinocchio.controller_bridge import PinocchioRealControllerBridge


ACTIVE_ARM_MASK = {
    "left": 1 << 0,
    "right": 1 << 1,
    "both": (1 << 0) | (1 << 1),
}


class PinocchioUnavailableError(RuntimeError):
    pass


def verify_pinocchio_runtime_available() -> None:
    probe = (
        "import pinocchio as pin; "
        "print(getattr(pin, '__version__', 'unknown'))"
    )
    try:
        completed = subprocess.run(
            [sys.executable, "-c", probe],
            text=True,
            capture_output=True,
            timeout=10.0,
            check=False,
        )
    except subprocess.TimeoutExpired as exc:
        raise PinocchioUnavailableError(
            "[Pinocchio Error] importing pinocchio timed out before CAN startup."
        ) from exc

    if completed.returncode == 0:
        version = completed.stdout.strip() or "unknown"
        print(f"[Pinocchio Config] import check passed, version: {version}")
        return

    detail = (completed.stderr or completed.stdout or "").strip()
    if len(detail) > 1200:
        detail = detail[-1200:]
    raise PinocchioUnavailableError(
        "[Pinocchio Error] selected backend cannot import pinocchio safely.\n"
        "This is an environment issue, not a CAN/controller failure. The current "
        "Pinocchio module appears incompatible with the active NumPy/Python ABI.\n"
        "Fix by running this backend in a Python environment where Pinocchio and "
        "NumPy are built for the same ABI, for example NumPy < 2 with the ROS "
        "Humble Pinocchio package, or a Pinocchio build compiled for NumPy 2.x.\n"
        f"Probe exit code: {completed.returncode}\n"
        f"{detail}"
    )


def run_real_pinocchio_control(arm: str) -> None:
    try:
        verify_pinocchio_runtime_available()
    except PinocchioUnavailableError as exc:
        print(str(exc), file=sys.stderr)
        raise SystemExit(2) from None
    run_real_control_with_bridge(
        arm,
        bridge_factory=lambda: PinocchioRealControllerBridge(active_arm_mask=ACTIVE_ARM_MASK[arm]),
        control_title="AM-DPBSURDF0422 Real SocketCAN Control [Pinocchio tau]",
        rerun_app_name="AM-DPBSURDF0422 Real CAN Pinocchio",
    )
