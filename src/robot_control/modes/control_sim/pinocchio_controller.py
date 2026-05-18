#!/usr/bin/env python3
"""Python simulation controller — replaces c_interface/c_main.

Connects to the MuJoCo UDP server (port 9876), receives state packets,
runs Pinocchio-based control, and sends MIT commands back.
"""

from __future__ import annotations

import socket
import sys
import time

import numpy as np

from robot_control.config import Config
from robot_control.shared.transforms import RobotMujocoTransformer
from robot_control.dynamics.gravity import GravityCompTool


_UDP_IP = "127.0.0.1"
_UDP_PORT = 9876
_STATE_DOUBLES = 28
_MIT_DOUBLES = 35


class SimController:
    def __init__(self) -> None:
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._sock.settimeout(2.0)

    def connect(self, max_retries: int = 10) -> np.ndarray:
        print(f"[SimCtrl] Connecting to MuJoCo server at {_UDP_IP}:{_UDP_PORT}...")
        for attempt in range(max_retries):
            try:
                self._sock.sendto(b"INIT", (_UDP_IP, _UDP_PORT))
                data, _addr = self._sock.recvfrom(4096)
                if len(data) != _STATE_DOUBLES * 8:
                    raise RuntimeError(f"Expected {_STATE_DOUBLES * 8} bytes, got {len(data)}")
                print("[SimCtrl] Connected, received initial state.")
                return np.frombuffer(data, dtype="<f8", count=_STATE_DOUBLES)
            except socket.timeout:
                if attempt < max_retries - 1:
                    print(f"[SimCtrl] Retry {attempt + 1}/{max_retries}...")
                else:
                    raise RuntimeError(
                        f"Connection failed after {max_retries} attempts"
                    )

    def recv_state(self) -> np.ndarray:
        data, _addr = self._sock.recvfrom(4096)
        return np.frombuffer(data, dtype="<f8", count=_STATE_DOUBLES)

    def send_mit_command(
        self, q_ref: np.ndarray, qd_ref: np.ndarray,
        kp: np.ndarray, kd: np.ndarray, tau_ff: np.ndarray,
    ) -> None:
        buf = np.empty(_MIT_DOUBLES, dtype="<f8")
        buf[0:7] = q_ref
        buf[7:14] = qd_ref
        buf[14:21] = kp
        buf[21:28] = kd
        buf[28:35] = tau_ff
        self._sock.sendto(buf.tobytes(), (_UDP_IP, _UDP_PORT))

    def close(self) -> None:
        self._sock.close()


def main() -> None:
    print("=" * 60)
    print("   AM-D02 Pinocchio Simulation Controller (Python)  ")
    print("=" * 60)

    ctrl = SimController()
    try:
        initial_state = ctrl.connect()
    except Exception as exc:
        print(f"[SimCtrl] Connection failed: {exc}")
        sys.exit(1)

    transformer = RobotMujocoTransformer()
    comp_tool = GravityCompTool()
    print("[SimCtrl] Pinocchio backend ready.")

    step_count = 0
    print("[SimCtrl] Starting control loop...\n")

    state = initial_state

    try:
        while True:
            q = state[0:7].copy()
            qd = state[7:14].copy()
            mj_target_pos = state[21:24].copy()
            mj_target_quat = state[24:28].copy()

            robot_target_pos, robot_target_quat = transformer.mujoco_to_robot(
                mj_target_pos, mj_target_quat,
            )

            output = comp_tool.compute(q, qd, robot_target_pos, robot_target_quat)

            if output.status < 0:
                print(f"[SimCtrl] Safety violation (status={output.status}), exiting.")
                break

            ctrl.send_mit_command(
                np.array(output.q_ref),
                np.array(output.qd_ref),
                np.array(output.kp),
                np.array(output.kd),
                np.array(output.tau_ff),
            )

            step_count += 1
            if step_count % 500 == 0:
                print(
                    f"[Step {step_count:6d}] path={output.path_progress:.4f}m | "
                    f"ee=[{output.ee_pos[0]:.3f} {output.ee_pos[1]:.3f} {output.ee_pos[2]:.3f}] | "
                    f"tau[0]={output.tau_total[0]:.3f}"
                )

            state = ctrl.recv_state()
    except KeyboardInterrupt:
        pass
    except Exception as exc:
        print(f"[SimCtrl] Error: {exc}")
    finally:
        comp_tool.close()
        ctrl.close()
        print("[SimCtrl] Exited.")


if __name__ == "__main__":
    main()
