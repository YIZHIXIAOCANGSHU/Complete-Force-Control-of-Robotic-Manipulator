from __future__ import annotations

import struct
import subprocess
from pathlib import Path

import numpy as np


PROJECT_ROOT = Path(__file__).resolve().parents[1]
REAL_CONTROLLER = PROJECT_ROOT / "c_interface" / "real" / "real_controller"
MAGIC = 0xAA55
MODE_CONTROL = 1
MODE_FK_ONLY = 2
INPUT_STRUCT = struct.Struct("<HBBd14d14d3d6d8d")
OUTPUT_STRUCT = struct.Struct("<Hi14d14d14d6d8d6d8d12ddi")


def _build_packet(mode: int, active_arm_mask: int, *, qd_value: float = 0.0) -> bytes:
    q = np.zeros(14, dtype=np.float64)
    q[3] = np.pi / 2.0
    q[10] = np.pi / 2.0
    qd = np.zeros(14, dtype=np.float64)
    qd[:7] = float(qd_value)
    qd[7:] = float(qd_value)
    body_q = np.zeros(3, dtype=np.float64)
    target_pos = np.zeros((2, 3), dtype=np.float64)
    target_quat = np.tile([1.0, 0.0, 0.0, 0.0], (2, 1))
    return INPUT_STRUCT.pack(
        MAGIC,
        int(mode),
        int(active_arm_mask),
        0.001,
        *q,
        *qd,
        *body_q,
        *target_pos.reshape(-1),
        *target_quat.reshape(-1),
    )


def _exchange(mode: int, active_arm_mask: int, *, qd_value: float = 0.0):
    subprocess.run(["make", "-C", "c_interface", "real_controller"], cwd=PROJECT_ROOT, check=True)
    completed = subprocess.run(
        [str(REAL_CONTROLLER)],
        input=_build_packet(mode, active_arm_mask, qd_value=qd_value),
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        cwd=PROJECT_ROOT,
        check=True,
    )
    assert len(completed.stdout) == OUTPUT_STRUCT.size
    parsed = OUTPUT_STRUCT.unpack(completed.stdout)
    assert parsed[0] == MAGIC
    tau = np.asarray(parsed[2:16], dtype=np.float64)
    tau_gravity = np.asarray(parsed[16:30], dtype=np.float64)
    tau_gc = np.asarray(parsed[30:44], dtype=np.float64)
    ee_pos = np.asarray(parsed[44:50], dtype=np.float64).reshape(2, 3)
    ref_pos = np.asarray(parsed[58:64], dtype=np.float64).reshape(2, 3)
    ref_quat = np.asarray(parsed[64:72], dtype=np.float64).reshape(2, 4)
    ee_twist = np.asarray(parsed[72:84], dtype=np.float64).reshape(2, 6)
    return parsed[1], tau, tau_gravity, tau_gc, ee_pos, ref_pos, ref_quat, ee_twist


def test_real_controller_left_mask_zeroes_right_arm_output():
    status, tau, tau_gravity, tau_gc, _ee_pos, ref_pos, ref_quat, _ee_twist = _exchange(
        MODE_CONTROL, 1 << 0, qd_value=0.25
    )

    assert status == 0
    assert np.any(np.abs(tau[:7]) > 1e-9)
    assert np.any(np.abs(tau_gravity[:7]) > 1e-9)
    assert np.any(np.abs(tau_gc[:7]) > 1e-9)
    assert np.linalg.norm(ref_pos[0]) > 1e-9
    assert np.linalg.norm(ref_quat[0]) > 1e-9
    np.testing.assert_allclose(tau[7:], 0.0)
    np.testing.assert_allclose(tau_gravity[7:], 0.0)
    np.testing.assert_allclose(tau_gc[7:], 0.0)
    np.testing.assert_allclose(ref_pos[1], 0.0)
    np.testing.assert_allclose(ref_quat[1], 0.0)


def test_real_controller_right_mask_zeroes_left_arm_output():
    status, tau, tau_gravity, tau_gc, _ee_pos, ref_pos, ref_quat, _ee_twist = _exchange(
        MODE_CONTROL, 1 << 1, qd_value=0.25
    )

    assert status == 0
    np.testing.assert_allclose(tau[:7], 0.0)
    np.testing.assert_allclose(tau_gravity[:7], 0.0)
    np.testing.assert_allclose(tau_gc[:7], 0.0)
    np.testing.assert_allclose(ref_pos[0], 0.0)
    np.testing.assert_allclose(ref_quat[0], 0.0)
    assert np.any(np.abs(tau[7:]) > 1e-9)
    assert np.any(np.abs(tau_gravity[7:]) > 1e-9)
    assert np.any(np.abs(tau_gc[7:]) > 1e-9)
    assert np.linalg.norm(ref_pos[1]) > 1e-9
    assert np.linalg.norm(ref_quat[1]) > 1e-9


def test_real_controller_fk_only_returns_pose_and_zero_torque():
    status, tau, tau_gravity, tau_gc, ee_pos, ref_pos, ref_quat, ee_twist = _exchange(
        MODE_FK_ONLY, (1 << 0) | (1 << 1)
    )

    assert status == 0
    np.testing.assert_allclose(tau, 0.0)
    np.testing.assert_allclose(tau_gravity, 0.0)
    np.testing.assert_allclose(tau_gc, 0.0)
    np.testing.assert_allclose(ref_pos, 0.0)
    np.testing.assert_allclose(ref_quat, 0.0)
    assert np.all(np.isfinite(ee_pos))
    assert ee_twist.shape == (2, 6)
    assert np.all(np.isfinite(ee_twist))
    np.testing.assert_allclose(ee_twist, 0.0)
    assert np.linalg.norm(ee_pos[0]) > 0.01
    assert np.linalg.norm(ee_pos[1]) > 0.01


def test_real_controller_fk_only_returns_active_arm_twist_and_zeroes_inactive_arm():
    status, tau, tau_gravity, tau_gc, _ee_pos, ref_pos, ref_quat, ee_twist = _exchange(
        MODE_FK_ONLY, 1 << 0, qd_value=0.25
    )

    assert status == 0
    np.testing.assert_allclose(tau, 0.0)
    np.testing.assert_allclose(tau_gravity, 0.0)
    np.testing.assert_allclose(tau_gc, 0.0)
    np.testing.assert_allclose(ref_pos, 0.0)
    np.testing.assert_allclose(ref_quat, 0.0)
    assert np.all(np.isfinite(ee_twist))
    assert np.linalg.norm(ee_twist[0]) > 1e-9
    np.testing.assert_allclose(ee_twist[1], 0.0)
