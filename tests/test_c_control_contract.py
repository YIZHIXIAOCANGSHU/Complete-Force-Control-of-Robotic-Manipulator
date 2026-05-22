from __future__ import annotations

import subprocess
import sys
import textwrap
from pathlib import Path

import numpy as np
import pytest


PROJECT_ROOT = Path(__file__).resolve().parents[1]
STM32_SOURCES = [
    "control_logic.c",
    "dynamics_lib.c",
    "kinematics_lib.c",
    "math_lib.c",
    "model_lib.c",
    "stm_controller.c",
    "trajectory_lib.c",
]


sys.path.insert(0, str(PROJECT_ROOT / "python"))


def _compile_c_probe(
    tmp_path: Path, source: str, *, include_stm_controller: bool = True
) -> Path:
    probe_c = tmp_path / "probe.c"
    probe_bin = tmp_path / "probe"
    probe_c.write_text(source, encoding="utf-8")

    sources = [
        str(PROJECT_ROOT / "stm32_code" / src)
        for src in STM32_SOURCES
        if include_stm_controller or src != "stm_controller.c"
    ]
    cmd = [
        "gcc",
        "-o",
        str(probe_bin),
        str(probe_c),
        *sources,
        "-I",
        str(PROJECT_ROOT / "stm32_code"),
        "-O2",
        "-Wall",
        "-lm",
    ]
    subprocess.run(cmd, cwd=PROJECT_ROOT, check=True)
    return probe_bin


def _quat_equiv(actual: np.ndarray, expected: np.ndarray, atol: float) -> None:
    if np.linalg.norm(actual + expected) < np.linalg.norm(actual - expected):
        actual = -actual
    np.testing.assert_allclose(actual, expected, atol=atol)


def test_c_fk_matches_mujoco_tcp_pose_in_urdf_base_link(tmp_path: Path):
    pytest.importorskip("mujoco")

    from config import Config
    from sim_env import MujocoSimEnv

    q_samples = np.array(
        [
            Config.HOME_QPOS,
            Config.INIT_QPOS,
            [0.2, -0.4, 0.3, 0.8, -0.2, 0.25, -0.1],
            [-0.3, 0.1, -0.2, 1.0, 0.15, -0.2, 0.3],
        ],
        dtype=np.float64,
    )
    source = textwrap.dedent(
        """
        #include "control_logic.h"
        #include <stdio.h>

        int main(void) {
          double q[][7] = {
            {0.0, 0.0, 0.0, 1.0471975511965976, 0.0, 0.0, 0.0},
            {0.0, 0.0, 0.0, 1.5707963267948966, 0.0, 0.0, 0.0},
            {0.2, -0.4, 0.3, 0.8, -0.2, 0.25, -0.1},
            {-0.3, 0.1, -0.2, 1.0, 0.15, -0.2, 0.3},
          };
          control_init();
          for (int i = 0; i < 4; ++i) {
            double pos[3];
            double quat[4];
            control_get_fk_with_offset(q[i], pos, quat);
            printf("%.12f %.12f %.12f %.12f %.12f %.12f %.12f\\n",
                   pos[0], pos[1], pos[2], quat[0], quat[1], quat[2], quat[3]);
          }
          return 0;
        }
        """
    )
    probe = _compile_c_probe(tmp_path, source)
    c_rows = np.array(
        [[float(value) for value in line.split()] for line in subprocess.check_output([str(probe)], text=True).splitlines()],
        dtype=np.float64,
    )

    env = MujocoSimEnv()
    for index, q in enumerate(q_samples):
        env.reset(q)
        env.forward()
        np.testing.assert_allclose(c_rows[index, 0:3], env.get_ee_pos(), atol=2e-3)
        _quat_equiv(c_rows[index, 3:7], env.get_ee_quat(), atol=2e-3)


def test_python_to_c_control_packet_contains_feedback_target_and_dt():
    from state_packets import CONTROL_INPUT_PACKET_SIZE, fill_control_input_packet

    assert CONTROL_INPUT_PACKET_SIZE == 22

    packet = np.empty(CONTROL_INPUT_PACKET_SIZE, dtype=np.float64)
    fill_control_input_packet(
        packet,
        np.arange(7, dtype=np.float64),
        np.arange(10, 17, dtype=np.float64),
        np.array([0.1, 0.2, 0.3], dtype=np.float64),
        np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float64),
        0.01,
    )

    np.testing.assert_allclose(packet[0:7], np.arange(7, dtype=np.float64))
    np.testing.assert_allclose(packet[7:14], np.arange(10, 17, dtype=np.float64))
    np.testing.assert_allclose(packet[14:17], [0.1, 0.2, 0.3])
    np.testing.assert_allclose(packet[17:21], [1.0, 0.0, 0.0, 0.0])
    assert packet[21] == pytest.approx(0.01)


def test_mujoco_state_packet_sends_target_pose_not_actual_tcp():
    pytest.importorskip("mujoco")

    from config import Config
    from sim_env import MujocoSimEnv
    from state_packets import CONTROL_INPUT_PACKET_SIZE

    env = MujocoSimEnv()
    env.reset(Config.HOME_QPOS)
    env.forward()
    target_pos = np.array([0.3, 0.2, 0.4], dtype=np.float64)
    target_quat = np.array([0.5, -0.5, 0.5, -0.5], dtype=np.float64)
    env.set_target_pose(target_pos, target_quat)

    packet = np.empty(CONTROL_INPUT_PACKET_SIZE, dtype=np.float64)
    env.write_state_packet(packet)

    np.testing.assert_allclose(packet[0:7], Config.HOME_QPOS)
    np.testing.assert_allclose(packet[14:17], target_pos)
    np.testing.assert_allclose(packet[17:21], target_quat)
    assert packet[21] == pytest.approx(Config.DT)
    assert not np.allclose(packet[14:17], env.get_ee_pos())


def test_stm_controller_uses_input_dt_with_control_dt_fallback(tmp_path: Path):
    source = textwrap.dedent(
        """
        #include "stm_controller.h"
        #include <stdio.h>
        #include <string.h>

        static void fill_input(stm_input_t *in, double dt_s) {
          memset(in, 0, sizeof(*in));
          in->q[3] = 1.0471975511965976;
          in->target_pos[0] = 0.215965423312;
          in->target_pos[1] = 0.215360776000;
          in->target_pos[2] = 0.225405075559;
          in->target_quat[0] = -0.612372435696;
          in->target_quat[1] = -0.612372435696;
          in->target_quat[2] = 0.353553390593;
          in->target_quat[3] = -0.353553390593;
          in->dt_s = dt_s;
        }

        int main(void) {
          stm_input_t in;
          stm_output_t out;

          stm_controller_init();
          stm_controller_reset();
          fill_input(&in, 0.02);
          stm_controller_step(&in, &out);
          printf("%.9f\\n", out.traj_t);

          stm_controller_reset();
          fill_input(&in, -1.0);
          stm_controller_step(&in, &out);
          printf("%.9f\\n", out.traj_t);
          return 0;
        }
        """
    )
    probe = _compile_c_probe(tmp_path, source)
    values = [
        float(line)
        for line in subprocess.check_output([str(probe)], text=True).splitlines()
    ]

    assert values[0] == pytest.approx(0.02)
    assert values[1] == pytest.approx(0.001)


def test_stm_controller_reference_moves_by_configured_speed_without_reset(tmp_path: Path):
    source = textwrap.dedent(
        f"""
        #include "stm_controller.c"
        #include <stdio.h>

        int main(void) {{
          double current_pos[3] = {{0.0, 0.0, 0.0}};
          double current_quat[4] = {{1.0, 0.0, 0.0, 0.0}};
          double target_a[3] = {{1.0, 0.0, 0.0}};
          double target_b[3] = {{1.0, 1.0, 0.0}};
          double target_quat[4] = {{1.0, 0.0, 0.0, 0.0}};
          double ref_pos[3];
          double ref_quat[4];

          stm_controller_reset();
          stm_controller_update_reference(current_pos, current_quat, target_a,
                                          target_quat, 0.1, ref_pos, ref_quat);
          printf("%.9f %.9f %.9f\\n", ref_pos[0], ref_pos[1], ref_pos[2]);

          current_pos[0] = -5.0;
          current_pos[1] = -5.0;
          current_pos[2] = -5.0;
          stm_controller_update_reference(current_pos, current_quat, target_b,
                                          target_quat, 0.1, ref_pos, ref_quat);
          printf("%.9f %.9f %.9f\\n", ref_pos[0], ref_pos[1], ref_pos[2]);
          return 0;
        }}
        """
    )
    probe = _compile_c_probe(tmp_path, source, include_stm_controller=False)
    rows = np.array(
        [
            [float(value) for value in line.split()]
            for line in subprocess.check_output([str(probe)], text=True).splitlines()
        ],
        dtype=np.float64,
    )

    first_step = 0.75 * 0.1
    np.testing.assert_allclose(rows[0], [first_step, 0.0, 0.0], atol=1e-9)

    direction = np.array([1.0, 1.0, 0.0]) - rows[0]
    expected_second = rows[0] + first_step * direction / np.linalg.norm(direction)
    np.testing.assert_allclose(rows[1], expected_second, atol=1e-9)
    assert np.linalg.norm(rows[1] - rows[0]) == pytest.approx(first_step)
    assert not np.allclose(rows[1], [-5.0, -5.0, -5.0])
