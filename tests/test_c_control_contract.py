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


def _compile_host_probe(tmp_path: Path, source: str, sources: list[str]) -> Path:
    probe_c = tmp_path / "host_probe.c"
    probe_bin = tmp_path / "host_probe"
    probe_c.write_text(source, encoding="utf-8")
    cmd = [
        "gcc",
        "-o",
        str(probe_bin),
        str(probe_c),
        *[str(PROJECT_ROOT / src) for src in sources],
        "-I",
        str(PROJECT_ROOT / "stm32_code"),
        "-I",
        str(PROJECT_ROOT / "c_interface" / "h7_clock_sim"),
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


def test_c_fk_matches_mujoco_tcp_pose_in_body0422_frame(tmp_path: Path):
    pytest.importorskip("mujoco")

    from config import Config
    from sim_env import MujocoSimEnv

    arm_samples = np.array(
        [
            Config.ARM_INIT_QPOS,
            Config.ARM_INIT_QPOS,
            [0.2, -0.4, 0.3, 0.8, -0.2, 0.25, -0.1],
            [-0.3, 0.1, -0.2, 1.0, 0.15, -0.2, 0.3],
        ],
        dtype=np.float64,
    )
    body_q = np.array([0.2, -0.3, 0.4], dtype=np.float64)
    source = textwrap.dedent(
        """
        #include "control_logic.h"
        #include <stdio.h>

        int main(void) {
          double body_q[3] = {0.2, -0.3, 0.4};
          double q[][7] = {
            {0.0, 0.0, 0.0, 1.5707963267948966, 0.0, 0.0, 0.0},
            {0.0, 0.0, 0.0, 1.5707963267948966, 0.0, 0.0, 0.0},
            {0.2, -0.4, 0.3, 0.8, -0.2, 0.25, -0.1},
            {-0.3, 0.1, -0.2, 1.0, 0.15, -0.2, 0.3},
          };
          control_init();
          control_update_body_gravity(body_q);
          for (int i = 0; i < 4; ++i) {
            for (int arm = 0; arm < 2; ++arm) {
              control_arm_kinematics_t kin;
              control_get_arm_kinematics_with_offset(arm, q[i], &kin);
              printf("%d %.12f %.12f %.12f %.12f %.12f %.12f %.12f\\n",
                     arm, kin.pos[0], kin.pos[1], kin.pos[2],
                     kin.quat_wxyz[0], kin.quat_wxyz[1],
                     kin.quat_wxyz[2], kin.quat_wxyz[3]);
            }
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
    for index, arm_q in enumerate(arm_samples):
        q = np.tile(arm_q, Config.NUM_ARMS)
        env.reset(q)
        env.set_body_qpos(body_q)
        env.forward()
        for arm in range(Config.NUM_ARMS):
            row = c_rows[index * Config.NUM_ARMS + arm]
            assert int(row[0]) == arm
            np.testing.assert_allclose(
                row[1:4],
                env.base_to_target_frame_pos(env.get_ee_pos(arm)),
                atol=2e-3,
            )
            _quat_equiv(row[4:8], env.base_to_target_frame_quat(env.get_ee_quat(arm)), atol=2e-3)


def test_python_to_c_control_packet_contains_feedback_and_target_without_dt():
    from state_packets import (
        BODY_Q_OFFSET,
        CONTROL_INPUT_PACKET_SIZE,
        LEFT_TARGET_POS_OFFSET,
        LEFT_TARGET_QUAT_OFFSET,
        QD_OFFSET,
        RIGHT_TARGET_POS_OFFSET,
        RIGHT_TARGET_QUAT_OFFSET,
        fill_control_input_packet,
    )

    assert CONTROL_INPUT_PACKET_SIZE == 45

    packet = np.empty(CONTROL_INPUT_PACKET_SIZE, dtype=np.float64)
    fill_control_input_packet(
        packet,
        np.arange(14, dtype=np.float64),
        np.arange(20, 34, dtype=np.float64),
        np.array([0.1, -0.2, 0.3], dtype=np.float64),
        np.array([0.1, 0.2, 0.3], dtype=np.float64),
        np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float64),
        np.array([-0.1, -0.2, -0.3], dtype=np.float64),
        np.array([0.0, 1.0, 0.0, 0.0], dtype=np.float64),
    )

    np.testing.assert_allclose(packet[0:QD_OFFSET], np.arange(14, dtype=np.float64))
    np.testing.assert_allclose(packet[QD_OFFSET:BODY_Q_OFFSET], np.arange(20, 34, dtype=np.float64))
    np.testing.assert_allclose(packet[BODY_Q_OFFSET:LEFT_TARGET_POS_OFFSET], [0.1, -0.2, 0.3])
    np.testing.assert_allclose(packet[LEFT_TARGET_POS_OFFSET:LEFT_TARGET_QUAT_OFFSET], [0.1, 0.2, 0.3])
    np.testing.assert_allclose(packet[LEFT_TARGET_QUAT_OFFSET:RIGHT_TARGET_POS_OFFSET], [1.0, 0.0, 0.0, 0.0])
    np.testing.assert_allclose(packet[RIGHT_TARGET_POS_OFFSET:RIGHT_TARGET_QUAT_OFFSET], [-0.1, -0.2, -0.3])
    np.testing.assert_allclose(packet[RIGHT_TARGET_QUAT_OFFSET:CONTROL_INPUT_PACKET_SIZE], [0.0, 1.0, 0.0, 0.0])


def test_mujoco_state_packet_sends_target_pose_not_actual_tcp():
    pytest.importorskip("mujoco")

    from config import Config
    from sim_env import MujocoSimEnv
    from state_packets import (
        BODY_Q_OFFSET,
        CONTROL_INPUT_PACKET_SIZE,
        LEFT_TARGET_POS_OFFSET,
        LEFT_TARGET_QUAT_OFFSET,
        QD_OFFSET,
        RIGHT_TARGET_POS_OFFSET,
        RIGHT_TARGET_QUAT_OFFSET,
    )

    env = MujocoSimEnv()
    env.reset(Config.HOME_QPOS)
    env.forward()
    target_pos = np.array([0.3, 0.2, 0.4], dtype=np.float64)
    target_quat = np.array([0.5, -0.5, 0.5, -0.5], dtype=np.float64)
    right_target_pos = np.array([0.25, -0.2, 0.45], dtype=np.float64)
    right_target_quat = np.array([0.5, 0.5, -0.5, 0.5], dtype=np.float64)
    env.set_target_pose(target_pos, target_quat, arm=Config.LEFT_ARM)
    env.set_target_pose(right_target_pos, right_target_quat, arm=Config.RIGHT_ARM)

    packet = np.empty(CONTROL_INPUT_PACKET_SIZE, dtype=np.float64)
    env.write_state_packet(packet)

    np.testing.assert_allclose(packet[0:QD_OFFSET], Config.HOME_QPOS)
    np.testing.assert_allclose(packet[BODY_Q_OFFSET:LEFT_TARGET_POS_OFFSET], env.get_body_qpos())
    np.testing.assert_allclose(packet[LEFT_TARGET_POS_OFFSET:LEFT_TARGET_QUAT_OFFSET], target_pos)
    np.testing.assert_allclose(packet[LEFT_TARGET_QUAT_OFFSET:RIGHT_TARGET_POS_OFFSET], target_quat)
    np.testing.assert_allclose(packet[RIGHT_TARGET_POS_OFFSET:RIGHT_TARGET_QUAT_OFFSET], right_target_pos)
    np.testing.assert_allclose(packet[RIGHT_TARGET_QUAT_OFFSET:CONTROL_INPUT_PACKET_SIZE], right_target_quat)
    assert not np.allclose(packet[LEFT_TARGET_POS_OFFSET:LEFT_TARGET_QUAT_OFFSET], env.get_ee_pos(Config.LEFT_ARM))
    assert not np.allclose(packet[RIGHT_TARGET_POS_OFFSET:RIGHT_TARGET_QUAT_OFFSET], env.get_ee_pos(Config.RIGHT_ARM))


def test_dual_control_zero_pose_error_outputs_bias_compensation_only(tmp_path: Path):
    source = textwrap.dedent(
        """
        #include "control_logic.h"
        #include <math.h>
        #include <stdio.h>

        int main(void) {
          double q[14] = {
            0.0, 0.0, 0.0, 1.5707963267948966, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 1.5707963267948966, 0.0, 0.0, 0.0
          };
          double qd[14] = {0.0};
          double target_pos[2][3];
          double target_quat[2][4];
          control_arm_kinematics_t kin[2];
          RBDLModel model;
          double tau_gc[7];
          double tau[14];
          double max_extra = 0.0;

          control_init();
          for (int arm = 0; arm < 2; ++arm) {
            int offset = arm * 7;
            control_get_arm_kinematics_with_offset(arm, q + offset, &kin[arm]);
            for (int i = 0; i < 3; ++i) {
              target_pos[arm][i] = kin[arm].pos[i];
            }
            for (int i = 0; i < 4; ++i) {
              target_quat[arm][i] = kin[arm].quat_wxyz[i];
            }
          }
          control_step_v2_dual_with_state(target_pos, target_quat, q, qd, kin, tau);

          for (int arm = 0; arm < 2; ++arm) {
            int offset = arm * 7;
            build_am_d02_arm_model(arm, &model);
            rbdl_calc_gc(&model, q + offset, qd + offset, tau_gc);
            for (int i = 0; i < 7; ++i) {
              double extra = fabs(tau[offset + i] - tau_gc[i]);
              if (extra > max_extra) {
                max_extra = extra;
              }
            }
          }

          printf("%.12f\\n", max_extra);
          return 0;
        }
        """
    )
    probe = _compile_c_probe(tmp_path, source)
    max_extra = float(subprocess.check_output([str(probe)], text=True).strip())

    assert max_extra < 1e-7


def test_dual_control_nonzero_pose_error_adds_cartesian_correction(tmp_path: Path):
    source = textwrap.dedent(
        """
        #include "control_logic.h"
        #include <math.h>
        #include <stdio.h>

        int main(void) {
          double q[14] = {
            0.0, 0.0, 0.0, 1.5707963267948966, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 1.5707963267948966, 0.0, 0.0, 0.0
          };
          double qd[14] = {0.0};
          double target_pos[2][3];
          double target_quat[2][4];
          control_arm_kinematics_t kin[2];
          RBDLModel model;
          double tau_gc[7];
          double tau[14];
          double max_extra = 0.0;

          control_init();
          for (int arm = 0; arm < 2; ++arm) {
            int offset = arm * 7;
            control_get_arm_kinematics_with_offset(arm, q + offset, &kin[arm]);
            for (int i = 0; i < 3; ++i) {
              target_pos[arm][i] = kin[arm].pos[i];
            }
            for (int i = 0; i < 4; ++i) {
              target_quat[arm][i] = kin[arm].quat_wxyz[i];
            }
            target_pos[arm][0] += 0.02;
            target_pos[arm][2] += 0.01;
          }
          control_step_v2_dual_with_state(target_pos, target_quat, q, qd, kin, tau);

          for (int arm = 0; arm < 2; ++arm) {
            int offset = arm * 7;
            build_am_d02_arm_model(arm, &model);
            rbdl_calc_gc(&model, q + offset, qd + offset, tau_gc);
            for (int i = 0; i < 7; ++i) {
              double extra = fabs(tau[offset + i] - tau_gc[i]);
              if (extra > max_extra) {
                max_extra = extra;
              }
            }
          }

          printf("%.12f\\n", max_extra);
          return 0;
        }
        """
    )
    probe = _compile_c_probe(tmp_path, source)
    max_extra = float(subprocess.check_output([str(probe)], text=True).strip())

    assert max_extra > 0.01


def test_dual_control_precomputed_fk_jacobian_matches_per_arm_dispatch(tmp_path: Path):
    source = textwrap.dedent(
        """
        #include "control_logic.h"
        #include <math.h>
        #include <stdio.h>

        int main(void) {
          double q[14] = {
            0.12, -0.21, 0.18, 1.20, -0.10, 0.16, -0.08,
            -0.10, 0.18, -0.16, 1.10, 0.12, -0.14, 0.06
          };
          double qd[14] = {
            0.03, -0.02, 0.01, -0.04, 0.02, -0.01, 0.005,
            -0.025, 0.015, -0.01, 0.035, -0.02, 0.012, -0.006
          };
          double target_pos[2][3];
          double target_quat[2][4];
          double tau_dual[14];
          double tau_arm[14] = {0.0};
          control_arm_kinematics_t kin[2];
          double max_diff = 0.0;

          control_init();
          for (int arm = 0; arm < 2; ++arm) {
            int offset = arm * 7;
            control_get_arm_kinematics_with_offset(arm, q + offset, &kin[arm]);
            target_pos[arm][0] = kin[arm].pos[0] + (arm == 0 ? 0.015 : -0.012);
            target_pos[arm][1] = kin[arm].pos[1] + (arm == 0 ? -0.008 : 0.010);
            target_pos[arm][2] = kin[arm].pos[2] + 0.006;
            for (int i = 0; i < 4; ++i) {
              target_quat[arm][i] = kin[arm].quat_wxyz[i];
            }
          }

          control_step_v2_dual_with_state(target_pos, target_quat, q, qd, kin, tau_dual);
          for (int arm = 0; arm < 2; ++arm) {
            int offset = arm * 7;
            control_step_v2_arm_with_state(arm, target_pos[arm], target_quat[arm],
                                           q + offset, qd + offset, &kin[arm],
                                           tau_arm + offset);
          }

          for (int i = 0; i < 14; ++i) {
            double diff = fabs(tau_dual[i] - tau_arm[i]);
            if (diff > max_diff) {
              max_diff = diff;
            }
          }
          printf("%.18f\\n", max_diff);
          return 0;
        }
        """
    )
    probe = _compile_c_probe(tmp_path, source)
    max_diff = float(subprocess.check_output([str(probe)], text=True).strip())

    assert max_diff < 1e-10


def test_trajectory_lib_remains_available_for_endpoint_path_planning(tmp_path: Path):
    source = textwrap.dedent(
        """
        #include "trajectory_lib.h"
        #include <stdio.h>

        int main(void) {
          LinearPathPlanner planner;
          double start_pos[3] = {0.0, 0.0, 0.0};
          double end_pos[3] = {1.0, 0.0, 0.0};
          double start_quat[4] = {1.0, 0.0, 0.0, 0.0};
          double end_quat[4] = {1.0, 0.0, 0.0, 0.0};
          double pos[3];
          double quat[4];

          linear_path_init(&planner, start_pos, start_quat, end_pos, end_quat, 0.5, 1.0);
          linear_path_evaluate(&planner, 0.1, pos, quat);
          printf("%.12f %.12f %.12f %.12f\\n", pos[0], pos[1], pos[2], planner.total_time);
          return 0;
        }
        """
    )
    probe = _compile_c_probe(tmp_path, source)
    values = [
        float(value)
        for value in subprocess.check_output([str(probe)], text=True).split()
    ]

    assert 0.0 < values[0] < 1.0
    assert values[1] == pytest.approx(0.0)
    assert values[2] == pytest.approx(0.0)
    assert values[3] > 0.0


def test_c_body_gravity_uses_three_body_joint_angles(tmp_path: Path):
    source = textwrap.dedent(
        """
        #include "control_logic.h"
        #include <stdio.h>

        static void print_tau(const double body_q[3]) {
          double q[7] = {0.0, 0.0, 0.0, 1.5707963267948966, 0.0, 0.0, 0.0};
          double qd[7] = {0.0};
          double pos[3];
          double quat[4];
          double tau[7];
          control_arm_kinematics_t kin;

          control_update_body_gravity(body_q);
          control_get_arm_kinematics_with_offset(ARM_LEFT, q, &kin);
          for (int i = 0; i < 3; ++i) {
            pos[i] = kin.pos[i];
          }
          for (int i = 0; i < 4; ++i) {
            quat[i] = kin.quat_wxyz[i];
          }
          control_step_v2_arm_with_state(ARM_LEFT, pos, quat, q, qd, &kin, tau);
          printf("%.12f %.12f %.12f\\n", tau[0], tau[1], tau[2]);
        }

        int main(void) {
          double zero_q[3] = {0.0, 0.0, 0.0};
          double body_q[3] = {0.3, -0.4, 0.2};

          control_init();
          print_tau(zero_q);
          print_tau(body_q);
          return 0;
        }
        """
    )
    probe = _compile_c_probe(tmp_path, source)
    rows = np.array(
        [
            [float(value) for value in line.split()]
            for line in subprocess.check_output([str(probe)], text=True).splitlines()
        ],
        dtype=np.float64,
    )

    assert np.isfinite(rows).all()
    assert not np.allclose(rows[0], rows[1])


def test_c_safety_uses_urdf_right_arm_position_limits_and_per_joint_velocity(tmp_path: Path):
    source = textwrap.dedent(
        """
        #include "control_logic.h"
        #include <stdio.h>

        static void print_status(const char *label, int side, double q[7], double qd[7]) {
          printf("%s %d\\n", label, control_check_safety_arm(side, q, qd));
        }

        static void fill_mid(double q[7], const double qmin[7], const double qmax[7]) {
          for (int i = 0; i < 7; ++i) {
            q[i] = 0.5 * (qmin[i] + qmax[i]);
          }
        }

        int main(void) {
          double right_min[7] = {
            RIGHT_JOINT_POS_MIN_1, RIGHT_JOINT_POS_MIN_2, RIGHT_JOINT_POS_MIN_3,
            RIGHT_JOINT_POS_MIN_4, RIGHT_JOINT_POS_MIN_5, RIGHT_JOINT_POS_MIN_6,
            RIGHT_JOINT_POS_MIN_7
          };
          double right_max[7] = {
            RIGHT_JOINT_POS_MAX_1, RIGHT_JOINT_POS_MAX_2, RIGHT_JOINT_POS_MAX_3,
            RIGHT_JOINT_POS_MAX_4, RIGHT_JOINT_POS_MAX_5, RIGHT_JOINT_POS_MAX_6,
            RIGHT_JOINT_POS_MAX_7
          };
          double left_min[7] = {
            JOINT_POS_MIN_1, JOINT_POS_MIN_2, JOINT_POS_MIN_3, JOINT_POS_MIN_4,
            JOINT_POS_MIN_5, JOINT_POS_MIN_6, JOINT_POS_MIN_7
          };
          double left_max[7] = {
            JOINT_POS_MAX_1, JOINT_POS_MAX_2, JOINT_POS_MAX_3, JOINT_POS_MAX_4,
            JOINT_POS_MAX_5, JOINT_POS_MAX_6, JOINT_POS_MAX_7
          };
          double right_vel[7] = {
            RIGHT_JOINT_VEL_LIMIT_1, RIGHT_JOINT_VEL_LIMIT_2, RIGHT_JOINT_VEL_LIMIT_3,
            RIGHT_JOINT_VEL_LIMIT_4, RIGHT_JOINT_VEL_LIMIT_5, RIGHT_JOINT_VEL_LIMIT_6,
            RIGHT_JOINT_VEL_LIMIT_7
          };
          double left_vel[7] = {
            JOINT_VEL_LIMIT_1, JOINT_VEL_LIMIT_2, JOINT_VEL_LIMIT_3, JOINT_VEL_LIMIT_4,
            JOINT_VEL_LIMIT_5, JOINT_VEL_LIMIT_6, JOINT_VEL_LIMIT_7
          };
          double q[7] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
          double qd[7] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

          control_init();
          fill_mid(q, right_min, right_max);
          print_status("right_center", ARM_RIGHT, q, qd);
          for (int i = 0; i < 7; ++i) {
            q[i] = right_min[i] - 0.011;
            print_status("right_low", ARM_RIGHT, q, qd);
            q[i] = right_max[i] + 0.011;
            print_status("right_high", ARM_RIGHT, q, qd);
            q[i] = 0.5 * (right_min[i] + right_max[i]);
          }

          fill_mid(q, left_min, left_max);
          q[0] = left_min[0] - 0.011;
          print_status("left_j1_low", ARM_LEFT, q, qd);
          q[0] = 0.5 * (left_min[0] + left_max[0]);
          q[6] = left_max[6] + 0.011;
          print_status("left_j7_high", ARM_LEFT, q, qd);
          q[6] = 0.5 * (left_min[6] + left_max[6]);

          fill_mid(q, right_min, right_max);
          qd[0] = right_vel[0] - 0.001;
          print_status("right_vel_ok", ARM_RIGHT, q, qd);
          qd[0] = right_vel[0] + 0.001;
          print_status("right_vel_high", ARM_RIGHT, q, qd);
          qd[0] = 0.0;
          fill_mid(q, left_min, left_max);
          qd[6] = -left_vel[6] - 0.001;
          print_status("left_vel_low", ARM_LEFT, q, qd);
          return 0;
        }
        """
    )
    probe = _compile_c_probe(tmp_path, source)
    lines = subprocess.check_output([str(probe)], text=True, stderr=subprocess.DEVNULL).splitlines()
    statuses: dict[str, list[int]] = {}
    for line in lines:
      label, status = line.split()
      statuses.setdefault(label, []).append(int(status))

    assert statuses["right_center"] == [0]
    assert statuses["right_low"] == [-1] * 7
    assert statuses["right_high"] == [-1] * 7
    assert statuses["left_j1_low"] == [-1]
    assert statuses["left_j7_high"] == [-1]
    assert statuses["right_vel_ok"] == [0]
    assert statuses["right_vel_high"] == [-2]
    assert statuses["left_vel_low"] == [-2]


def test_stm_controller_step_elapsed_uses_supplied_h7_elapsed_time(tmp_path: Path):
    source = textwrap.dedent(
        """
        #include "control_logic.h"
        #include "stm_controller.h"
        #include <math.h>
        #include <stdio.h>
        #include <string.h>

        static void fill_input(stm_input_t *in) {
          memset(in, 0, sizeof(*in));
          in->q[3] = 1.5707963267948966;
          in->q[10] = 1.5707963267948966;
          for (int arm = 0; arm < 2; ++arm) {
            in->target_pos[arm][0] = 0.382108000000;
            in->target_pos[arm][1] = arm == 0 ? 0.215360776000 : -0.215360776000;
            in->target_pos[arm][2] = 0.304749500000;
            in->target_quat[arm][0] = -0.500000000000;
            in->target_quat[arm][1] = -0.500000000000;
            in->target_quat[arm][2] = -0.500000000000;
            in->target_quat[arm][3] = 0.500000000000;
          }
        }

        int main(void) {
          stm_input_t in;
          stm_output_t out;

          stm_controller_init();
          stm_controller_reset();
          fill_input(&in);
          stm_controller_step_elapsed(&in, &out, 0.0);
          printf("%.9f %d %.9f\\n", out.traj_t, out.step_count, out.tau[3]);
          stm_controller_step_elapsed(&in, &out, 0.002);
          stm_controller_step_elapsed(&in, &out, 0.003);
          printf("%.9f\\n", out.traj_t);
          printf("%d\\n", out.step_count);
          return 0;
        }
        """
    )
    probe = _compile_c_probe(tmp_path, source)
    values = [
        float(line)
        for line in subprocess.check_output([str(probe)], text=True).split()
    ]

    assert values[0] == pytest.approx(0.0)
    assert values[1] == pytest.approx(1)
    assert np.isfinite(values[2])
    assert values[3] == pytest.approx(0.005)
    assert values[4] == pytest.approx(3)


def test_stm_controller_elapsed_time_is_sanitized_and_clamped(tmp_path: Path):
    source = textwrap.dedent(
        """
        #include "control_logic.h"
        #include "stm_controller.h"
        #include <math.h>
        #include <stdio.h>
        #include <string.h>

        static void fill_input(stm_input_t *in) {
          memset(in, 0, sizeof(*in));
          in->q[3] = 1.5707963267948966;
          in->q[10] = 1.5707963267948966;
          for (int arm = 0; arm < 2; ++arm) {
            in->target_pos[arm][0] = 0.382108000000;
            in->target_pos[arm][1] = arm == 0 ? 0.215360776000 : -0.215360776000;
            in->target_pos[arm][2] = 0.304749500000;
            in->target_quat[arm][0] = -0.500000000000;
            in->target_quat[arm][1] = -0.500000000000;
            in->target_quat[arm][2] = -0.500000000000;
            in->target_quat[arm][3] = 0.500000000000;
          }
        }

        int main(void) {
          stm_input_t in;
          stm_output_t out;

          stm_controller_init();
          stm_controller_reset();
          fill_input(&in);

          stm_controller_step_elapsed(&in, &out, -1.0);
          printf("%.9f\\n", out.traj_t);
          stm_controller_step_elapsed(&in, &out, NAN);
          printf("%.9f\\n", out.traj_t);
          stm_controller_step_elapsed(&in, &out, 1.0);
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

    assert values[0] == pytest.approx(0.0)
    assert values[1] == pytest.approx(0.0)
    assert values[2] == pytest.approx(0.02)


def test_stm_controller_safety_latch_outputs_zero_until_zero_hold_then_replans(tmp_path: Path):
    source = textwrap.dedent(
        """
        #include "stm_controller.c"
        #include <math.h>
        #include <stdio.h>
        #include <string.h>

        static void fill_zero_input(stm_input_t *in) {
          memset(in, 0, sizeof(*in));
          for (int arm = 0; arm < NUM_ARMS; ++arm) {
            int offset = arm * ARM_JOINTS;
            in->q[offset + 3] = 1.0;
          }
          for (int arm = 0; arm < NUM_ARMS; ++arm) {
            for (int i = 0; i < 3; ++i) {
              in->target_pos[arm][i] = NAN;
            }
            for (int i = 0; i < 4; ++i) {
              in->target_quat[arm][i] = NAN;
            }
          }
        }

        static double tau_abs_sum(const stm_output_t *out) {
          double sum = 0.0;
          for (int i = 0; i < NUM_JOINTS; ++i) {
            sum += fabs(out->tau[i]);
          }
          return sum;
        }

        static void step_and_print(const char *label, stm_input_t *in,
                                   stm_output_t *out, double elapsed_s) {
          stm_controller_step_elapsed(in, out, elapsed_s);
          printf("%s %d %.12f %d %d %d %.12f %.12f %.12f %.6f\\n", label, out->status,
                 tau_abs_sum(out), g_controller.safety_latched,
                 g_controller.target_valid[ARM_LEFT],
                 g_controller.ref_valid[ARM_LEFT],
                 g_controller.path_progress_t[ARM_LEFT],
                 g_controller.path_gate[ARM_LEFT],
                 g_controller.path_lookahead_m[ARM_LEFT],
                 out->traj_t);
        }

        int main(void) {
          stm_input_t in;
          stm_output_t out;

          stm_controller_init();
          stm_controller_reset();
          fill_zero_input(&in);

          step_and_print("initial_ok", &in, &out, 0.001);

          fill_zero_input(&in);
          in.q[0] = JOINT_POS_MAX_1 + 0.02;
          step_and_print("pos_limit", &in, &out, 0.001);

          fill_zero_input(&in);
          in.qd[0] = 1.0;
          step_and_print("not_zero_qd", &in, &out, 0.020);

          fill_zero_input(&in);
          in.q[3] = 1.0;
          step_and_print("hold_02", &in, &out, 0.020);
          in.q[3] = 1.0;
          step_and_print("hold_04", &in, &out, 0.020);
          in.q[3] = 1.0;
          step_and_print("hold_06", &in, &out, 0.020);
          in.q[3] = 1.0;
          step_and_print("hold_08", &in, &out, 0.020);
          in.q[3] = 1.0;
          step_and_print("hold_10", &in, &out, 0.020);
          step_and_print("recovered", &in, &out, 0.001);
          return 0;
        }
        """
    )
    probe = _compile_c_probe(tmp_path, source, include_stm_controller=False)
    rows = {}
    for line in subprocess.check_output([str(probe)], text=True).splitlines():
        (
            label,
            status,
            tau_sum,
            latched,
            target_valid,
            ref_valid,
            path_progress_t,
            path_gate,
            path_lookahead_m,
            traj_t,
        ) = line.split()
        rows[label] = {
            "status": int(status),
            "tau_sum": float(tau_sum),
            "latched": int(latched),
            "target_valid": int(target_valid),
            "ref_valid": int(ref_valid),
            "path_progress_t": float(path_progress_t),
            "path_gate": float(path_gate),
            "path_lookahead_m": float(path_lookahead_m),
            "traj_t": float(traj_t),
        }

    assert rows["initial_ok"]["status"] == 0
    assert rows["initial_ok"]["target_valid"] == 1

    assert rows["pos_limit"]["status"] == -1
    assert rows["pos_limit"]["tau_sum"] == pytest.approx(0.0)
    assert rows["pos_limit"]["latched"] == 1

    assert rows["not_zero_qd"]["status"] == -2
    assert rows["not_zero_qd"]["tau_sum"] == pytest.approx(0.0)
    assert rows["not_zero_qd"]["latched"] == 1

    assert rows["hold_02"]["status"] == -2
    assert rows["hold_02"]["latched"] == 1
    assert rows["hold_04"]["status"] == -2
    assert rows["hold_04"]["latched"] == 1
    assert rows["hold_06"]["status"] == -2
    assert rows["hold_06"]["latched"] == 1
    assert rows["hold_08"]["status"] == -2
    assert rows["hold_08"]["latched"] == 1

    assert rows["hold_10"]["status"] == -2
    assert rows["hold_10"]["latched"] == 0
    assert rows["hold_10"]["target_valid"] == 0
    assert rows["hold_10"]["ref_valid"] == 0
    assert rows["hold_10"]["path_progress_t"] == pytest.approx(0.0)
    assert rows["hold_10"]["path_gate"] == pytest.approx(0.0)
    assert rows["hold_10"]["path_lookahead_m"] == pytest.approx(0.0)
    assert rows["hold_10"]["tau_sum"] == pytest.approx(0.0)

    assert rows["recovered"]["status"] == 0
    assert rows["recovered"]["latched"] == 0
    assert rows["recovered"]["target_valid"] == 1
    assert rows["recovered"]["ref_valid"] == 1
    assert rows["recovered"]["path_progress_t"] > 0.0
    assert rows["recovered"]["path_gate"] == pytest.approx(1.0)
    assert rows["recovered"]["path_lookahead_m"] > 0.0
    assert rows["recovered"]["traj_t"] > rows["hold_10"]["traj_t"]


def test_stm_controller_velocity_limit_enters_safety_latch_with_zero_torque(tmp_path: Path):
    source = textwrap.dedent(
        """
        #include "stm_controller.h"
        #include <math.h>
        #include <stdio.h>
        #include <string.h>

        int main(void) {
          stm_input_t in;
          stm_output_t out;
          double tau_sum = 0.0;

          memset(&in, 0, sizeof(in));
          for (int arm = 0; arm < NUM_ARMS; ++arm) {
            for (int i = 0; i < 3; ++i) {
              in.target_pos[arm][i] = NAN;
            }
            for (int i = 0; i < 4; ++i) {
              in.target_quat[arm][i] = NAN;
            }
          }
          in.qd[ARM_RIGHT * ARM_JOINTS] = RIGHT_JOINT_VEL_LIMIT_1 + 0.5;

          stm_controller_init();
          stm_controller_reset();
          stm_controller_step_elapsed(&in, &out, 0.001);

          for (int i = 0; i < NUM_JOINTS; ++i) {
            tau_sum += fabs(out.tau[i]);
          }
          printf("%d %.12f\\n", out.status, tau_sum);
          return 0;
        }
        """
    )
    probe = _compile_c_probe(tmp_path, source)
    status, tau_sum = subprocess.check_output([str(probe)], text=True).split()

    assert int(status) == -1
    assert float(tau_sum) == pytest.approx(0.0)


def test_control_safety_insets_position_limits_and_uses_4rad_velocity_limit(tmp_path: Path):
    source = textwrap.dedent(
        """
        #include "control_logic.h"
        #include <stdio.h>

        int main(void) {
          double q[7] = {0.0};
          double qd[7] = {0.0};
          double span = RIGHT_JOINT_POS_MAX_1 - RIGHT_JOINT_POS_MIN_1;
          double safe_min = RIGHT_JOINT_POS_MIN_1 + CONTROL_JOINT_LIMIT_INSET_RATIO * span;
          double safe_max = RIGHT_JOINT_POS_MAX_1 - CONTROL_JOINT_LIMIT_INSET_RATIO * span;

          for (int i = 0; i < 7; ++i) {
            q[i] = 0.5 * (RIGHT_JOINT_POS_MIN_1 + RIGHT_JOINT_POS_MAX_1);
          }

          q[0] = safe_min - 0.0001;
          printf("inset_low %d\\n", control_check_safety_arm(ARM_RIGHT, q, qd));
          q[0] = safe_max + 0.0001;
          printf("inset_high %d\\n", control_check_safety_arm(ARM_RIGHT, q, qd));
          q[0] = 0.5 * (safe_min + safe_max);
          qd[0] = 4.001;
          printf("vel_high %d %.3f %.2f\\n", control_check_safety_arm(ARM_RIGHT, q, qd),
                 JOINT_VEL_LIMIT, CONTROL_JOINT_LIMIT_INSET_RATIO);
          return 0;
        }
        """
    )
    probe = _compile_c_probe(tmp_path, source)
    rows = {}
    for line in subprocess.check_output([str(probe)], text=True).splitlines():
        parts = line.split()
        rows[parts[0]] = parts[1:]

    assert int(rows["inset_low"][0]) == -1
    assert int(rows["inset_high"][0]) == -1
    assert int(rows["vel_high"][0]) == -2
    assert float(rows["vel_high"][1]) == pytest.approx(4.0)
    assert float(rows["vel_high"][2]) == pytest.approx(0.01)


def test_stm_controller_active_arm_mask_controls_only_selected_arm(tmp_path: Path):
    source = textwrap.dedent(
        """
        #include "control_logic.h"
        #include "stm_controller.h"
        #include <math.h>
        #include <stdio.h>
        #include <string.h>

        static void fill_input(stm_input_t *in) {
          memset(in, 0, sizeof(*in));
          in->q[0] = 0.10;
          in->q[1] = -0.20;
          in->q[2] = 0.15;
          in->q[3] = 1.10;
          in->q[4] = -0.08;
          in->q[5] = 0.12;
          in->q[6] = -0.05;
          in->q[7] = -0.12;
          in->q[8] = 0.18;
          in->q[9] = -0.14;
          in->q[10] = 1.12;
          in->q[11] = 0.10;
          in->q[12] = -0.11;
          in->q[13] = 0.04;
          for (int arm = 0; arm < NUM_ARMS; ++arm) {
            int offset = arm * ARM_JOINTS;
            control_arm_kinematics_t kin;
            control_get_arm_kinematics_with_offset(arm, in->q + offset, &kin);
            for (int i = 0; i < 3; ++i) {
              in->target_pos[arm][i] = kin.pos[i];
            }
            for (int i = 0; i < 4; ++i) {
              in->target_quat[arm][i] = kin.quat_wxyz[i];
            }
            in->target_pos[arm][0] += arm == ARM_LEFT ? 0.02 : -0.02;
            in->target_pos[arm][2] += 0.01;
          }
        }

        static double abs_sum(const double *values, int start, int count) {
          double sum = 0.0;
          for (int i = 0; i < count; ++i) {
            sum += fabs(values[start + i]);
          }
          return sum;
        }

        int main(void) {
          stm_input_t in;
          stm_output_t out;

          stm_controller_init();

          fill_input(&in);
          in.active_arm_mask = STM_ARM_MASK_LEFT;
          in.q[ARM_RIGHT * ARM_JOINTS] = RIGHT_JOINT_POS_MAX_1 + 1.0;
          in.qd[ARM_RIGHT * ARM_JOINTS] = NAN;
          stm_controller_reset();
          stm_controller_step_elapsed(&in, &out, 0.001);
          printf("left %d %.12f %.12f\\n", out.status,
                 abs_sum(out.tau, 0, ARM_JOINTS),
                 abs_sum(out.tau, ARM_JOINTS, ARM_JOINTS));

          fill_input(&in);
          in.active_arm_mask = STM_ARM_MASK_RIGHT;
          in.q[0] = JOINT_POS_MAX_1 + 1.0;
          in.qd[0] = NAN;
          stm_controller_reset();
          stm_controller_step_elapsed(&in, &out, 0.001);
          printf("right %d %.12f %.12f\\n", out.status,
                 abs_sum(out.tau, 0, ARM_JOINTS),
                 abs_sum(out.tau, ARM_JOINTS, ARM_JOINTS));

          fill_input(&in);
          in.active_arm_mask = 0;
          stm_controller_reset();
          stm_controller_step_elapsed(&in, &out, 0.001);
          printf("default %d %.12f %.12f\\n", out.status,
                 abs_sum(out.tau, 0, ARM_JOINTS),
                 abs_sum(out.tau, ARM_JOINTS, ARM_JOINTS));
          return 0;
        }
        """
    )
    probe = _compile_c_probe(tmp_path, source)
    rows = {}
    for line in subprocess.check_output([str(probe)], text=True).splitlines():
        label, status, left_sum, right_sum = line.split()
        rows[label] = {
            "status": int(status),
            "left_sum": float(left_sum),
            "right_sum": float(right_sum),
        }

    assert rows["left"]["status"] == 0
    assert rows["left"]["left_sum"] > 0.0
    assert rows["left"]["right_sum"] == pytest.approx(0.0)
    assert rows["right"]["status"] == 0
    assert rows["right"]["left_sum"] == pytest.approx(0.0)
    assert rows["right"]["right_sum"] > 0.0
    assert rows["default"]["status"] == 0
    assert rows["default"]["left_sum"] > 0.0
    assert rows["default"]["right_sum"] > 0.0


def test_stm_controller_single_arm_safety_recovery_uses_selected_arm_velocity_only(
    tmp_path: Path,
):
    source = textwrap.dedent(
        """
        #include "stm_controller.c"
        #include <math.h>
        #include <stdio.h>
        #include <string.h>

        static void fill_input(stm_input_t *in, double left_qd, double right_qd) {
          memset(in, 0, sizeof(*in));
          in->active_arm_mask = STM_ARM_MASK_LEFT;
          in->qd[0] = left_qd;
          in->qd[ARM_RIGHT * ARM_JOINTS] = right_qd;
          for (int arm = 0; arm < NUM_ARMS; ++arm) {
            for (int i = 0; i < 3; ++i) {
              in->target_pos[arm][i] = NAN;
            }
            for (int i = 0; i < 4; ++i) {
              in->target_quat[arm][i] = NAN;
            }
          }
        }

        static void print_step(const char *label, stm_input_t *in,
                               stm_output_t *out, double elapsed_s) {
          stm_controller_step_elapsed(in, out, elapsed_s);
          printf("%s %d %d %.6f\\n", label, out->status,
                 g_controller.safety_latched, g_controller.zero_hold_s);
        }

        int main(void) {
          stm_input_t in;
          stm_output_t out;

          stm_controller_init();
          stm_controller_reset();

          fill_input(&in, JOINT_VEL_LIMIT_1 + 1.0, RIGHT_JOINT_VEL_LIMIT_1 + 1.0);
          print_step("trigger", &in, &out, 0.001);

          fill_input(&in, 0.0, RIGHT_JOINT_VEL_LIMIT_1 + 1.0);
          print_step("hold_02", &in, &out, 0.020);
          print_step("hold_04", &in, &out, 0.020);
          print_step("hold_06", &in, &out, 0.020);
          print_step("hold_08", &in, &out, 0.020);
          print_step("hold_10", &in, &out, 0.020);
          return 0;
        }
        """
    )
    probe = _compile_c_probe(tmp_path, source, include_stm_controller=False)
    rows = {}
    for line in subprocess.check_output([str(probe)], text=True).splitlines():
        label, status, latched, hold_s = line.split()
        rows[label] = {
            "status": int(status),
            "latched": int(latched),
            "hold_s": float(hold_s),
        }

    assert rows["trigger"]["status"] == -1
    assert rows["trigger"]["latched"] == 1
    assert rows["hold_08"]["status"] == -2
    assert rows["hold_08"]["latched"] == 1
    assert rows["hold_10"]["status"] == -2
    assert rows["hold_10"]["latched"] == 0


def test_stm_controller_safety_recovery_ignores_joint_position_and_requires_still_velocity(
    tmp_path: Path,
):
    source = textwrap.dedent(
        """
        #include "stm_controller.c"
        #include <math.h>
        #include <stdio.h>
        #include <string.h>

        static void fill_input(stm_input_t *in, double q3, double qd0) {
          memset(in, 0, sizeof(*in));
          in->q[3] = q3;
          in->qd[0] = qd0;
          for (int arm = 0; arm < NUM_ARMS; ++arm) {
            for (int i = 0; i < 3; ++i) {
              in->target_pos[arm][i] = NAN;
            }
            for (int i = 0; i < 4; ++i) {
              in->target_quat[arm][i] = NAN;
            }
          }
        }

        static void print_step(const char *label, stm_input_t *in, stm_output_t *out,
                               double elapsed_s) {
          stm_controller_step_elapsed(in, out, elapsed_s);
          printf("%s %d %d %.6f\\n", label, out->status,
                 g_controller.safety_latched, g_controller.zero_hold_s);
        }

        int main(void) {
          stm_input_t in;
          stm_output_t out;

          stm_controller_init();
          stm_controller_reset();

          fill_input(&in, 0.0, RIGHT_JOINT_VEL_LIMIT_1 + 0.5);
          print_step("trigger", &in, &out, 0.001);

          fill_input(&in, 1.0, 1.0);
          print_step("moving", &in, &out, 0.020);

          fill_input(&in, 1.0, 0.0);
          print_step("still_02", &in, &out, 0.020);
          print_step("still_04", &in, &out, 0.020);
          print_step("still_06", &in, &out, 0.020);
          print_step("still_08", &in, &out, 0.020);
          print_step("still_10", &in, &out, 0.020);
          return 0;
        }
        """
    )
    probe = _compile_c_probe(tmp_path, source, include_stm_controller=False)
    rows = {}
    for line in subprocess.check_output([str(probe)], text=True).splitlines():
        label, status, latched, hold_s = line.split()
        rows[label] = {
            "status": int(status),
            "latched": int(latched),
            "hold_s": float(hold_s),
        }

    assert rows["trigger"]["status"] == -1
    assert rows["trigger"]["latched"] == 1
    assert rows["moving"]["status"] == -2
    assert rows["moving"]["latched"] == 1
    assert rows["moving"]["hold_s"] == pytest.approx(0.0)
    assert rows["still_08"]["status"] == -2
    assert rows["still_08"]["latched"] == 1
    assert rows["still_10"]["status"] == -2
    assert rows["still_10"]["latched"] == 0


def test_stm_input_no_longer_exposes_sim_dt(tmp_path: Path):
    source = textwrap.dedent(
        """
        #include "stm_controller.h"

        int main(void) {
          stm_input_t in;
          in.dt_s = 0.01;
          return 0;
        }
        """
    )
    probe_c = tmp_path / "probe.c"
    probe_bin = tmp_path / "probe"
    probe_c.write_text(source, encoding="utf-8")
    cmd = [
        "gcc",
        "-o",
        str(probe_bin),
        str(probe_c),
        "-I",
        str(PROJECT_ROOT / "stm32_code"),
        "-O2",
        "-Wall",
        "-Werror",
        "-lm",
    ]
    completed = subprocess.run(cmd, cwd=PROJECT_ROOT, text=True, capture_output=True)
    assert completed.returncode != 0
    assert "dt_s" in completed.stderr


@pytest.mark.parametrize(
    ("source", "missing_name"),
    [
        (
            """
            #include "stm_controller.h"
            int main(void) {
              stm_input_t in;
              stm_output_t out;
              stm_controller_step(&in, &out);
              return 0;
            }
            """,
            "stm_controller_step",
        ),
        (
            """
            #include "main_stm.h"
            int main(void) {
              stm_input_t in;
              stm_output_t out;
              stm_step(&in, &out);
              return 0;
            }
            """,
            "stm_step",
        ),
        (
            """
            #include "stm_controller.h"
            int main(void) {
              stm_platform_hooks_t hooks;
              (void)hooks;
              return 0;
            }
            """,
            "stm_platform_hooks_t",
        ),
        (
            """
            #include "stm_controller.h"
            int main(void) {
              stm_output_t out;
              out.calc_time_ms = 0.0;
              return 0;
            }
            """,
            "calc_time_ms",
        ),
    ],
)
def test_removed_stm_timebase_and_fixed_step_symbols_no_longer_compile(
    tmp_path: Path, source: str, missing_name: str
):
    probe_c = tmp_path / "probe.c"
    probe_bin = tmp_path / "probe"
    probe_c.write_text(textwrap.dedent(source), encoding="utf-8")
    cmd = [
        "gcc",
        "-o",
        str(probe_bin),
        str(probe_c),
        "-I",
        str(PROJECT_ROOT / "stm32_code"),
        "-I",
        str(PROJECT_ROOT / "c_interface"),
        "-O2",
        "-Wall",
        "-Werror",
        "-lm",
    ]
    completed = subprocess.run(cmd, cwd=PROJECT_ROOT, text=True, capture_output=True)
    assert completed.returncode != 0
    assert missing_name in completed.stderr


@pytest.mark.parametrize(
    ("source", "missing_name"),
    [
        (
            """
            #include "control_logic.h"
            int main(void) {
              double q[7] = {0};
              double qd[7] = {0};
              double pos[3] = {0};
              double quat[4] = {1, 0, 0, 0};
              double tau[7];
              control_step_v2(pos, quat, q, qd, tau);
              return 0;
            }
            """,
            "control_step_v2",
        ),
        (
            """
            #include "control_logic.h"
            int main(void) {
              double q[7] = {0};
              double pos[3];
              double quat[4];
              control_get_fk_with_offset(q, pos, quat);
              return 0;
            }
            """,
            "control_get_fk_with_offset",
        ),
        (
            """
            #include "control_logic.h"
            int main(void) {
              double q[7] = {0};
              double qd[7] = {0};
              return control_check_safety(q, qd);
            }
            """,
            "control_check_safety",
        ),
        (
            """
            #include "control_logic.h"
            int main(void) {
              double pos[2][3] = {{0}};
              double quat[2][4] = {{0}};
              double q[14] = {0};
              double qd[14] = {0};
              double tau[14];
              control_step_v2_dual(pos, quat, q, qd, tau);
              return 0;
            }
            """,
            "control_step_v2_dual",
        ),
        (
            """
            #include "control_logic.h"
            int main(void) {
              double pos[3] = {0};
              double quat[4] = {1, 0, 0, 0};
              double q[7] = {0};
              double qd[7] = {0};
              double tau[7];
              control_step_v2_arm(ARM_LEFT, pos, quat, q, qd, tau);
              return 0;
            }
            """,
            "control_step_v2_arm",
        ),
        (
            """
            #include "control_logic.h"
            int main(void) {
              double q[7] = {0};
              double pos[3];
              double quat[4];
              control_get_fk_with_offset_arm(ARM_LEFT, q, pos, quat);
              return 0;
            }
            """,
            "control_get_fk_with_offset_arm",
        ),
        (
            """
            #include "control_logic.h"
            int main(void) {
              double q[7] = {0};
              double g[7];
              control_calc_gravity_compensation_arm(ARM_LEFT, q, g);
              return 0;
            }
            """,
            "control_calc_gravity_compensation_arm",
        ),
        (
            """
            #include "control_logic.h"
            int main(void) {
              double q[7] = {0};
              double qd[7] = {0};
              double c[7];
              control_calc_coriolis_compensation_arm(ARM_LEFT, q, qd, c);
              return 0;
            }
            """,
            "control_calc_coriolis_compensation_arm",
        ),
        (
            """
            #include "control_logic.h"
            int main(void) {
              double gravity[3];
              control_get_body_gravity(gravity);
              return 0;
            }
            """,
            "control_get_body_gravity",
        ),
        (
            """
            #include "model_lib.h"
            int main(void) {
              RBDLModel model;
              build_am_d02_model(&model);
              return 0;
            }
            """,
            "build_am_d02_model",
        ),
        (
            """
            #include "kinematics_lib.h"
            int main(void) {
              KinematicsSolver solver;
              (void)solver;
              return 0;
            }
            """,
            "KinematicsSolver",
        ),
    ],
)
def test_removed_left_arm_and_legacy_ik_symbols_no_longer_compile(
    tmp_path: Path, source: str, missing_name: str
):
    probe_c = tmp_path / "probe.c"
    probe_bin = tmp_path / "probe"
    probe_c.write_text(textwrap.dedent(source), encoding="utf-8")
    cmd = [
        "gcc",
        "-o",
        str(probe_bin),
        str(probe_c),
        "-I",
        str(PROJECT_ROOT / "stm32_code"),
        "-O2",
        "-Wall",
        "-Werror",
        "-lm",
    ]
    completed = subprocess.run(cmd, cwd=PROJECT_ROOT, text=True, capture_output=True)
    assert completed.returncode != 0
    assert missing_name in completed.stderr


def test_uart_protocol_removed_from_portable_stm32_core(tmp_path: Path):
    source = textwrap.dedent(
        """
        #include "uart_protocol.h"
        int main(void) { return 0; }
        """
    )
    probe_c = tmp_path / "probe.c"
    probe_bin = tmp_path / "probe"
    probe_c.write_text(source, encoding="utf-8")
    cmd = [
        "gcc",
        "-o",
        str(probe_bin),
        str(probe_c),
        "-I",
        str(PROJECT_ROOT / "stm32_code"),
        "-O2",
        "-Wall",
        "-Werror",
        "-lm",
    ]
    completed = subprocess.run(cmd, cwd=PROJECT_ROOT, text=True, capture_output=True)
    assert completed.returncode != 0
    assert "uart_protocol.h" in completed.stderr


def test_h7_clock_sim_samples_1mhz_elapsed_time_with_clamp(tmp_path: Path):
    source = textwrap.dedent(
        """
        #include "h7_clock_sim.h"
        #include <stdint.h>
        #include <stdio.h>

        static uint64_t fake_times_us[] = {1000000ULL, 1000500ULL, 1002200ULL, 1045000ULL};
        static int fake_index = 0;

        static uint64_t fake_now_us(void *user_ctx) {
          (void)user_ctx;
          return fake_times_us[fake_index++];
        }

        int main(void) {
          h7_clock_sim_t clock;
          h7_clock_sim_init(&clock);
          h7_clock_sim_set_now_fn(&clock, fake_now_us, 0);
          h7_clock_sim_set_max_elapsed(&clock, 0.02);
          printf("%llu\\n", (unsigned long long)h7_clock_sim_elapsed_us(&clock));
          printf("%llu\\n", (unsigned long long)h7_clock_sim_elapsed_us(&clock));
          printf("%llu\\n", (unsigned long long)h7_clock_sim_elapsed_us(&clock));
          printf("%llu\\n", (unsigned long long)h7_clock_sim_elapsed_us(&clock));
          return 0;
        }
        """
    )
    probe = _compile_host_probe(
        tmp_path,
        source,
        ["c_interface/h7_clock_sim/h7_clock_sim.c"],
    )
    values = [
        int(line)
        for line in subprocess.check_output([str(probe)], text=True).splitlines()
    ]

    assert values == [0, 500, 1700, 20000]


def test_c_sim_main_is_one_receive_one_send_without_scheduler_ticks():
    main_source = (PROJECT_ROOT / "c_interface" / "main.c").read_text(encoding="utf-8")

    assert "wait_for_h7_due_ticks" not in main_source
    assert "h7_clock_sim_due_ticks" not in main_source
    assert "stm_step_elapsed" in main_source
    assert "for (int tick" not in main_source


def test_c_sim_main_keeps_running_during_safety_recovery():
    main_source = (PROJECT_ROOT / "c_interface" / "main.c").read_text(encoding="utf-8")

    assert "EMERGENCY STOP" not in main_source
    assert "exit(1)" not in main_source
    assert "STM_STATUS_SAFETY_LATCHED" in main_source
    assert "STM_STATUS_WAITING_ZERO" in main_source
    assert main_source.index("if (stm_out.status != last_status)") < main_source.index(
        "sim_apply_torque(stm_out.tau)"
    )


def test_stm32_code_has_no_host_stdio_or_clock_dependency():
    forbidden = [
        "<stdio.h>",
        "fprintf",
        "stderr",
        "<time.h>",
        "clock_gettime",
        "malloc",
        "free(",
    ]
    for path in (PROJECT_ROOT / "stm32_code").glob("*.[ch]"):
        text = path.read_text(encoding="utf-8")
        for token in forbidden:
            assert token not in text, f"{token} leaked into {path.relative_to(PROJECT_ROOT)}"


def test_stm32h7_porting_doc_describes_elapsed_time_contract():
    doc_path = PROJECT_ROOT / "stm32_code" / "STM32H7_PORTING.md"
    text = doc_path.read_text(encoding="utf-8")

    assert "elapsed_s" in text
    assert "timer_frequency_hz" in text
    assert "CONTROL_DT" in text
    assert "h7_clock_sim" in text
    assert "STM_LOG_ERROR" in text


def test_cartesian_pd_gains_are_endpoint_tracking_tuned(tmp_path: Path):
    source = textwrap.dedent(
        """
        #include "config.h"
        #include <stdio.h>

        int main(void) {
          printf("%.1f %.1f %.1f %.1f %.1f %.1f %.1f %.1f %.1f %.1f %.1f %.1f\\n",
                 KP_CART_X, KP_CART_Y, KP_CART_Z,
                 KP_CART_ROLL, KP_CART_PITCH, KP_CART_YAW,
                 KD_CART_X, KD_CART_Y, KD_CART_Z,
                 KD_CART_ROLL, KD_CART_PITCH, KD_CART_YAW);
          return 0;
        }
        """
    )
    probe = _compile_c_probe(tmp_path, source, include_stm_controller=False)
    values = [float(value) for value in subprocess.check_output([str(probe)], text=True).split()]

    assert values == pytest.approx(
        [260.0, 260.0, 260.0, 12.0, 12.0, 12.0, 70.0, 70.0, 70.0, 4.0, 4.0, 4.0]
    )


def test_stm_controller_uses_linear_path_planner_with_acceleration(tmp_path: Path):
    source = textwrap.dedent(
        """
        #include "stm_controller.c"
        #include <math.h>
        #include <stdio.h>
        #include <string.h>

        static void fill_input(stm_input_t *in, double target_x, double target_y) {
          memset(in, 0, sizeof(*in));
          in->active_arm_mask = STM_ARM_MASK_LEFT;
          in->target_pos[ARM_LEFT][0] = target_x;
          in->target_pos[ARM_LEFT][1] = target_y;
          in->target_pos[ARM_LEFT][2] = 0.0;
          in->target_quat[ARM_LEFT][0] = 1.0;
          in->target_quat[ARM_LEFT][1] = 0.0;
          in->target_quat[ARM_LEFT][2] = 0.0;
          in->target_quat[ARM_LEFT][3] = 0.0;
        }

        static double dist3(const double a[3], const double b[3]) {
          double dx = a[0] - b[0];
          double dy = a[1] - b[1];
          double dz = a[2] - b[2];
          return sqrt(dx * dx + dy * dy + dz * dz);
        }

        int main(void) {
          stm_input_t in;
          stm_output_t out;
          double first_step = 0.5 * TRAJ_PLAN_ACCEL * CONTROL_DT * CONTROL_DT;
          double second_step = 0.5 * TRAJ_PLAN_ACCEL * (2.0 * CONTROL_DT) * (2.0 * CONTROL_DT);
          double first_lookahead = CONTROL_PATH_LOOKAHEAD_M * CONTROL_DT / CONTROL_PATH_LOOKAHEAD_RAMP_S;
          double second_lookahead = CONTROL_PATH_LOOKAHEAD_M * (2.0 * CONTROL_DT) / CONTROL_PATH_LOOKAHEAD_RAMP_S;
          double start_pos[3];
          double previous_ref[3];
          double changed_start[3];

          stm_controller_init();
          stm_controller_reset();
          fill_input(&in, 1.0, 0.0);
          stm_controller_step_elapsed(&in, &out, CONTROL_DT);
          memcpy(start_pos, g_controller.path_planner[ARM_LEFT].start_pos, sizeof(start_pos));
          printf("%.12f %.12f %.12f %.12f %.12f %.12f\\n",
                 dist3(g_controller.ref_pos[ARM_LEFT], start_pos),
                 first_step + first_lookahead,
                 g_controller.path_progress_t[ARM_LEFT],
                 CONTROL_DT,
                 g_controller.path_planner[ARM_LEFT].v_max,
                 TRAJ_PLAN_SPEED);

          stm_controller_step_elapsed(&in, &out, CONTROL_DT);
          memcpy(previous_ref, g_controller.ref_pos[ARM_LEFT], sizeof(previous_ref));
          printf("%.12f %.12f %.12f %.12f %.12f %.12f\\n",
                 dist3(g_controller.ref_pos[ARM_LEFT], start_pos),
                 second_step + second_lookahead,
                 g_controller.path_progress_t[ARM_LEFT],
                 2.0 * CONTROL_DT,
                 g_controller.path_planner[ARM_LEFT].a,
                 TRAJ_PLAN_ACCEL);

          fill_input(&in, 1.0, 1.0);
          stm_controller_step_elapsed(&in, &out, CONTROL_DT);
          memcpy(changed_start, g_controller.path_planner[ARM_LEFT].start_pos,
                 sizeof(changed_start));
          printf("%.12f %.12f %.12f %.12f %.12f %.12f\\n",
                 dist3(g_controller.ref_pos[ARM_LEFT], changed_start),
                 first_step + second_lookahead + first_lookahead,
                 g_controller.path_progress_t[ARM_LEFT],
                 CONTROL_DT,
                 dist3(changed_start, previous_ref),
                 0.0);
          return 0;
        }
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

    np.testing.assert_allclose(rows[0, 0], rows[0, 1], atol=1e-9)
    np.testing.assert_allclose(rows[0, 2], rows[0, 3], atol=1e-12)
    assert rows[0, 4] <= rows[0, 5] + 1e-12
    np.testing.assert_allclose(rows[1, 0], rows[1, 1], atol=1e-9)
    assert rows[1, 0] > rows[0, 0]
    np.testing.assert_allclose(rows[1, 2], rows[1, 3], atol=1e-12)
    np.testing.assert_allclose(rows[1, 4], rows[1, 5], atol=1e-12)
    np.testing.assert_allclose(rows[2, 0], rows[2, 1], atol=1e-9)
    np.testing.assert_allclose(rows[2, 2], rows[2, 3], atol=1e-12)
    np.testing.assert_allclose(rows[2, 4], 0.0, atol=1e-12)


def test_stm_controller_path_gate_slows_reference_when_tcp_lags(tmp_path: Path):
    source = textwrap.dedent(
        """
        #include "stm_controller.c"
        #include <math.h>
        #include <stdio.h>
        #include <string.h>

        static double dist3(const double a[3], const double b[3]) {
          double dx = a[0] - b[0];
          double dy = a[1] - b[1];
          double dz = a[2] - b[2];
          return sqrt(dx * dx + dy * dy + dz * dz);
        }

        static void fill_input(stm_input_t *in, double start_pos[3]) {
          control_arm_kinematics_t kin;
          memset(in, 0, sizeof(*in));
          in->active_arm_mask = STM_ARM_MASK_LEFT;
          in->q[3] = 1.0;
          control_get_arm_kinematics_with_offset(ARM_LEFT, in->q, &kin);
          memcpy(start_pos, kin.pos, sizeof(double) * 3);
          memcpy(in->target_pos[ARM_LEFT], kin.pos, sizeof(double) * 3);
          memcpy(in->target_quat[ARM_LEFT], kin.quat_wxyz, sizeof(double) * 4);
          in->target_pos[ARM_LEFT][0] += 0.50;
        }

        int main(void) {
          stm_input_t in;
          stm_output_t out;
          double start_pos[3];

          stm_controller_init();
          stm_controller_reset();
          fill_input(&in, start_pos);

          for (int i = 0; i < 2000; ++i) {
            stm_controller_step_elapsed(&in, &out, CONTROL_DT);
          }

          printf("%.12f %.12f %.12f %.12f %.12f\\n",
                 dist3(g_controller.ref_pos[ARM_LEFT], start_pos),
                 g_controller.path_progress_t[ARM_LEFT],
                 g_controller.path_gate[ARM_LEFT],
                 g_controller.path_lookahead_m[ARM_LEFT],
                 CONTROL_PATH_GATE_STOP_ERROR_M);
          return 0;
        }
        """
    )
    probe = _compile_c_probe(tmp_path, source, include_stm_controller=False)
    ref_error, progress_t, gate, lookahead, stop_error = [
        float(value) for value in subprocess.check_output([str(probe)], text=True).split()
    ]

    assert ref_error < 0.05
    assert ref_error <= stop_error + lookahead + 0.01
    assert progress_t < 0.5
    assert gate < 0.1
    assert lookahead == pytest.approx(0.008)


def test_stm_controller_path_lookahead_ramps_smoothly(tmp_path: Path):
    source = textwrap.dedent(
        """
        #include "stm_controller.c"
        #include <math.h>
        #include <stdio.h>
        #include <string.h>

        static double dist3(const double a[3], const double b[3]) {
          double dx = a[0] - b[0];
          double dy = a[1] - b[1];
          double dz = a[2] - b[2];
          return sqrt(dx * dx + dy * dy + dz * dz);
        }

        static void fill_input(stm_input_t *in, double start_pos[3]) {
          control_arm_kinematics_t kin;
          memset(in, 0, sizeof(*in));
          in->active_arm_mask = STM_ARM_MASK_LEFT;
          in->q[3] = 1.0;
          control_get_arm_kinematics_with_offset(ARM_LEFT, in->q, &kin);
          memcpy(start_pos, kin.pos, sizeof(double) * 3);
          memcpy(in->target_pos[ARM_LEFT], kin.pos, sizeof(double) * 3);
          memcpy(in->target_quat[ARM_LEFT], kin.quat_wxyz, sizeof(double) * 4);
          in->target_pos[ARM_LEFT][0] += 0.50;
        }

        int main(void) {
          stm_input_t in;
          stm_output_t out;
          double start_pos[3];

          stm_controller_init();
          stm_controller_reset();
          fill_input(&in, start_pos);

          stm_controller_step_elapsed(&in, &out, CONTROL_DT);
          printf("%.12f %.12f %.12f\\n",
                 g_controller.path_lookahead_m[ARM_LEFT],
                 CONTROL_PATH_LOOKAHEAD_M * CONTROL_DT / CONTROL_PATH_LOOKAHEAD_RAMP_S,
                 dist3(g_controller.ref_pos[ARM_LEFT], start_pos));

          for (int i = 1; i < 300; ++i) {
            stm_controller_step_elapsed(&in, &out, CONTROL_DT);
          }
          printf("%.12f %.12f\\n",
                 g_controller.path_lookahead_m[ARM_LEFT],
                 CONTROL_PATH_LOOKAHEAD_M);
          return 0;
        }
        """
    )
    probe = _compile_c_probe(tmp_path, source, include_stm_controller=False)
    rows = [
        [float(value) for value in line.split()]
        for line in subprocess.check_output([str(probe)], text=True).splitlines()
    ]

    np.testing.assert_allclose(rows[0][0], rows[0][1], atol=1e-9)
    assert 0.0 < rows[0][0] < 0.008
    assert rows[0][2] < 0.001
    np.testing.assert_allclose(rows[1][0], rows[1][1], atol=1e-9)


def test_stm_controller_path_gate_changes_smoothly(tmp_path: Path):
    source = textwrap.dedent(
        """
        #include "stm_controller.c"
        #include <math.h>
        #include <stdio.h>
        #include <string.h>

        static void fill_input(stm_input_t *in, double current_pos[3]) {
          control_arm_kinematics_t kin;
          memset(in, 0, sizeof(*in));
          in->active_arm_mask = STM_ARM_MASK_LEFT;
          in->q[3] = 1.0;
          control_get_arm_kinematics_with_offset(ARM_LEFT, in->q, &kin);
          memcpy(current_pos, kin.pos, sizeof(double) * 3);
          memcpy(in->target_pos[ARM_LEFT], kin.pos, sizeof(double) * 3);
          memcpy(in->target_quat[ARM_LEFT], kin.quat_wxyz, sizeof(double) * 4);
          in->target_pos[ARM_LEFT][0] += 0.50;
        }

        int main(void) {
          stm_input_t in;
          stm_output_t out;
          double current_pos[3];

          stm_controller_init();
          stm_controller_reset();
          fill_input(&in, current_pos);
          stm_controller_step_elapsed(&in, &out, CONTROL_DT);

          memcpy(g_controller.ref_pos[ARM_LEFT], current_pos, sizeof(current_pos));
          g_controller.ref_pos[ARM_LEFT][0] += CONTROL_PATH_GATE_STOP_ERROR_M + 0.01;
          g_controller.path_gate[ARM_LEFT] = 1.0;
          stm_controller_step_elapsed(&in, &out, CONTROL_DT);
          printf("%.12f %.12f\\n",
                 g_controller.path_gate[ARM_LEFT],
                 1.0 - CONTROL_DT / CONTROL_PATH_GATE_FALL_TIME_S);

          memcpy(g_controller.ref_pos[ARM_LEFT], current_pos, sizeof(current_pos));
          g_controller.path_gate[ARM_LEFT] = 0.0;
          stm_controller_step_elapsed(&in, &out, CONTROL_DT);
          printf("%.12f %.12f\\n",
                 g_controller.path_gate[ARM_LEFT],
                 CONTROL_DT / CONTROL_PATH_GATE_RISE_TIME_S);
          return 0;
        }
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

    np.testing.assert_allclose(rows[:, 0], rows[:, 1], atol=1e-9)
    assert 0.0 < rows[0, 0] < 1.0
    assert 0.0 < rows[1, 0] < 1.0


def test_trajectory_speed_is_real_safe_tuned(tmp_path: Path):
    source = textwrap.dedent(
        """
        #include "config.h"
        #include <stdio.h>

        int main(void) {
          printf("%.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f\\n",
                 TRAJ_PLAN_SPEED,
                 TRAJ_PLAN_ACCEL,
                 JOINT_VEL_LIMIT,
                 CONTROL_JOINT_LIMIT_INSET_RATIO,
                 CONTROL_PATH_GATE_FULL_ERROR_M,
                 CONTROL_PATH_GATE_STOP_ERROR_M,
                 CONTROL_PATH_GATE_RISE_TIME_S,
                 CONTROL_PATH_GATE_FALL_TIME_S,
                 CONTROL_PATH_LOOKAHEAD_M,
                 CONTROL_PATH_LOOKAHEAD_RAMP_S,
                 CONTROL_TARGET_REPLAN_POS_EPS_M,
                 CONTROL_TARGET_REPLAN_ORI_EPS_RAD);
          return 0;
        }
        """
    )
    probe = _compile_c_probe(tmp_path, source, include_stm_controller=False)
    values = [float(value) for value in subprocess.check_output([str(probe)], text=True).split()]

    assert values == pytest.approx(
        [0.5, 0.2, 4.0, 0.01, 0.005, 0.02, 0.08, 0.03, 0.008, 0.30, 0.001, 0.002]
    )


def test_python_config_mirrors_c_path_gate_defaults():
    from config import Config

    assert Config.CONTROL_PATH_GATE_FULL_ERROR_M == pytest.approx(0.005)
    assert Config.CONTROL_PATH_GATE_STOP_ERROR_M == pytest.approx(0.020)
    assert Config.CONTROL_PATH_GATE_RISE_TIME_S == pytest.approx(0.080)
    assert Config.CONTROL_PATH_GATE_FALL_TIME_S == pytest.approx(0.030)
    assert Config.CONTROL_PATH_LOOKAHEAD_M == pytest.approx(0.008)
    assert Config.CONTROL_PATH_LOOKAHEAD_RAMP_S == pytest.approx(0.30)
    assert Config.CONTROL_TARGET_REPLAN_POS_EPS_M == pytest.approx(0.001)
    assert Config.CONTROL_TARGET_REPLAN_ORI_EPS_RAD == pytest.approx(0.002)


def test_stm_controller_step_elapsed_hot_path_benchmark_probe(tmp_path: Path):
    source = textwrap.dedent(
        """
        #include "control_logic.h"
        #include "stm_controller.h"
        #include <stdio.h>
        #include <string.h>
        #include <time.h>

        static void fill_input(stm_input_t *in) {
          memset(in, 0, sizeof(*in));
          in->q[0] = 0.10;
          in->q[1] = -0.20;
          in->q[2] = 0.15;
          in->q[3] = 1.10;
          in->q[4] = -0.08;
          in->q[5] = 0.12;
          in->q[6] = -0.05;
          in->q[7] = -0.12;
          in->q[8] = 0.18;
          in->q[9] = -0.14;
          in->q[10] = 1.12;
          in->q[11] = 0.10;
          in->q[12] = -0.11;
          in->q[13] = 0.04;
          for (int arm = 0; arm < NUM_ARMS; ++arm) {
            int offset = arm * ARM_JOINTS;
            control_arm_kinematics_t kin;
            control_get_arm_kinematics_with_offset(arm, in->q + offset, &kin);
            for (int i = 0; i < 3; ++i) {
              in->target_pos[arm][i] = kin.pos[i];
            }
            for (int i = 0; i < 4; ++i) {
              in->target_quat[arm][i] = kin.quat_wxyz[i];
            }
            in->target_pos[arm][0] += arm == ARM_LEFT ? 0.02 : -0.02;
            in->target_pos[arm][2] += 0.01;
          }
        }

        int main(void) {
          enum { N = 200 };
          stm_input_t in;
          stm_output_t out;
          struct timespec t0;
          struct timespec t1;
          double elapsed_us;

          stm_controller_init();
          stm_controller_reset();
          fill_input(&in);
          clock_gettime(CLOCK_MONOTONIC, &t0);
          for (int i = 0; i < N; ++i) {
            stm_controller_step_elapsed(&in, &out, 0.001);
          }
          clock_gettime(CLOCK_MONOTONIC, &t1);
          elapsed_us = (double)(t1.tv_sec - t0.tv_sec) * 1000000.0 +
                       (double)(t1.tv_nsec - t0.tv_nsec) / 1000.0;
          printf("%.6f %d %.12f\\n", elapsed_us / (double)N, out.status, out.tau[0]);
          return 0;
        }
        """
    )
    probe = _compile_c_probe(tmp_path, source)
    avg_us, status, tau0 = subprocess.check_output([str(probe)], text=True).split()

    assert float(avg_us) > 0.0
    assert int(status) == 0
    assert np.isfinite(float(tau0))
