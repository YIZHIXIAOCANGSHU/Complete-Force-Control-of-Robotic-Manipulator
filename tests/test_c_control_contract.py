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
              double pos[3];
              double quat[4];
              control_get_fk_with_offset_arm(arm, q[i], pos, quat);
              printf("%d %.12f %.12f %.12f %.12f %.12f %.12f %.12f\\n",
                     arm, pos[0], pos[1], pos[2], quat[0], quat[1], quat[2], quat[3]);
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


def test_control_step_zero_pose_error_outputs_bias_compensation_only(tmp_path: Path):
    source = textwrap.dedent(
        """
        #include "control_logic.h"
        #include <math.h>
        #include <stdio.h>

        int main(void) {
          double q[7] = {0.0, 0.0, 0.0, 1.5707963267948966, 0.0, 0.0, 0.0};
          double qd[7] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
          double target_pos[3];
          double target_quat[4];
          double gravity[7];
          double coriolis[7];
          double tau[7];
          double max_extra = 0.0;

          control_init();
          control_get_fk_with_offset(q, target_pos, target_quat);
          control_calc_gravity_compensation(q, gravity);
          control_calc_coriolis_compensation(q, qd, coriolis);
          control_step_v2(target_pos, target_quat, q, qd, tau);

          for (int i = 0; i < 7; ++i) {
            double expected = gravity[i] + coriolis[i];
            double extra = fabs(tau[i] - expected);
            if (extra > max_extra) {
              max_extra = extra;
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


def test_control_step_nonzero_pose_error_adds_cartesian_correction(tmp_path: Path):
    source = textwrap.dedent(
        """
        #include "control_logic.h"
        #include <math.h>
        #include <stdio.h>

        int main(void) {
          double q[7] = {0.0, 0.0, 0.0, 1.5707963267948966, 0.0, 0.0, 0.0};
          double qd[7] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
          double target_pos[3];
          double target_quat[4];
          double gravity[7];
          double coriolis[7];
          double tau[7];
          double max_extra = 0.0;

          control_init();
          control_get_fk_with_offset(q, target_pos, target_quat);
          target_pos[0] += 0.02;
          target_pos[2] += 0.01;
          control_calc_gravity_compensation(q, gravity);
          control_calc_coriolis_compensation(q, qd, coriolis);
          control_step_v2(target_pos, target_quat, q, qd, tau);

          for (int i = 0; i < 7; ++i) {
            double expected = gravity[i] + coriolis[i];
            double extra = fabs(tau[i] - expected);
            if (extra > max_extra) {
              max_extra = extra;
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


def test_c_body_gravity_uses_three_body_joint_angles(tmp_path: Path):
    source = textwrap.dedent(
        """
        #include "control_logic.h"
        #include <stdio.h>

        int main(void) {
          double zero_q[3] = {0.0, 0.0, 0.0};
          double body_q[3] = {0.3, -0.4, 0.2};
          double gravity[3];

          control_init();
          control_update_body_gravity(zero_q);
          control_get_body_gravity(gravity);
          printf("%.12f %.12f %.12f\\n", gravity[0], gravity[1], gravity[2]);

          control_update_body_gravity(body_q);
          control_get_body_gravity(gravity);
          printf("%.12f %.12f %.12f\\n", gravity[0], gravity[1], gravity[2]);
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

    np.testing.assert_allclose(rows[0], [0.0, 0.0, -9.81], atol=1e-9)
    assert not np.allclose(rows[1], [0.0, 0.0, -9.81])
    assert np.linalg.norm(rows[1]) == pytest.approx(9.81, abs=1e-9)


def test_c_safety_uses_urdf_right_arm_position_limits_and_per_joint_velocity(tmp_path: Path):
    source = textwrap.dedent(
        """
        #include "control_logic.h"
        #include <stdio.h>

        static void print_status(const char *label, int side, double q[7], double qd[7]) {
          printf("%s %d\\n", label, control_check_safety_arm(side, q, qd));
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
          print_status("right_center", ARM_RIGHT, q, qd);
          for (int i = 0; i < 7; ++i) {
            q[i] = right_min[i] - 0.011;
            print_status("right_low", ARM_RIGHT, q, qd);
            q[i] = right_max[i] + 0.011;
            print_status("right_high", ARM_RIGHT, q, qd);
            q[i] = 0.0;
          }

          q[0] = left_min[0] - 0.011;
          print_status("left_j1_low", ARM_LEFT, q, qd);
          q[0] = 0.0;
          q[6] = left_max[6] + 0.011;
          print_status("left_j7_high", ARM_LEFT, q, qd);
          q[6] = 0.0;

          qd[0] = right_vel[0] - 0.001;
          print_status("right_vel_ok", ARM_RIGHT, q, qd);
          qd[0] = right_vel[0] + 0.001;
          print_status("right_vel_high", ARM_RIGHT, q, qd);
          qd[0] = 0.0;
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
                                          target_quat, CONTROL_DT, ref_pos, ref_quat);
          printf("%.9f %.9f %.9f\\n", ref_pos[0], ref_pos[1], ref_pos[2]);

          current_pos[0] = -5.0;
          current_pos[1] = -5.0;
          current_pos[2] = -5.0;
          stm_controller_update_reference(current_pos, current_quat, target_b,
                                          target_quat, CONTROL_DT, ref_pos, ref_quat);
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

    first_step = 0.75 * 0.001
    np.testing.assert_allclose(rows[0], [first_step, 0.0, 0.0], atol=1e-9)

    direction = np.array([1.0, 1.0, 0.0]) - rows[0]
    expected_second = rows[0] + first_step * direction / np.linalg.norm(direction)
    np.testing.assert_allclose(rows[1], expected_second, atol=1e-9)
    assert np.linalg.norm(rows[1] - rows[0]) == pytest.approx(first_step)
    assert not np.allclose(rows[1], [-5.0, -5.0, -5.0])
