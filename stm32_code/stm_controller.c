#include "stm_controller.h"

#include "control_logic.h"
#include "trajectory_lib.h"
#include <math.h>
#include <stdio.h>
#include <string.h>

typedef struct {
  double path_s;
  int step_count;
  int initialized;
  int path_valid;
  int have_last_q_ref;
  stm_platform_hooks_t hooks;
  LinearPathPlanner path;
  double latched_target_pos[3];
  double latched_target_quat[4];
  double last_q_ref[NUM_JOINTS];
} StmControllerState;

static StmControllerState g_controller = {0};

static const double TORQUE_LIMIT_CHECK[NUM_JOINTS] = {
    JOINT_TORQUE_LIMIT_1, JOINT_TORQUE_LIMIT_2, JOINT_TORQUE_LIMIT_3,
    JOINT_TORQUE_LIMIT_4, JOINT_TORQUE_LIMIT_5, JOINT_TORQUE_LIMIT_6,
    JOINT_TORQUE_LIMIT_7};

static const double JOINT_KP_DEFAULT[NUM_JOINTS] = {
    KP_JOINT_1, KP_JOINT_2, KP_JOINT_3, KP_JOINT_4,
    KP_JOINT_5, KP_JOINT_6, KP_JOINT_7};

static const double JOINT_KD_DEFAULT[NUM_JOINTS] = {
    KD_JOINT_1, KD_JOINT_2, KD_JOINT_3, KD_JOINT_4,
    KD_JOINT_5, KD_JOINT_6, KD_JOINT_7};

static KinematicsSolver g_stm_ik_solver;
static RBDLModel g_stm_dynamics_model;

static double stm_controller_now_ms(void) {
  if (g_controller.hooks.now_ms == NULL) {
    return 0.0;
  }
  return g_controller.hooks.now_ms(g_controller.hooks.user_ctx);
}

static int stm_controller_is_finite_vec(const double *values, int count) {
  for (int i = 0; i < count; ++i) {
    if (!isfinite(values[i])) {
      return 0;
    }
  }
  return 1;
}

static void stm_controller_sanitize_target_pose(const stm_input_t *in,
                                                const double fallback_pos[3],
                                                const double fallback_quat[4],
                                                double target_pos[3],
                                                double target_quat[4]) {
  double quat_norm_sq = 0.0;
  int pos_valid = stm_controller_is_finite_vec(in->target_pos, 3);
  int quat_valid = stm_controller_is_finite_vec(in->target_quat, 4);

  if (pos_valid) {
    memcpy(target_pos, in->target_pos, sizeof(double) * 3);
  } else if (g_controller.path_valid) {
    memcpy(target_pos, g_controller.latched_target_pos, sizeof(double) * 3);
  } else {
    memcpy(target_pos, fallback_pos, sizeof(double) * 3);
  }

  if (quat_valid) {
    for (int i = 0; i < 4; ++i) {
      quat_norm_sq += in->target_quat[i] * in->target_quat[i];
    }
  }

  if (quat_norm_sq > 1e-12 && isfinite(quat_norm_sq)) {
    double quat_norm_inv = 1.0 / sqrt(quat_norm_sq);
    for (int i = 0; i < 4; ++i) {
      target_quat[i] = in->target_quat[i] * quat_norm_inv;
    }
  } else if (g_controller.path_valid) {
    memcpy(target_quat, g_controller.latched_target_quat, sizeof(double) * 4);
  } else {
    memcpy(target_quat, fallback_quat, sizeof(double) * 4);
  }
}

static int stm_controller_target_changed(const double target_pos[3],
                                         const double target_quat[4]) {
  const double pos_thresh = 1e-4;
  const double quat_dot_thresh = 0.99999;
  double pos_delta_sq = 0.0;
  double quat_dot = 0.0;

  if (!g_controller.path_valid) {
    return 1;
  }

  for (int i = 0; i < 3; ++i) {
    double delta = target_pos[i] - g_controller.latched_target_pos[i];
    pos_delta_sq += delta * delta;
  }
  if (pos_delta_sq > pos_thresh * pos_thresh) {
    return 1;
  }

  for (int i = 0; i < 4; ++i) {
    quat_dot += target_quat[i] * g_controller.latched_target_quat[i];
  }
  if (fabs(quat_dot) < quat_dot_thresh) {
    return 1;
  }

  return 0;
}

static void stm_controller_start_path(const double current_pos[3],
                                      const double current_quat[4],
                                      const double target_pos[3],
                                      const double target_quat[4]) {
  linear_path_init(&g_controller.path, current_pos, current_quat, target_pos,
                   target_quat, TRAJ_PLAN_SPEED, TRAJ_PLAN_ACCEL);
  memcpy(g_controller.latched_target_pos, target_pos, sizeof(double) * 3);
  memcpy(g_controller.latched_target_quat, target_quat, sizeof(double) * 4);
  g_controller.path_s = 0.0;
  g_controller.have_last_q_ref = 0;
  g_controller.path_valid = 1;
}

static void stm_controller_prepare_output(stm_output_t *out) {
  memset(out, 0, sizeof(*out));
  out->status = -1;
}

static int stm_controller_check_joint_safety(const double q[NUM_JOINTS],
                                             const double qd[NUM_JOINTS],
                                             const double tau[NUM_JOINTS]) {
  if (!stm_controller_is_finite_vec(q, NUM_JOINTS) ||
      !stm_controller_is_finite_vec(qd, NUM_JOINTS)) {
    fprintf(stderr, "[SAFETY] Non-finite joint state detected.\n");
    return -1;
  }

  int status = control_check_safety(q, qd);
  if (status < 0) {
    if (status == -1) {
      fprintf(stderr, "[SAFETY] Joint position limit violated!\n");
    } else if (status == -2) {
      fprintf(stderr, "[SAFETY] Joint velocity limit violated!\n");
    }
    return -1;
  }

  for (int i = 0; i < NUM_JOINTS; ++i) {
    if (!isfinite(tau[i])) {
      fprintf(stderr, "[SAFETY] J%d torque is non-finite.\n", i + 1);
      return -1;
    }
    if (fabs(tau[i]) > TORQUE_LIMIT_CHECK[i]) {
      fprintf(stderr, "[SAFETY] J%d torque error: %.4f\n", i + 1, tau[i]);
      return -1;
    }
  }

  return 0;
}

static void stm_controller_fill_mit_gains(stm_output_t *out) {
  memcpy(out->kp, JOINT_KP_DEFAULT, sizeof(out->kp));
  memcpy(out->kd, JOINT_KD_DEFAULT, sizeof(out->kd));
}

static void stm_controller_fill_zero_mit(const double q[NUM_JOINTS],
                                         stm_output_t *out) {
  memcpy(out->q_ref, q, sizeof(out->q_ref));
  memset(out->qd_ref, 0, sizeof(out->qd_ref));
  memset(out->tau_ff, 0, sizeof(out->tau_ff));
  memset(out->tau, 0, sizeof(out->tau));
  stm_controller_fill_mit_gains(out);
}

static void stm_controller_pose_from_arrays(const double pos[3],
                                            const double quat_wxyz[4],
                                            Pose *pose) {
  double R[9];
  double tcp_offset[3] = {TCP_OFFSET_X, TCP_OFFSET_Y, TCP_OFFSET_Z};
  double tcp_delta[3];
  double qw = quat_wxyz[0], qx = quat_wxyz[1], qy = quat_wxyz[2],
         qz = quat_wxyz[3];

  R[0] = 1.0 - 2.0 * (qy * qy + qz * qz);
  R[1] = 2.0 * (qx * qy - qz * qw);
  R[2] = 2.0 * (qx * qz + qy * qw);
  R[3] = 2.0 * (qx * qy + qz * qw);
  R[4] = 1.0 - 2.0 * (qx * qx + qz * qz);
  R[5] = 2.0 * (qy * qz - qx * qw);
  R[6] = 2.0 * (qx * qz - qy * qw);
  R[7] = 2.0 * (qy * qz + qx * qw);
  R[8] = 1.0 - 2.0 * (qx * qx + qy * qy);
  mat3_mul_vec3(R, tcp_offset, tcp_delta);

  pose->position[0] = pos[0] - tcp_delta[0];
  pose->position[1] = pos[1] - tcp_delta[1];
  pose->position[2] = pos[2] - tcp_delta[2];
  pose->orientation.w = quat_wxyz[0];
  pose->orientation.x = quat_wxyz[1];
  pose->orientation.y = quat_wxyz[2];
  pose->orientation.z = quat_wxyz[3];
}

static void stm_controller_sample_path_at_s(double path_s, double ref_pos[3],
                                            double ref_quat[4]) {
  double ratio = 1.0;

  if (!g_controller.path_valid || g_controller.path.L < 1e-9) {
    memcpy(ref_pos, g_controller.latched_target_pos, sizeof(double) * 3);
    memcpy(ref_quat, g_controller.latched_target_quat, sizeof(double) * 4);
    return;
  }

  if (path_s < 0.0) {
    path_s = 0.0;
  }
  if (path_s > g_controller.path.L) {
    path_s = g_controller.path.L;
  }

  ref_pos[0] = g_controller.path.start_pos[0] +
               path_s * g_controller.path.dir[0];
  ref_pos[1] = g_controller.path.start_pos[1] +
               path_s * g_controller.path.dir[1];
  ref_pos[2] = g_controller.path.start_pos[2] +
               path_s * g_controller.path.dir[2];

  if (g_controller.path.L > 1e-9) {
    ratio = path_s / g_controller.path.L;
  }
  if (ratio > 1.0) {
    ratio = 1.0;
  }
  quat_slerp(g_controller.path.start_quat, g_controller.path.end_quat, ratio,
             ref_quat);
}

static void stm_controller_evaluate_round_path(double ref_pos[3],
                                               double ref_quat[4]) {
  if (!g_controller.path_valid || g_controller.path.L < 1e-9) {
    memcpy(ref_pos, g_controller.latched_target_pos, sizeof(double) * 3);
    memcpy(ref_quat, g_controller.latched_target_quat, sizeof(double) * 4);
    g_controller.path_s = g_controller.path.L;
    return;
  }

  if (g_controller.path_s < g_controller.path.L) {
    g_controller.path_s += TRAJ_PLAN_STEP_M;
    if (g_controller.path_s > g_controller.path.L) {
      g_controller.path_s = g_controller.path.L;
    }
  }

  stm_controller_sample_path_at_s(g_controller.path_s, ref_pos, ref_quat);
}

static int stm_controller_compute_q_ref(const double current_q[NUM_JOINTS],
                                        const double ref_pos[3],
                                        const double ref_quat[4],
                                        double q_ref[NUM_JOINTS]) {
  Pose target_pose;
  double initial_q[NUM_JOINTS];

  stm_controller_pose_from_arrays(ref_pos, ref_quat, &target_pose);
  if (g_controller.have_last_q_ref) {
    memcpy(initial_q, g_controller.last_q_ref, sizeof(initial_q));
  } else {
    memcpy(initial_q, current_q, sizeof(initial_q));
  }

  return kinematics_compute_inverse_pose_dls(&g_stm_ik_solver, &target_pose,
                                             initial_q, q_ref,
                                             IK_MAX_ITERATIONS);
}

static int stm_controller_compute_q_ref_from_initial(
    const double initial_q[NUM_JOINTS], const double ref_pos[3],
    const double ref_quat[4], double q_ref[NUM_JOINTS]) {
  Pose target_pose;

  stm_controller_pose_from_arrays(ref_pos, ref_quat, &target_pose);
  return kinematics_compute_inverse_pose_dls(&g_stm_ik_solver, &target_pose,
                                             initial_q, q_ref,
                                             IK_MAX_ITERATIONS);
}

static void stm_controller_compute_qd_ref(const double q_ref[NUM_JOINTS],
                                          double qd_ref[NUM_JOINTS]) {
  double next_s;
  double ds;
  double dt_segment;
  double next_pos[3];
  double next_quat[4];
  double q_next_ref[NUM_JOINTS];

  memset(qd_ref, 0, sizeof(double) * NUM_JOINTS);

  if (!g_controller.path_valid || g_controller.path.L < 1e-9 ||
      TRAJ_PLAN_SPEED <= 0.0) {
    return;
  }

  next_s = g_controller.path_s + TRAJ_PLAN_STEP_M;
  if (next_s > g_controller.path.L) {
    next_s = g_controller.path.L;
  }

  ds = next_s - g_controller.path_s;
  if (ds <= 1e-12) {
    return;
  }

  dt_segment = ds / TRAJ_PLAN_SPEED;
  if (dt_segment <= 1e-12 || !isfinite(dt_segment)) {
    return;
  }

  stm_controller_sample_path_at_s(next_s, next_pos, next_quat);
  if (!stm_controller_compute_q_ref_from_initial(q_ref, next_pos, next_quat,
                                                 q_next_ref)) {
    return;
  }

  for (int i = 0; i < NUM_JOINTS; ++i) {
    qd_ref[i] = normalize_angle(q_next_ref[i] - q_ref[i]) / dt_segment;
  }
}

static void stm_controller_compute_equivalent_tau(
    const double current_q[NUM_JOINTS], const double current_qd[NUM_JOINTS],
    stm_output_t *out) {
  for (int i = 0; i < NUM_JOINTS; ++i) {
    double pos_err = normalize_angle(out->q_ref[i] - current_q[i]);
    double vel_err = out->qd_ref[i] - current_qd[i];
    out->tau[i] = out->kp[i] * pos_err + out->kd[i] * vel_err + out->tau_ff[i];

    if (out->tau[i] > TORQUE_LIMIT_CHECK[i])
      out->tau[i] = TORQUE_LIMIT_CHECK[i];
    if (out->tau[i] < -TORQUE_LIMIT_CHECK[i])
      out->tau[i] = -TORQUE_LIMIT_CHECK[i];
  }
}

void stm_controller_set_platform_hooks(const stm_platform_hooks_t *hooks) {
  if (hooks == NULL) {
    memset(&g_controller.hooks, 0, sizeof(g_controller.hooks));
    return;
  }

  g_controller.hooks = *hooks;
}

void stm_controller_reset(void) {
  g_controller.path_s = 0.0;
  g_controller.step_count = 0;
  g_controller.path_valid = 0;
  g_controller.have_last_q_ref = 0;
  memset(&g_controller.path, 0, sizeof(g_controller.path));
  memset(g_controller.latched_target_pos, 0, sizeof(g_controller.latched_target_pos));
  memset(g_controller.latched_target_quat, 0, sizeof(g_controller.latched_target_quat));
  memset(g_controller.last_q_ref, 0, sizeof(g_controller.last_q_ref));
}

void stm_controller_init(void) {
  if (g_controller.initialized) {
    return;
  }

  control_init();
  kinematics_init(&g_stm_ik_solver);
  build_am_d02_model(&g_stm_dynamics_model);
  stm_controller_reset();
  g_controller.initialized = 1;
}

void stm_controller_step(const stm_input_t *in, stm_output_t *out) {
  double filtered_qd[NUM_JOINTS];
  double target_pos[3];
  double target_quat[4];
  double ref_pos[3];
  double ref_quat[4];
  double start_ms;
  double end_ms;
  int ik_status;

  if (in == NULL || out == NULL) {
    return;
  }

  if (!g_controller.initialized) {
    stm_controller_init();
  }

  stm_controller_prepare_output(out);
  stm_controller_fill_mit_gains(out);
  start_ms = stm_controller_now_ms();

  control_filter_velocities(in->qd, filtered_qd);
  control_get_fk_with_offset(in->q, out->ee_pos, out->ee_quat);
  stm_controller_sanitize_target_pose(in, out->ee_pos, out->ee_quat, target_pos,
                                      target_quat);

  if (control_check_safety(in->q, filtered_qd) < 0) {
    stm_controller_fill_zero_mit(in->q, out);
    goto finalize_step;
  }

  if (stm_controller_target_changed(target_pos, target_quat)) {
    stm_controller_start_path(out->ee_pos, out->ee_quat, target_pos,
                              target_quat);
  }

  if (g_controller.path_valid) {
    stm_controller_evaluate_round_path(ref_pos, ref_quat);
  } else {
    memcpy(ref_pos, target_pos, sizeof(ref_pos));
    memcpy(ref_quat, target_quat, sizeof(ref_quat));
  }

  ik_status = stm_controller_compute_q_ref(in->q, ref_pos, ref_quat,
                                           out->q_ref);
  if (!ik_status) {
    memcpy(out->q_ref, in->q, sizeof(out->q_ref));
    memset(out->qd_ref, 0, sizeof(out->qd_ref));
    out->status = 1;
  } else {
    stm_controller_compute_qd_ref(out->q_ref, out->qd_ref);
  }

  rbdl_calc_gc(&g_stm_dynamics_model, in->q, filtered_qd, out->tau_ff);
  stm_controller_compute_equivalent_tau(in->q, filtered_qd, out);

  if (stm_controller_check_joint_safety(in->q, filtered_qd, out->tau) == 0) {
    if (out->status != 1) {
      out->status = 0;
    }
    memcpy(g_controller.last_q_ref, out->q_ref, sizeof(g_controller.last_q_ref));
    g_controller.have_last_q_ref = 1;
  } else {
    stm_controller_fill_zero_mit(in->q, out);
    out->status = -1;
  }

finalize_step:
  g_controller.step_count++;
  out->path_progress = g_controller.path_s;
  out->step_count = g_controller.step_count;

  end_ms = stm_controller_now_ms();
  if (g_controller.hooks.now_ms != NULL && end_ms >= start_ms) {
    out->calc_time_ms = end_ms - start_ms;
  }
}
