#include "stm_controller.h"

#include "control_logic.h"
#include "trajectory_lib.h"
#include <math.h>
#include <stdio.h>
#include <string.h>

typedef struct {
  double traj_t;
  double path_t;
  int step_count;
  int initialized;
  int path_valid;
  stm_platform_hooks_t hooks;
  LinearPathPlanner path;
  double latched_target_pos[3];
  double latched_target_quat[4];
} StmControllerState;

static StmControllerState g_controller = {0};

static const double TORQUE_LIMIT_CHECK[NUM_JOINTS] = {
    JOINT_TORQUE_LIMIT_1, JOINT_TORQUE_LIMIT_2, JOINT_TORQUE_LIMIT_3,
    JOINT_TORQUE_LIMIT_4, JOINT_TORQUE_LIMIT_5, JOINT_TORQUE_LIMIT_6,
    JOINT_TORQUE_LIMIT_7};

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
  g_controller.path_t = 0.0;
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

void stm_controller_set_platform_hooks(const stm_platform_hooks_t *hooks) {
  if (hooks == NULL) {
    memset(&g_controller.hooks, 0, sizeof(g_controller.hooks));
    return;
  }

  g_controller.hooks = *hooks;
}

void stm_controller_reset(void) {
  g_controller.traj_t = 0.0;
  g_controller.path_t = 0.0;
  g_controller.step_count = 0;
  g_controller.path_valid = 0;
  memset(&g_controller.path, 0, sizeof(g_controller.path));
  memset(g_controller.latched_target_pos, 0, sizeof(g_controller.latched_target_pos));
  memset(g_controller.latched_target_quat, 0, sizeof(g_controller.latched_target_quat));
}

void stm_controller_init(void) {
  if (g_controller.initialized) {
    return;
  }

  control_init();
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

  if (in == NULL || out == NULL) {
    return;
  }

  if (!g_controller.initialized) {
    stm_controller_init();
  }

  stm_controller_prepare_output(out);
  start_ms = stm_controller_now_ms();

  control_filter_velocities(in->qd, filtered_qd);
  control_get_fk_with_offset(in->q, out->ee_pos, out->ee_quat);
  stm_controller_sanitize_target_pose(in, out->ee_pos, out->ee_quat, target_pos,
                                      target_quat);

  if (control_check_safety(in->q, filtered_qd) < 0) {
    memset(out->tau, 0, sizeof(out->tau));
    goto finalize_step;
  }

  if (stm_controller_target_changed(target_pos, target_quat)) {
    stm_controller_start_path(out->ee_pos, out->ee_quat, target_pos,
                              target_quat);
  }

  if (g_controller.path_valid) {
    linear_path_evaluate(&g_controller.path, g_controller.path_t, ref_pos,
                         ref_quat);
  } else {
    memcpy(ref_pos, target_pos, sizeof(ref_pos));
    memcpy(ref_quat, target_quat, sizeof(ref_quat));
  }

  control_step_v2(ref_pos, ref_quat, in->q, filtered_qd, out->tau);

  if (stm_controller_check_joint_safety(in->q, filtered_qd, out->tau) == 0) {
    out->status = 0;
  } else {
    memset(out->tau, 0, sizeof(out->tau));
  }

finalize_step:
  g_controller.traj_t += CONTROL_DT;
  g_controller.path_t += CONTROL_DT;
  g_controller.step_count++;
  out->traj_t = g_controller.traj_t;
  out->step_count = g_controller.step_count;

  end_ms = stm_controller_now_ms();
  if (g_controller.hooks.now_ms != NULL && end_ms >= start_ms) {
    out->calc_time_ms = end_ms - start_ms;
  }
}
