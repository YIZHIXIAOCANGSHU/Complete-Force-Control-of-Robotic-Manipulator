#include "stm_controller.h"

#include "control_logic.h"
#include <math.h>
#include <stdio.h>
#include <string.h>

typedef struct {
  double traj_t;
  int step_count;
  int initialized;
  int target_valid[NUM_ARMS];
  int ref_valid[NUM_ARMS];
  stm_platform_hooks_t hooks;
  double target_pos[NUM_ARMS][3];
  double target_quat[NUM_ARMS][4];
  double ref_pos[NUM_ARMS][3];
  double ref_quat[NUM_ARMS][4];
} StmControllerState;

static StmControllerState g_controller = {0};

static const double TORQUE_LIMIT_CHECK[NUM_JOINTS] = {
    JOINT_TORQUE_LIMIT_1, JOINT_TORQUE_LIMIT_2, JOINT_TORQUE_LIMIT_3,
    JOINT_TORQUE_LIMIT_4, JOINT_TORQUE_LIMIT_5, JOINT_TORQUE_LIMIT_6,
    JOINT_TORQUE_LIMIT_7, JOINT_TORQUE_LIMIT_1, JOINT_TORQUE_LIMIT_2,
    JOINT_TORQUE_LIMIT_3, JOINT_TORQUE_LIMIT_4, 9.0, 9.0,
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
                                                int arm,
                                                const double fallback_pos[3],
                                                const double fallback_quat[4],
                                                double target_pos[3],
                                                double target_quat[4]) {
  double quat_norm_sq = 0.0;
  int pos_valid = stm_controller_is_finite_vec(in->target_pos[arm], 3);
  int quat_valid = stm_controller_is_finite_vec(in->target_quat[arm], 4);

  if (pos_valid) {
    memcpy(target_pos, in->target_pos[arm], sizeof(double) * 3);
  } else if (g_controller.target_valid[arm]) {
    memcpy(target_pos, g_controller.target_pos[arm], sizeof(double) * 3);
  } else {
    memcpy(target_pos, fallback_pos, sizeof(double) * 3);
  }

  if (quat_valid) {
    for (int i = 0; i < 4; ++i) {
      quat_norm_sq += in->target_quat[arm][i] * in->target_quat[arm][i];
    }
  }

  if (quat_norm_sq > 1e-12 && isfinite(quat_norm_sq)) {
    double quat_norm_inv = 1.0 / sqrt(quat_norm_sq);
    for (int i = 0; i < 4; ++i) {
      target_quat[i] = in->target_quat[arm][i] * quat_norm_inv;
    }
  } else if (g_controller.target_valid[arm]) {
    memcpy(target_quat, g_controller.target_quat[arm], sizeof(double) * 4);
  } else {
    memcpy(target_quat, fallback_quat, sizeof(double) * 4);
  }
}

static void stm_controller_update_reference_arm(int arm,
                                                const double current_pos[3],
                                                const double current_quat[4],
                                                const double target_pos[3],
                                                const double target_quat[4],
                                                double step_s, double ref_pos[3],
                                                double ref_quat[4]) {
  double delta[3];
  double distance;
  double ratio;

  memcpy(g_controller.target_pos[arm], target_pos, sizeof(double) * 3);
  memcpy(g_controller.target_quat[arm], target_quat, sizeof(double) * 4);
  g_controller.target_valid[arm] = 1;

  if (!g_controller.ref_valid[arm]) {
    memcpy(g_controller.ref_pos[arm], current_pos, sizeof(double) * 3);
    memcpy(g_controller.ref_quat[arm], current_quat, sizeof(double) * 4);
    g_controller.ref_valid[arm] = 1;
  }

  for (int i = 0; i < 3; ++i) {
    delta[i] = target_pos[i] - g_controller.ref_pos[arm][i];
  }
  distance = sqrt(delta[0] * delta[0] + delta[1] * delta[1] +
                  delta[2] * delta[2]);

  if (distance <= 1e-9) {
    ratio = 1.0;
    memcpy(g_controller.ref_pos[arm], target_pos, sizeof(double) * 3);
  } else {
    double max_step = TRAJ_PLAN_SPEED * step_s;
    ratio = max_step / distance;
    if (ratio > 1.0) {
      ratio = 1.0;
    }
    if (ratio < 0.0 || !isfinite(ratio)) {
      ratio = 0.0;
    }
    for (int i = 0; i < 3; ++i) {
      g_controller.ref_pos[arm][i] += ratio * delta[i];
    }
  }

  quat_slerp(g_controller.ref_quat[arm], target_quat, ratio,
             g_controller.ref_quat[arm]);

  memcpy(ref_pos, g_controller.ref_pos[arm], sizeof(double) * 3);
  memcpy(ref_quat, g_controller.ref_quat[arm], sizeof(double) * 4);
}

static void stm_controller_update_reference(const double current_pos[3],
                                            const double current_quat[4],
                                            const double target_pos[3],
                                            const double target_quat[4],
                                            double step_s, double ref_pos[3],
                                            double ref_quat[4])
    __attribute__((unused));

static void stm_controller_update_reference(const double current_pos[3],
                                            const double current_quat[4],
                                            const double target_pos[3],
                                            const double target_quat[4],
                                            double step_s, double ref_pos[3],
                                            double ref_quat[4]) {
  stm_controller_update_reference_arm(ARM_LEFT, current_pos, current_quat,
                                      target_pos, target_quat, step_s, ref_pos,
                                      ref_quat);
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

  for (int arm = 0; arm < NUM_ARMS; ++arm) {
    int offset = arm * ARM_JOINTS;
    int status = control_check_safety_arm(arm, q + offset, qd + offset);
    if (status < 0) {
      if (status == -1) {
        fprintf(stderr, "[SAFETY] Arm %d joint position limit violated!\n", arm);
      } else if (status == -2) {
        fprintf(stderr, "[SAFETY] Arm %d joint velocity limit violated!\n", arm);
      }
      return -1;
    }
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
  g_controller.step_count = 0;
  memset(g_controller.target_valid, 0, sizeof(g_controller.target_valid));
  memset(g_controller.ref_valid, 0, sizeof(g_controller.ref_valid));
  memset(g_controller.target_pos, 0, sizeof(g_controller.target_pos));
  memset(g_controller.target_quat, 0, sizeof(g_controller.target_quat));
  memset(g_controller.ref_pos, 0, sizeof(g_controller.ref_pos));
  memset(g_controller.ref_quat, 0, sizeof(g_controller.ref_quat));
}

void stm_controller_init(void) {
  if (g_controller.initialized) {
    return;
  }

  control_init();
  stm_controller_reset();
  g_controller.initialized = 1;
}

void stm_controller_step_elapsed(const stm_input_t *in, stm_output_t *out,
                                 double elapsed_s) {
  double filtered_qd[NUM_JOINTS];
  double target_pos[NUM_ARMS][3];
  double target_quat[NUM_ARMS][4];
  double ref_pos[NUM_ARMS][3];
  double ref_quat[NUM_ARMS][4];
  double start_ms;
  double end_ms;

  if (in == NULL || out == NULL) {
    return;
  }

  if (!g_controller.initialized) {
    stm_controller_init();
  }

  if (elapsed_s < 0.0 || !isfinite(elapsed_s)) {
    elapsed_s = 0.0;
  }

  stm_controller_prepare_output(out);
  start_ms = stm_controller_now_ms();
  if (stm_controller_is_finite_vec(in->body_q, NUM_BODY_JOINTS)) {
    control_update_body_gravity(in->body_q);
  }

  for (int arm = 0; arm < NUM_ARMS; ++arm) {
    int offset = arm * ARM_JOINTS;
    control_filter_velocities_arm(arm, in->qd + offset, filtered_qd + offset);
    control_get_fk_with_offset_arm(arm, in->q + offset, out->ee_pos[arm],
                                   out->ee_quat[arm]);
    stm_controller_sanitize_target_pose(in, arm, out->ee_pos[arm],
                                        out->ee_quat[arm], target_pos[arm],
                                        target_quat[arm]);
    if (control_check_safety_arm(arm, in->q + offset, filtered_qd + offset) < 0) {
      memset(out->tau, 0, sizeof(out->tau));
      goto finalize_step;
    }
  }

  for (int arm = 0; arm < NUM_ARMS; ++arm) {
    stm_controller_update_reference_arm(arm, out->ee_pos[arm], out->ee_quat[arm],
                                        target_pos[arm], target_quat[arm], elapsed_s,
                                        ref_pos[arm], ref_quat[arm]);
  }

  control_step_v2_dual(ref_pos, ref_quat, in->q, filtered_qd, out->tau);

  if (stm_controller_check_joint_safety(in->q, filtered_qd, out->tau) == 0) {
    out->status = 0;
  } else {
    memset(out->tau, 0, sizeof(out->tau));
  }

finalize_step:
  g_controller.traj_t += elapsed_s;
  g_controller.step_count++;
  out->traj_t = g_controller.traj_t;
  out->step_count = g_controller.step_count;

  end_ms = stm_controller_now_ms();
  if (g_controller.hooks.now_ms != NULL && end_ms >= start_ms) {
    out->calc_time_ms = end_ms - start_ms;
  }
}

void stm_controller_step(const stm_input_t *in, stm_output_t *out) {
  stm_controller_step_elapsed(in, out, CONTROL_DT);
}
