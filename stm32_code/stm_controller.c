#include "stm_controller.h"

#include "control_logic.h"
#include "trajectory_lib.h"
#include <math.h>
#include <string.h>

typedef struct {
  double traj_t;
  int step_count;
  int initialized;
  int safety_latched;
  int last_safety_reason;
  uint8_t safety_arm_mask;
  int target_valid[NUM_ARMS];
  int ref_valid[NUM_ARMS];
  double zero_hold_s;
  double target_pos[NUM_ARMS][3];
  double target_quat[NUM_ARMS][4];
  double ref_pos[NUM_ARMS][3];
  double ref_quat[NUM_ARMS][4];
  LinearPathPlanner path_planner[NUM_ARMS];
  double path_progress_t[NUM_ARMS];
  double path_gate[NUM_ARMS];
  double path_lookahead_m[NUM_ARMS];
} StmControllerState;

static StmControllerState g_controller = {0};

static const double TORQUE_LIMIT_CHECK[NUM_JOINTS] = {
    JOINT_TORQUE_LIMIT_1, JOINT_TORQUE_LIMIT_2, JOINT_TORQUE_LIMIT_3,
    JOINT_TORQUE_LIMIT_4, JOINT_TORQUE_LIMIT_5, JOINT_TORQUE_LIMIT_6,
    JOINT_TORQUE_LIMIT_7, JOINT_TORQUE_LIMIT_1, JOINT_TORQUE_LIMIT_2,
    JOINT_TORQUE_LIMIT_3, JOINT_TORQUE_LIMIT_4, 9.0, 9.0,
    JOINT_TORQUE_LIMIT_7};

static int stm_controller_is_finite_vec(const double *values, int count) {
  for (int i = 0; i < count; ++i) {
    if (!isfinite(values[i])) {
      return 0;
    }
  }
  return 1;
}

static void stm_controller_clear_path_state(void) {
  memset(g_controller.target_valid, 0, sizeof(g_controller.target_valid));
  memset(g_controller.ref_valid, 0, sizeof(g_controller.ref_valid));
  memset(g_controller.target_pos, 0, sizeof(g_controller.target_pos));
  memset(g_controller.target_quat, 0, sizeof(g_controller.target_quat));
  memset(g_controller.ref_pos, 0, sizeof(g_controller.ref_pos));
  memset(g_controller.ref_quat, 0, sizeof(g_controller.ref_quat));
  memset(g_controller.path_planner, 0, sizeof(g_controller.path_planner));
  memset(g_controller.path_progress_t, 0, sizeof(g_controller.path_progress_t));
  memset(g_controller.path_gate, 0, sizeof(g_controller.path_gate));
  memset(g_controller.path_lookahead_m, 0, sizeof(g_controller.path_lookahead_m));
}

static double stm_controller_clamp(double value, double low, double high) {
  if (value < low) {
    return low;
  }
  if (value > high) {
    return high;
  }
  return value;
}

static double stm_controller_vec3_distance(const double a[3],
                                           const double b[3]) {
  double dx = a[0] - b[0];
  double dy = a[1] - b[1];
  double dz = a[2] - b[2];
  return sqrt(dx * dx + dy * dy + dz * dz);
}

static double stm_controller_path_distance_at_time(const LinearPathPlanner *lp,
                                                   double t) {
  if (lp == NULL || lp->L < 1e-6 || t <= 0.0 || !isfinite(t)) {
    return 0.0;
  }
  if (t >= lp->total_time) {
    return lp->L;
  }
  if (t < lp->t_a) {
    return 0.5 * lp->a * t * t;
  }
  if (t < lp->t_a + lp->t_c) {
    return lp->d_a + lp->v_max * (t - lp->t_a);
  }

  double dt = t - lp->t_a - lp->t_c;
  double s = lp->d_a + lp->v_max * lp->t_c + lp->v_max * dt -
             0.5 * lp->a * dt * dt;
  return stm_controller_clamp(s, 0.0, lp->L);
}

static double stm_controller_path_time_at_distance(const LinearPathPlanner *lp,
                                                   double s) {
  if (lp == NULL || lp->L < 1e-6 || s <= 0.0 || !isfinite(s)) {
    return 0.0;
  }
  if (s >= lp->L) {
    return lp->total_time;
  }
  if (lp->a <= 0.0 || !isfinite(lp->a)) {
    return 0.0;
  }
  if (s <= lp->d_a) {
    return sqrt(2.0 * s / lp->a);
  }
  if (lp->t_c > 0.0 && s <= lp->d_a + lp->v_max * lp->t_c) {
    return lp->t_a + (s - lp->d_a) / lp->v_max;
  }

  double remaining = lp->L - s;
  if (remaining <= 0.0) {
    return lp->total_time;
  }
  return lp->total_time - sqrt(2.0 * remaining / lp->a);
}

static uint8_t stm_controller_normalize_arm_mask(uint8_t active_arm_mask) {
  uint8_t mask = active_arm_mask & STM_ARM_MASK_BOTH;
  return mask == 0u ? STM_ARM_MASK_BOTH : mask;
}

static void stm_controller_enter_safety_latch(int reason, uint8_t active_arm_mask) {
  g_controller.safety_latched = 1;
  g_controller.zero_hold_s = 0.0;
  g_controller.last_safety_reason = reason;
  g_controller.safety_arm_mask = stm_controller_normalize_arm_mask(active_arm_mask);
}

static int stm_controller_joints_still(const double qd[NUM_JOINTS],
                                       uint8_t active_arm_mask) {
  uint8_t mask = stm_controller_normalize_arm_mask(active_arm_mask);
  for (int arm = 0; arm < NUM_ARMS; ++arm) {
    if ((mask & (1u << arm)) == 0u) {
      continue;
    }
    int offset = arm * ARM_JOINTS;
    for (int i = 0; i < ARM_JOINTS; ++i) {
      if (!isfinite(qd[offset + i])) {
        return 0;
      }
      if (fabs(qd[offset + i]) > SAFETY_RECOVERY_QD_ZERO_TOL) {
        return 0;
      }
    }
  }
  return 1;
}

static int stm_controller_update_safety_recovery(const double qd[NUM_JOINTS],
                                                 double elapsed_s) {
  if (!g_controller.safety_latched) {
    return 0;
  }

  if (stm_controller_joints_still(qd, g_controller.safety_arm_mask)) {
    g_controller.zero_hold_s += elapsed_s;
    if (g_controller.zero_hold_s >= SAFETY_RECOVERY_HOLD_S) {
      g_controller.safety_latched = 0;
      g_controller.zero_hold_s = 0.0;
      g_controller.last_safety_reason = STM_STATUS_OK;
      g_controller.safety_arm_mask = STM_ARM_MASK_BOTH;
      stm_controller_clear_path_state();
    }
  } else {
    g_controller.zero_hold_s = 0.0;
  }

  return 1;
}

static int stm_controller_pose_changed(int arm, const double target_pos[3],
                                       const double target_quat[4]) {
  if (!g_controller.target_valid[arm]) {
    return 1;
  }

  for (int i = 0; i < 3; ++i) {
    if (fabs(target_pos[i] - g_controller.target_pos[arm][i]) >
        CONTROL_TARGET_REPLAN_POS_EPS_M) {
      return 1;
    }
  }

  double dot = 0.0;
  for (int i = 0; i < 4; ++i) {
    dot += target_quat[i] * g_controller.target_quat[arm][i];
  }
  double dot_abs = fabs(dot);
  dot_abs = stm_controller_clamp(dot_abs, 0.0, 1.0);
  double angle = 2.0 * acos(dot_abs);
  return angle > CONTROL_TARGET_REPLAN_ORI_EPS_RAD;
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

static double stm_controller_tracking_gate(double error_m, double elapsed_s,
                                           double previous_gate) {
  double gate_cmd;
  double gate_delta;
  double full_error = CONTROL_PATH_GATE_FULL_ERROR_M;
  double stop_error = CONTROL_PATH_GATE_STOP_ERROR_M;

  if (!isfinite(error_m)) {
    gate_cmd = 0.0;
  } else if (error_m <= full_error) {
    gate_cmd = 1.0;
  } else if (error_m >= stop_error || stop_error <= full_error) {
    gate_cmd = 0.0;
  } else {
    double x = (stop_error - error_m) / (stop_error - full_error);
    x = stm_controller_clamp(x, 0.0, 1.0);
    gate_cmd = x * x * (3.0 - 2.0 * x);
  }

  if (elapsed_s <= 0.0 || !isfinite(elapsed_s)) {
    return stm_controller_clamp(previous_gate, 0.0, 1.0);
  }

  gate_delta = gate_cmd - previous_gate;
  if (gate_delta >= 0.0) {
    double max_rise = elapsed_s / CONTROL_PATH_GATE_RISE_TIME_S;
    gate_delta = stm_controller_clamp(gate_delta, 0.0, max_rise);
  } else {
    double max_fall = elapsed_s / CONTROL_PATH_GATE_FALL_TIME_S;
    gate_delta = stm_controller_clamp(gate_delta, -max_fall, 0.0);
  }

  return stm_controller_clamp(previous_gate + gate_delta, 0.0, 1.0);
}

static void stm_controller_update_reference_arm(int arm,
                                                const double current_pos[3],
                                                const double current_quat[4],
                                                const double target_pos[3],
                                                const double target_quat[4],
                                                double step_s, double ref_pos[3],
                                                double ref_quat[4]) {
  int replan = stm_controller_pose_changed(arm, target_pos, target_quat);
  double tracking_error;
  double lookahead_step;
  double base_distance;
  double eval_distance;
  double eval_t;

  if (!g_controller.ref_valid[arm]) {
    memcpy(g_controller.ref_pos[arm], current_pos, sizeof(double) * 3);
    memcpy(g_controller.ref_quat[arm], current_quat, sizeof(double) * 4);
    g_controller.ref_valid[arm] = 1;
    g_controller.path_gate[arm] = 1.0;
    g_controller.path_progress_t[arm] = 0.0;
    g_controller.path_lookahead_m[arm] = 0.0;
  }

  if (replan) {
    tracking_error =
        stm_controller_vec3_distance(current_pos, g_controller.ref_pos[arm]);
    if (tracking_error >= CONTROL_PATH_GATE_STOP_ERROR_M) {
      memcpy(g_controller.ref_pos[arm], current_pos, sizeof(double) * 3);
      memcpy(g_controller.ref_quat[arm], current_quat, sizeof(double) * 4);
      g_controller.path_gate[arm] = 0.0;
      g_controller.path_lookahead_m[arm] = 0.0;
    }
    linear_path_init(&g_controller.path_planner[arm], g_controller.ref_pos[arm],
                     g_controller.ref_quat[arm], target_pos, target_quat,
                     TRAJ_PLAN_SPEED, TRAJ_PLAN_ACCEL);
    g_controller.path_progress_t[arm] = 0.0;
    memcpy(g_controller.target_pos[arm], target_pos, sizeof(double) * 3);
    memcpy(g_controller.target_quat[arm], target_quat, sizeof(double) * 4);
    g_controller.target_valid[arm] = 1;
  }

  tracking_error =
      stm_controller_vec3_distance(current_pos, g_controller.ref_pos[arm]);
  g_controller.path_gate[arm] =
      stm_controller_tracking_gate(tracking_error, step_s, g_controller.path_gate[arm]);
  g_controller.path_progress_t[arm] += step_s * g_controller.path_gate[arm];
  if (g_controller.path_progress_t[arm] < 0.0 ||
      !isfinite(g_controller.path_progress_t[arm])) {
    g_controller.path_progress_t[arm] = 0.0;
  }

  if (step_s > 0.0 && isfinite(step_s) &&
      CONTROL_PATH_LOOKAHEAD_RAMP_S > 0.0) {
    lookahead_step =
        CONTROL_PATH_LOOKAHEAD_M * step_s / CONTROL_PATH_LOOKAHEAD_RAMP_S;
    g_controller.path_lookahead_m[arm] += lookahead_step;
    if (g_controller.path_lookahead_m[arm] > CONTROL_PATH_LOOKAHEAD_M) {
      g_controller.path_lookahead_m[arm] = CONTROL_PATH_LOOKAHEAD_M;
    }
  }

  base_distance = stm_controller_path_distance_at_time(
      &g_controller.path_planner[arm], g_controller.path_progress_t[arm]);
  eval_distance = base_distance + g_controller.path_lookahead_m[arm];
  if (eval_distance > g_controller.path_planner[arm].L) {
    eval_distance = g_controller.path_planner[arm].L;
  }
  eval_t = stm_controller_path_time_at_distance(&g_controller.path_planner[arm],
                                                eval_distance);
  linear_path_evaluate(&g_controller.path_planner[arm],
                       eval_t,
                       g_controller.ref_pos[arm],
                       g_controller.ref_quat[arm]);

  memcpy(ref_pos, g_controller.ref_pos[arm], sizeof(double) * 3);
  memcpy(ref_quat, g_controller.ref_quat[arm], sizeof(double) * 4);
}

static void stm_controller_prepare_output(stm_output_t *out) {
  memset(out, 0, sizeof(*out));
  out->status = STM_STATUS_SAFETY_LATCHED;
}

static int stm_controller_check_joint_safety(const double q[NUM_JOINTS],
                                             const double qd[NUM_JOINTS],
                                             const double tau[NUM_JOINTS],
                                             uint8_t active_arm_mask) {
  uint8_t mask = stm_controller_normalize_arm_mask(active_arm_mask);

  for (int arm = 0; arm < NUM_ARMS; ++arm) {
    if ((mask & (1u << arm)) == 0u) {
      continue;
    }
    int offset = arm * ARM_JOINTS;
    if (!stm_controller_is_finite_vec(q + offset, ARM_JOINTS) ||
        !stm_controller_is_finite_vec(qd + offset, ARM_JOINTS)) {
      STM_LOG_ERROR("[SAFETY] Arm %d non-finite joint state detected.\n", arm);
      return -1;
    }
    int status = control_check_safety_arm(arm, q + offset, qd + offset);
    if (status < 0) {
      if (status == -1) {
        STM_LOG_ERROR("[SAFETY] Arm %d joint position limit violated!\n", arm);
      } else if (status == -2) {
        STM_LOG_ERROR("[SAFETY] Arm %d joint velocity limit violated!\n", arm);
      }
      return -1;
    }
  }

  for (int arm = 0; arm < NUM_ARMS; ++arm) {
    if ((mask & (1u << arm)) == 0u) {
      continue;
    }
    int offset = arm * ARM_JOINTS;
    for (int i = 0; i < ARM_JOINTS; ++i) {
      int joint = offset + i;
      if (!isfinite(tau[joint])) {
        STM_LOG_ERROR("[SAFETY] J%d torque is non-finite.\n", joint + 1);
        return -1;
      }
      if (fabs(tau[joint]) > TORQUE_LIMIT_CHECK[joint]) {
        STM_LOG_ERROR("[SAFETY] J%d torque error: %.4f\n", joint + 1,
                      tau[joint]);
        return -1;
      }
    }
  }

  return 0;
}

void stm_controller_reset(void) {
  g_controller.traj_t = 0.0;
  g_controller.step_count = 0;
  g_controller.safety_latched = 0;
  g_controller.zero_hold_s = 0.0;
  g_controller.last_safety_reason = STM_STATUS_OK;
  g_controller.safety_arm_mask = STM_ARM_MASK_BOTH;
  stm_controller_clear_path_state();
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
  control_arm_kinematics_t arm_kinematics[NUM_ARMS];
  uint8_t active_arm_mask;

  if (in == NULL || out == NULL) {
    return;
  }

  if (!g_controller.initialized) {
    stm_controller_init();
  }

  if (elapsed_s < 0.0 || !isfinite(elapsed_s)) {
    elapsed_s = 0.0;
  } else if (elapsed_s > CONTROL_MAX_ELAPSED_S) {
    elapsed_s = CONTROL_MAX_ELAPSED_S;
  }

  stm_controller_prepare_output(out);
  active_arm_mask = stm_controller_normalize_arm_mask(in->active_arm_mask);

  if (stm_controller_update_safety_recovery(in->qd, elapsed_s)) {
    out->status = STM_STATUS_WAITING_ZERO;
    memset(out->tau, 0, sizeof(out->tau));
    goto finalize_step;
  }

  if (stm_controller_is_finite_vec(in->body_q, NUM_BODY_JOINTS)) {
    control_update_body_gravity(in->body_q);
  }

  for (int arm = 0; arm < NUM_ARMS; ++arm) {
    if ((active_arm_mask & (1u << arm)) == 0u) {
      continue;
    }
    int offset = arm * ARM_JOINTS;
    if (!stm_controller_is_finite_vec(in->q + offset, ARM_JOINTS) ||
        !stm_controller_is_finite_vec(in->qd + offset, ARM_JOINTS)) {
      stm_controller_enter_safety_latch(STM_STATUS_SAFETY_LATCHED,
                                        active_arm_mask);
      out->status = STM_STATUS_SAFETY_LATCHED;
      memset(out->tau, 0, sizeof(out->tau));
      goto finalize_step;
    }
    control_filter_velocities_arm(arm, in->qd + offset, filtered_qd + offset);
    control_get_arm_kinematics_with_offset(arm, in->q + offset,
                                           &arm_kinematics[arm]);
    memcpy(out->ee_pos[arm], arm_kinematics[arm].pos, sizeof(double) * 3);
    memcpy(out->ee_quat[arm], arm_kinematics[arm].quat_wxyz,
           sizeof(double) * 4);
    stm_controller_sanitize_target_pose(in, arm, out->ee_pos[arm],
                                        out->ee_quat[arm], target_pos[arm],
                                        target_quat[arm]);
    if (control_check_safety_arm(arm, in->q + offset, filtered_qd + offset) < 0) {
      stm_controller_enter_safety_latch(STM_STATUS_SAFETY_LATCHED,
                                        active_arm_mask);
      out->status = STM_STATUS_SAFETY_LATCHED;
      memset(out->tau, 0, sizeof(out->tau));
      goto finalize_step;
    }
  }

  for (int arm = 0; arm < NUM_ARMS; ++arm) {
    if ((active_arm_mask & (1u << arm)) == 0u) {
      continue;
    }
    stm_controller_update_reference_arm(arm, out->ee_pos[arm], out->ee_quat[arm],
                                        target_pos[arm], target_quat[arm], elapsed_s,
                                        ref_pos[arm], ref_quat[arm]);
  }

  for (int arm = 0; arm < NUM_ARMS; ++arm) {
    if ((active_arm_mask & (1u << arm)) == 0u) {
      continue;
    }
    int offset = arm * ARM_JOINTS;
    control_step_v2_arm_with_state(arm, ref_pos[arm], ref_quat[arm],
                                   in->q + offset, filtered_qd + offset,
                                   &arm_kinematics[arm], out->tau + offset);
  }

  if (stm_controller_check_joint_safety(in->q, filtered_qd, out->tau,
                                        active_arm_mask) == 0) {
    out->status = STM_STATUS_OK;
  } else {
    stm_controller_enter_safety_latch(STM_STATUS_SAFETY_LATCHED,
                                      active_arm_mask);
    out->status = STM_STATUS_SAFETY_LATCHED;
    memset(out->tau, 0, sizeof(out->tau));
  }

finalize_step:
  g_controller.traj_t += elapsed_s;
  g_controller.step_count++;
  out->traj_t = g_controller.traj_t;
  out->step_count = g_controller.step_count;
}
