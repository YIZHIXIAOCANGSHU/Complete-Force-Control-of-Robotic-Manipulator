#include "stm_controller.h"

#include "control_logic.h"
#include <math.h>
#include <string.h>

typedef struct {
  int initialized;
  int arrived;
  double start_pos[3];
  double start_quat[4];
  double target_pos[3];
  double target_quat[4];
  double ref_pos[3];
  double ref_quat[4];
  double direction[3];
  double path_length;
  double progress;
  double path_time_s;
  double duration_s;
} CartesianPathState;

typedef struct {
  double traj_t;
  int step_count;
  int initialized;
  int safety_latched;
  int last_safety_reason;
  uint8_t safety_arm_mask;
  int target_valid[NUM_ARMS];
  double zero_hold_s;
  double target_pos[NUM_ARMS][3];
  double target_quat[NUM_ARMS][4];
  CartesianPathState path[NUM_ARMS];
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

static void stm_controller_clear_target_state(void) {
  memset(g_controller.target_valid, 0, sizeof(g_controller.target_valid));
  memset(g_controller.target_pos, 0, sizeof(g_controller.target_pos));
  memset(g_controller.target_quat, 0, sizeof(g_controller.target_quat));
}

static void stm_controller_clear_path_state(void) {
  memset(g_controller.path, 0, sizeof(g_controller.path));
}

static uint8_t stm_controller_normalize_arm_mask(uint8_t active_arm_mask) {
  uint8_t mask = active_arm_mask & STM_ARM_MASK_BOTH;
  return mask == 0u ? STM_ARM_MASK_BOTH : mask;
}

static double stm_controller_vec3_distance(const double a[3],
                                           const double b[3]) {
  double dx = a[0] - b[0];
  double dy = a[1] - b[1];
  double dz = a[2] - b[2];
  return sqrt(dx * dx + dy * dy + dz * dz);
}

static void stm_controller_quat_wxyz_to_xyzw(const double q_wxyz[4],
                                             double q_xyzw[4]) {
  q_xyzw[0] = q_wxyz[1];
  q_xyzw[1] = q_wxyz[2];
  q_xyzw[2] = q_wxyz[3];
  q_xyzw[3] = q_wxyz[0];
}

static void stm_controller_quat_xyzw_to_wxyz(const double q_xyzw[4],
                                             double q_wxyz[4]) {
  q_wxyz[0] = q_xyzw[3];
  q_wxyz[1] = q_xyzw[0];
  q_wxyz[2] = q_xyzw[1];
  q_wxyz[3] = q_xyzw[2];
}

static double stm_controller_quat_error_norm_wxyz(const double target_wxyz[4],
                                                  const double current_wxyz[4]) {
  double target_xyzw[4];
  double current_xyzw[4];
  double current_inv[4];
  double q_err[4];
  stm_controller_quat_wxyz_to_xyzw(target_wxyz, target_xyzw);
  stm_controller_quat_wxyz_to_xyzw(current_wxyz, current_xyzw);
  quat_conjugate(current_xyzw, current_inv);
  quat_mul(target_xyzw, current_inv, q_err);
  quat_normalize(q_err);
  if (q_err[3] < 0.0) {
    q_err[0] = -q_err[0];
    q_err[1] = -q_err[1];
    q_err[2] = -q_err[2];
    q_err[3] = -q_err[3];
  }
  if (q_err[3] > 1.0) {
    q_err[3] = 1.0;
  }
  if (q_err[3] < -1.0) {
    q_err[3] = -1.0;
  }
  return 2.0 * acos(q_err[3]);
}

static void stm_controller_compute_tcp_velocity(
    const control_arm_kinematics_t *kinematics,
    const double qd[ARM_JOINTS],
    double v_tcp[6]) {
  memset(v_tcp, 0, sizeof(double) * 6);
  if (kinematics == NULL || qd == NULL) {
    return;
  }
  for (int r = 0; r < 6; ++r) {
    for (int c = 0; c < ARM_JOINTS; ++c) {
      v_tcp[r] += kinematics->J[r][c] * qd[c];
    }
  }
}

static double stm_controller_path_tangent_speed(const double v_tcp[6],
                                                const double direction[3]) {
  return v_tcp[0] * direction[0] + v_tcp[1] * direction[1] +
         v_tcp[2] * direction[2];
}

static double stm_controller_clamp01(double value) {
  if (value < 0.0) {
    return 0.0;
  }
  if (value > 1.0) {
    return 1.0;
  }
  return value;
}

static double stm_controller_quintic_s(double u) {
  return u * u * u * (10.0 + u * (-15.0 + 6.0 * u));
}

static double stm_controller_quintic_ds_du(double u) {
  double one_minus_u = 1.0 - u;
  return 30.0 * u * u * one_minus_u * one_minus_u;
}

static int stm_controller_target_changed(const CartesianPathState *path,
                                         const double target_pos[3],
                                         const double target_quat[4]) {
  if (!path->initialized) {
    return 1;
  }
  if (stm_controller_vec3_distance(path->target_pos, target_pos) >
      TRAJ_TARGET_CHANGE_POS_EPS_M) {
    return 1;
  }
  if (stm_controller_quat_error_norm_wxyz(target_quat, path->target_quat) >
      TRAJ_TARGET_CHANGE_ORI_EPS_RAD) {
    return 1;
  }
  return 0;
}

static void stm_controller_plan_path_from_current(
    CartesianPathState *path,
    const control_arm_kinematics_t *kinematics,
    const double target_pos[3],
    const double target_quat[4]) {
  double delta[3];

  memset(path, 0, sizeof(*path));
  path->initialized = 1;
  memcpy(path->start_pos, kinematics->pos, sizeof(path->start_pos));
  memcpy(path->start_quat, kinematics->quat_wxyz, sizeof(path->start_quat));
  memcpy(path->target_pos, target_pos, sizeof(path->target_pos));
  memcpy(path->target_quat, target_quat, sizeof(path->target_quat));

  delta[0] = target_pos[0] - path->start_pos[0];
  delta[1] = target_pos[1] - path->start_pos[1];
  delta[2] = target_pos[2] - path->start_pos[2];
  path->path_length = sqrt(delta[0] * delta[0] + delta[1] * delta[1] +
                           delta[2] * delta[2]);

  if (isfinite(path->path_length) && path->path_length > 1e-12) {
    path->direction[0] = delta[0] / path->path_length;
    path->direction[1] = delta[1] / path->path_length;
    path->direction[2] = delta[2] / path->path_length;
  }

  if (path->path_length > END_EFFECTOR_TARGET_POS_TOL_M) {
    double v_peak = END_EFFECTOR_LINEAR_SPEED_MPS;
    if (isfinite(v_peak) && v_peak > 1e-9) {
      path->duration_s = 1.875 * path->path_length / v_peak;
    }
  }

  memcpy(path->ref_pos, path->start_pos, sizeof(path->ref_pos));
  memcpy(path->ref_quat, path->start_quat, sizeof(path->ref_quat));
}

static void stm_controller_update_path_reference(CartesianPathState *path,
                                                 double elapsed_s,
                                                 double ref_twist[6]) {
  double ratio = 1.0;
  memset(ref_twist, 0, sizeof(double) * 6);

  if (!path->initialized) {
    return;
  }

  if (path->path_length > END_EFFECTOR_TARGET_POS_TOL_M) {
    double u;
    double s;
    double ds_dt = 0.0;
    if (path->duration_s <= 0.0 || !isfinite(path->duration_s)) {
      path->duration_s = 0.0;
      path->path_time_s = 0.0;
      path->progress = path->path_length;
      u = 1.0;
    } else {
      if (isfinite(elapsed_s) && elapsed_s > 0.0) {
        path->path_time_s += elapsed_s;
      }
      if (path->path_time_s > path->duration_s) {
        path->path_time_s = path->duration_s;
      }
      u = stm_controller_clamp01(path->path_time_s / path->duration_s);
      ds_dt = stm_controller_quintic_ds_du(u) / path->duration_s;
    }
    s = stm_controller_quintic_s(u);
    ratio = stm_controller_clamp01(s);
    path->progress = path->path_length * ratio;
    for (int i = 0; i < 3; ++i) {
      path->ref_pos[i] = path->start_pos[i] + path->direction[i] * path->progress;
    }
    if (u < 1.0) {
      double speed = path->path_length * ds_dt;
      for (int i = 0; i < 3; ++i) {
        ref_twist[i] = speed * path->direction[i];
      }
    }
  } else {
    path->progress = path->path_length;
    memcpy(path->ref_pos, path->target_pos, sizeof(path->ref_pos));
  }

  double start_xyzw[4], target_xyzw[4], ref_xyzw[4];
  stm_controller_quat_wxyz_to_xyzw(path->start_quat, start_xyzw);
  stm_controller_quat_wxyz_to_xyzw(path->target_quat, target_xyzw);
  quat_slerp(start_xyzw, target_xyzw, ratio, ref_xyzw);
  quat_normalize(ref_xyzw);
  stm_controller_quat_xyzw_to_wxyz(ref_xyzw, path->ref_quat);
}

static int stm_controller_path_arrived(
    CartesianPathState *path,
    const control_arm_kinematics_t *kinematics,
    const double v_tcp[6]) {
  double pos_err =
      stm_controller_vec3_distance(kinematics->pos, path->target_pos);
  double ori_err =
      stm_controller_quat_error_norm_wxyz(path->target_quat,
                                          kinematics->quat_wxyz);
  double speed_abs = fabs(stm_controller_path_tangent_speed(v_tcp,
                                                            path->direction));

  if (pos_err <= END_EFFECTOR_TARGET_POS_TOL_M &&
      ori_err <= END_EFFECTOR_TARGET_ORI_TOL_RAD &&
      speed_abs <= END_EFFECTOR_ARRIVAL_SPEED_TOL_MPS) {
    path->arrived = 1;
    path->progress = path->path_length;
    memcpy(path->ref_pos, path->target_pos, sizeof(path->ref_pos));
    memcpy(path->ref_quat, path->target_quat, sizeof(path->ref_quat));
    return 1;
  }
  return 0;
}

static void stm_controller_enter_safety_latch(int reason, uint8_t active_arm_mask) {
  g_controller.safety_latched = 1;
  g_controller.zero_hold_s = 0.0;
  g_controller.last_safety_reason = reason;
  g_controller.safety_arm_mask = stm_controller_normalize_arm_mask(active_arm_mask);
  stm_controller_clear_path_state();
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
      stm_controller_clear_target_state();
      stm_controller_clear_path_state();
    }
  } else {
    g_controller.zero_hold_s = 0.0;
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

  memcpy(g_controller.target_pos[arm], target_pos, sizeof(double) * 3);
  memcpy(g_controller.target_quat[arm], target_quat, sizeof(double) * 4);
  g_controller.target_valid[arm] = 1;
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
  stm_controller_clear_target_state();
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
  double ref_twist[NUM_ARMS][6];
  double v_tcp[NUM_ARMS][6];
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
      memset(&g_controller.path[arm], 0, sizeof(g_controller.path[arm]));
    }
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
    stm_controller_compute_tcp_velocity(&arm_kinematics[arm],
                                        filtered_qd + offset, v_tcp[arm]);
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
    int offset = arm * ARM_JOINTS;
    CartesianPathState *path = &g_controller.path[arm];
    if (stm_controller_target_changed(path, target_pos[arm],
                                      target_quat[arm])) {
      stm_controller_plan_path_from_current(path, &arm_kinematics[arm],
                                            target_pos[arm],
                                            target_quat[arm]);
    }
    stm_controller_update_path_reference(path, elapsed_s, ref_twist[arm]);
    if (stm_controller_path_arrived(path, &arm_kinematics[arm], v_tcp[arm])) {
      memset(ref_twist[arm], 0, sizeof(ref_twist[arm]));
    }
    control_step_v2_arm_with_reference(arm, path->ref_pos, path->ref_quat,
                                       ref_twist[arm], in->q + offset,
                                       filtered_qd + offset,
                                       &arm_kinematics[arm],
                                       out->tau + offset);
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
