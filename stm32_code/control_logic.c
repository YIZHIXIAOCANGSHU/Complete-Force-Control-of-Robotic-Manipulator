#include "control_logic.h"
#include "config.h"
#include <string.h>

typedef struct {
  RBDLModel rbdl_model;
  KalmanFilter1D vel_filters[ARM_JOINTS];
  int initialized;
} ControlArmContext;

/* 左右臂独立控制上下文。 */
static ControlArmContext g_arm_contexts[NUM_ARMS];

static const double TCP_OFFSETS[NUM_ARMS][3] = {
    {TCP_LEFT_OFFSET_X, TCP_LEFT_OFFSET_Y, TCP_LEFT_OFFSET_Z},
    {TCP_RIGHT_OFFSET_X, TCP_RIGHT_OFFSET_Y, TCP_RIGHT_OFFSET_Z},
};

/* TCP 局部姿态 (xyzw)。TCP +Z 指向机器人前方(当前 URDF 零位 base/world +X)，
 * 同时保持左右末端坐标轴方向统一。 */
static const double TCP_FRAME_QUATS_XYZW[NUM_ARMS][4] = {
    {0.0, 0.7071067811865476, 0.7071067811865476, 0.0},
    {0.7071067811865476, 0.0, 0.0, 0.7071067811865476},
};

/* AM-D02 的关节力矩限制 */
static const double TORQUE_LIMIT[ARM_JOINTS] = {
    JOINT_TORQUE_LIMIT_1, JOINT_TORQUE_LIMIT_2, JOINT_TORQUE_LIMIT_3,
    JOINT_TORQUE_LIMIT_4, JOINT_TORQUE_LIMIT_5, JOINT_TORQUE_LIMIT_6,
    JOINT_TORQUE_LIMIT_7};

static const double CARTESIAN_K[6] = {KP_CART_X,    KP_CART_Y,     KP_CART_Z,
                                      KP_CART_ROLL, KP_CART_PITCH, KP_CART_YAW};
static const double CARTESIAN_D[6] = {KD_CART_X,    KD_CART_Y,     KD_CART_Z,
                                      KD_CART_ROLL, KD_CART_PITCH, KD_CART_YAW};

static const double JOINT_MIN_LEFT[ARM_JOINTS] = {
    JOINT_POS_MIN_1, JOINT_POS_MIN_2, JOINT_POS_MIN_3, JOINT_POS_MIN_4,
    JOINT_POS_MIN_5, JOINT_POS_MIN_6, JOINT_POS_MIN_7};
static const double JOINT_MAX_LEFT[ARM_JOINTS] = {
    JOINT_POS_MAX_1, JOINT_POS_MAX_2, JOINT_POS_MAX_3, JOINT_POS_MAX_4,
    JOINT_POS_MAX_5, JOINT_POS_MAX_6, JOINT_POS_MAX_7};
static const double JOINT_MIN_RIGHT[ARM_JOINTS] = {
    RIGHT_JOINT_POS_MIN_1, RIGHT_JOINT_POS_MIN_2, RIGHT_JOINT_POS_MIN_3,
    RIGHT_JOINT_POS_MIN_4, RIGHT_JOINT_POS_MIN_5, RIGHT_JOINT_POS_MIN_6,
    RIGHT_JOINT_POS_MIN_7};
static const double JOINT_MAX_RIGHT[ARM_JOINTS] = {
    RIGHT_JOINT_POS_MAX_1, RIGHT_JOINT_POS_MAX_2, RIGHT_JOINT_POS_MAX_3,
    RIGHT_JOINT_POS_MAX_4, RIGHT_JOINT_POS_MAX_5, RIGHT_JOINT_POS_MAX_6,
    RIGHT_JOINT_POS_MAX_7};
static const double JOINT_VEL_LEFT[ARM_JOINTS] = {
    JOINT_VEL_LIMIT_1, JOINT_VEL_LIMIT_2, JOINT_VEL_LIMIT_3,
    JOINT_VEL_LIMIT_4, JOINT_VEL_LIMIT_5, JOINT_VEL_LIMIT_6,
    JOINT_VEL_LIMIT_7};
static const double JOINT_VEL_RIGHT[ARM_JOINTS] = {
    RIGHT_JOINT_VEL_LIMIT_1, RIGHT_JOINT_VEL_LIMIT_2,
    RIGHT_JOINT_VEL_LIMIT_3, RIGHT_JOINT_VEL_LIMIT_4,
    RIGHT_JOINT_VEL_LIMIT_5, RIGHT_JOINT_VEL_LIMIT_6,
    RIGHT_JOINT_VEL_LIMIT_7};

static double g_body_gravity[3] = {0.0, 0.0, -9.81};

static void control_init_context(int side);

static int normalize_side(int side) {
  return side == ARM_RIGHT ? ARM_RIGHT : ARM_LEFT;
}

static ControlArmContext *control_get_context(int side) {
  return &g_arm_contexts[normalize_side(side)];
}

static void quat_error_to_axis_angle(const double target_quat_wxyz[4],
                                     const double current_quat_xyzw[4],
                                     double e_rot[3]) {
  double target_xyzw[4] = {target_quat_wxyz[1], target_quat_wxyz[2],
                           target_quat_wxyz[3], target_quat_wxyz[0]};
  double q_cur_inv[4], q_err[4];
  quat_conjugate(current_quat_xyzw, q_cur_inv);
  quat_mul(target_xyzw, q_cur_inv, q_err);

  if (q_err[3] < 0.0) {
    q_err[0] = -q_err[0];
    q_err[1] = -q_err[1];
    q_err[2] = -q_err[2];
    q_err[3] = -q_err[3];
  }

  double s = sqrt(q_err[0] * q_err[0] + q_err[1] * q_err[1] +
                  q_err[2] * q_err[2]);
  if (s < 1e-6) {
    e_rot[0] = 2.0 * q_err[0];
    e_rot[1] = 2.0 * q_err[1];
    e_rot[2] = 2.0 * q_err[2];
  } else {
    double angle = 2.0 * atan2(s, q_err[3]);
    double k = angle / s;
    e_rot[0] = k * q_err[0];
    e_rot[1] = k * q_err[1];
    e_rot[2] = k * q_err[2];
  }
}

static void rotz(double angle, double R[9]) {
  double c = cos(angle);
  double s = sin(angle);
  R[0] = c;
  R[1] = -s;
  R[2] = 0.0;
  R[3] = s;
  R[4] = c;
  R[5] = 0.0;
  R[6] = 0.0;
  R[7] = 0.0;
  R[8] = 1.0;
}

static void mat3_mul_inplace(double A[9], const double B[9]) {
  double C[9];
  mat3_mul(A, B, C);
  memcpy(A, C, sizeof(C));
}

static void control_apply_gravity_to_contexts(const double gravity[3]) {
  for (int arm = 0; arm < NUM_ARMS; ++arm) {
    ControlArmContext *ctx = control_get_context(arm);
    control_init_context(arm);
    vec3_copy(gravity, ctx->rbdl_model.gravity);
  }
}

static void control_compute_body_rotation(const double body_q[NUM_BODY_JOINTS],
                                          double R_base_to_body[9]) {
  double R_origin[9];
  double R_joint[9];

  rpy_to_rotmat(-M_PI / 2.0, 0.0, 0.0, R_base_to_body);
  rotz(body_q[0], R_joint);
  mat3_mul_inplace(R_base_to_body, R_joint);

  rpy_to_rotmat(0.0, 0.0, 0.0, R_origin);
  mat3_mul_inplace(R_base_to_body, R_origin);
  rotz(body_q[1], R_joint);
  mat3_mul_inplace(R_base_to_body, R_joint);

  rpy_to_rotmat(0.0, 0.0, 0.0, R_origin);
  mat3_mul_inplace(R_base_to_body, R_origin);
  rotz(body_q[2], R_joint);
  mat3_mul_inplace(R_base_to_body, R_joint);
}

/* ================================================================
 *  内部辅助函数
 * ================================================================ */

/**
 * 应用 TCP 偏移到末端执行器位置和雅可比矩阵
 */
static void apply_tcp_offset(double pos_ee[3], const double quat_xyzw[4],
                             const double tcp_offset[3],
                             double J_6x7[6][ARM_JOINTS]) {
  double R[9];
  /* 使用 math_lib 中的 mat3 数学函数 */
  double qw = quat_xyzw[3], qx = quat_xyzw[0], qy = quat_xyzw[1],
         qz = quat_xyzw[2];
  R[0] = 1.0 - 2.0 * (qy * qy + qz * qz);
  R[1] = 2.0 * (qx * qy - qz * qw);
  R[2] = 2.0 * (qx * qz + qy * qw);
  R[3] = 2.0 * (qx * qy + qz * qw);
  R[4] = 1.0 - 2.0 * (qx * qx + qz * qz);
  R[5] = 2.0 * (qy * qz - qx * qw);
  R[6] = 2.0 * (qx * qz - qy * qw);
  R[7] = 2.0 * (qy * qz + qx * qw);
  R[8] = 1.0 - 2.0 * (qx * qx + qy * qy);

  double delta[3];
  mat3_mul_vec3(R, tcp_offset, delta);

  pos_ee[0] += delta[0];
  pos_ee[1] += delta[1];
  pos_ee[2] += delta[2];

  if (J_6x7) {
    for (int c = 0; c < ARM_JOINTS; c++) {
      double w[3] = {J_6x7[3][c], J_6x7[4][c], J_6x7[5][c]};
      double cross[3];
      vec3_cross(delta, w, cross);
      J_6x7[0][c] -= cross[0];
      J_6x7[1][c] -= cross[1];
      J_6x7[2][c] -= cross[2];
    }
  }
}

static void apply_tcp_frame_orientation(int side, double quat_xyzw[4]) {
  double rotated[4];
  quat_mul(quat_xyzw, TCP_FRAME_QUATS_XYZW[normalize_side(side)], rotated);
  memcpy(quat_xyzw, rotated, sizeof(rotated));
  quat_normalize(quat_xyzw);
}

static void control_desired_tcp_linear_velocity(const double pos_error[3],
                                                double v_des[3]) {
  double distance = sqrt(pos_error[0] * pos_error[0] +
                         pos_error[1] * pos_error[1] +
                         pos_error[2] * pos_error[2]);
  double speed = END_EFFECTOR_LINEAR_SPEED_MPS;

  v_des[0] = 0.0;
  v_des[1] = 0.0;
  v_des[2] = 0.0;

  if (!isfinite(distance) || distance <= END_EFFECTOR_TARGET_POS_TOL_M ||
      !isfinite(speed) || speed <= 0.0) {
    return;
  }

  v_des[0] = speed * pos_error[0] / distance;
  v_des[1] = speed * pos_error[1] / distance;
  v_des[2] = speed * pos_error[2] / distance;
}

static void control_limit_linear_wrench_by_tcp_speed(const double v_ee[6],
                                                     double F[6]) {
  double speed_limit = END_EFFECTOR_REAL_SPEED_LIMIT_MPS;
  double linear_speed = sqrt(v_ee[0] * v_ee[0] + v_ee[1] * v_ee[1] +
                             v_ee[2] * v_ee[2]);
  double dir[3];

  if (!isfinite(speed_limit) || speed_limit <= 0.0 ||
      !isfinite(linear_speed) || linear_speed <= speed_limit ||
      linear_speed <= 1e-12) {
    return;
  }

  dir[0] = v_ee[0] / linear_speed;
  dir[1] = v_ee[1] / linear_speed;
  dir[2] = v_ee[2] / linear_speed;

  double accelerating_force = F[0] * dir[0] + F[1] * dir[1] + F[2] * dir[2];
  if (isfinite(accelerating_force) && accelerating_force > 0.0) {
    for (int i = 0; i < 3; ++i) {
      F[i] -= accelerating_force * dir[i];
    }
  }

  double brake_damping =
      (CARTESIAN_D[0] + CARTESIAN_D[1] + CARTESIAN_D[2]) / 3.0;
  if (isfinite(brake_damping) && brake_damping > 0.0) {
    double brake_force = brake_damping * (linear_speed - speed_limit);
    for (int i = 0; i < 3; ++i) {
      F[i] -= brake_force * dir[i];
    }
  }
}

/* ================================================================
 *  公开 API
 * ================================================================ */

static void control_init_context(int side) {
  ControlArmContext *ctx = control_get_context(side);
  if (ctx->initialized)
    return;
  build_am_d02_arm_model(normalize_side(side), &ctx->rbdl_model);
  
  for (int i = 0; i < ARM_JOINTS; i++) {
    kalman_filter1d_init(&ctx->vel_filters[i], KALMAN_Q_VEL, KALMAN_R_VEL);
  }

  ctx->initialized = 1;
}

/* 初始化控制系统 */
void control_init(void) {
  control_init_context(ARM_LEFT);
  control_init_context(ARM_RIGHT);
  control_apply_gravity_to_contexts(g_body_gravity);
}

void control_update_body_gravity(const double body_q[NUM_BODY_JOINTS]) {
  static const double gravity_world[3] = {0.0, 0.0, -9.81};
  static const double zero_body_q[NUM_BODY_JOINTS] = {0.0, 0.0, 0.0};
  double R_base_to_body_zero[9];
  double R_base_to_body_current[9];
  double R_body_zero_to_base[9];
  double R_zero_to_current[9];
  double R_current_to_zero[9];

  if (body_q == NULL) {
    return;
  }

  control_compute_body_rotation(zero_body_q, R_base_to_body_zero);
  control_compute_body_rotation(body_q, R_base_to_body_current);
  mat3_transpose(R_base_to_body_zero, R_body_zero_to_base);
  mat3_mul(R_base_to_body_current, R_body_zero_to_base, R_zero_to_current);
  mat3_transpose(R_zero_to_current, R_current_to_zero);
  mat3_mul_vec3(R_current_to_zero, gravity_world, g_body_gravity);
  control_apply_gravity_to_contexts(g_body_gravity);
}

void control_get_arm_kinematics_with_offset(
    int side, const double q[ARM_JOINTS],
    control_arm_kinematics_t *kinematics) {
  ControlArmContext *ctx = control_get_context(side);
  double J_1D[42];
  int arm_side = normalize_side(side);

  if (kinematics == NULL) {
    return;
  }

  control_init_context(side);
  rbdl_forward_kinematics(&ctx->rbdl_model, q, kinematics->pos,
                          kinematics->quat_xyzw);
  rbdl_calc_jacobian(&ctx->rbdl_model, q, J_1D);

  for (int r = 0; r < 6; r++) {
    for (int c = 0; c < ARM_JOINTS; c++) {
      kinematics->J[r][c] = J_1D[r + 6 * c];
    }
  }

  apply_tcp_offset(kinematics->pos, kinematics->quat_xyzw,
                   TCP_OFFSETS[arm_side], kinematics->J);
  apply_tcp_frame_orientation(arm_side, kinematics->quat_xyzw);
  /* 转换为 Python/MuJoCo 的顺序 (w, x, y, z) */
  kinematics->quat_wxyz[0] = kinematics->quat_xyzw[3];
  kinematics->quat_wxyz[1] = kinematics->quat_xyzw[0];
  kinematics->quat_wxyz[2] = kinematics->quat_xyzw[1];
  kinematics->quat_wxyz[3] = kinematics->quat_xyzw[2];
}

/* ================================================================
 *  核心控制逻辑: 笛卡尔阻抗 + g+c
 * ================================================================ */
void control_step_v2_arm_with_reference(
    int side, const double ref_pos[3], const double ref_quat[4],
    const double ref_twist[6],
    const double current_q[ARM_JOINTS],
    const double current_qd[ARM_JOINTS],
    const control_arm_kinematics_t *kinematics,
    double tau_out[ARM_JOINTS]) {
  int arm_side = normalize_side(side);
  ControlArmContext *ctx = control_get_context(arm_side);

  if (kinematics == NULL) {
    control_arm_kinematics_t local_kinematics;
    control_get_arm_kinematics_with_offset(arm_side, current_q,
                                           &local_kinematics);
    control_step_v2_arm_with_reference(arm_side, ref_pos, ref_quat, ref_twist,
                                       current_q, current_qd,
                                       &local_kinematics, tau_out);
    return;
  }

  control_init_context(arm_side);

  /* 1. 计算末端位置与姿态误差 (与 Python quaternion_conjugate 和 multiply 对齐)
   */
  double e6[6], v_ee[6] = {0};

  // 位置误差
  e6[0] = ref_pos[0] - kinematics->pos[0];
  e6[1] = ref_pos[1] - kinematics->pos[1];
  e6[2] = ref_pos[2] - kinematics->pos[2];

  // 姿态误差使用轴角小量，与 Python 侧误差约定对齐。
  quat_error_to_axis_angle(ref_quat, kinematics->quat_xyzw, &e6[3]);

  /* 2. 计算末端速度: v_ee = J * current_qd */
  for (int i = 0; i < 6; i++) {
    for (int j = 0; j < ARM_JOINTS; j++) {
      v_ee[i] += kinematics->J[i][j] * current_qd[j];
    }
  }

  /* 3. 笛卡尔空间主任务力。平移轴跟踪期望 TCP 线速度；
   * 姿态轴本轮通常使用零期望角速度阻尼。若估计 TCP 线速度超过真机限速，
   * 去除继续沿当前速度方向加速的线性力，并追加反向制动。
   */
  double F[6] = {0};
  for (int j = 0; j < 6; j++) {
    double v_ref = ref_twist != NULL ? ref_twist[j] : 0.0;
    F[j] = CARTESIAN_K[j] * e6[j] + CARTESIAN_D[j] * (v_ref - v_ee[j]);
  }
  control_limit_linear_wrench_by_tcp_speed(v_ee, F);

  double tau_task[ARM_JOINTS] = {0};
  for (int i = 0; i < ARM_JOINTS; i++) {
    for (int j = 0; j < 6; j++) {
      tau_task[i] += kinematics->J[j][i] * F[j];
    }
  }

  /* 4. 动力学补偿: 包含科氏力(Coriolis)和重力(Gravity) */
  double tau_gc[ARM_JOINTS];
  rbdl_calc_gc(&ctx->rbdl_model, current_q, current_qd, tau_gc);

  /* 5. 融合总力矩并限制输出 */
  for (int i = 0; i < ARM_JOINTS; i++) {
    tau_out[i] = tau_task[i] + tau_gc[i];

    /* 施加关节力矩饱和限位 */
    if (tau_out[i] > TORQUE_LIMIT[i])
      tau_out[i] = TORQUE_LIMIT[i];
    if (tau_out[i] < -TORQUE_LIMIT[i])
      tau_out[i] = -TORQUE_LIMIT[i];
  }
}

void control_step_v2_arm_with_state(
    int side, const double target_pos[3], const double target_quat[4],
    const double current_q[ARM_JOINTS],
    const double current_qd[ARM_JOINTS],
    const control_arm_kinematics_t *kinematics,
    double tau_out[ARM_JOINTS]) {
  double ref_twist[6] = {0.0};
  double pos_error[3] = {0.0};

  if (kinematics == NULL) {
    control_arm_kinematics_t local_kinematics;
    control_get_arm_kinematics_with_offset(side, current_q,
                                           &local_kinematics);
    control_step_v2_arm_with_state(side, target_pos, target_quat, current_q,
                                   current_qd, &local_kinematics, tau_out);
    return;
  }

  if (kinematics != NULL) {
    pos_error[0] = target_pos[0] - kinematics->pos[0];
    pos_error[1] = target_pos[1] - kinematics->pos[1];
    pos_error[2] = target_pos[2] - kinematics->pos[2];
    control_desired_tcp_linear_velocity(pos_error, ref_twist);
  }

  control_step_v2_arm_with_reference(side, target_pos, target_quat, ref_twist,
                                     current_q, current_qd, kinematics,
                                     tau_out);
}

void control_step_v2_dual_with_state(
    const double target_pos[NUM_ARMS][3],
    const double target_quat[NUM_ARMS][4],
    const double current_q[NUM_JOINTS],
    const double current_qd[NUM_JOINTS],
    const control_arm_kinematics_t kinematics[NUM_ARMS],
    double tau_out[NUM_JOINTS]) {
  for (int arm = 0; arm < NUM_ARMS; ++arm) {
    int offset = arm * ARM_JOINTS;
    control_step_v2_arm_with_state(arm, target_pos[arm], target_quat[arm],
                                   current_q + offset, current_qd + offset,
                                   &kinematics[arm], tau_out + offset);
  }
}

int control_check_safety_arm(int side, const double q[ARM_JOINTS],
                             const double qd[ARM_JOINTS]) {
  int arm_side = normalize_side(side);
  const double *joint_min =
      arm_side == ARM_RIGHT ? JOINT_MIN_RIGHT : JOINT_MIN_LEFT;
  const double *joint_max =
      arm_side == ARM_RIGHT ? JOINT_MAX_RIGHT : JOINT_MAX_LEFT;
  const double *joint_vel =
      arm_side == ARM_RIGHT ? JOINT_VEL_RIGHT : JOINT_VEL_LEFT;

  for (int i = 0; i < ARM_JOINTS; i++) {
    double span = joint_max[i] - joint_min[i];
    double inset = span * CONTROL_JOINT_LIMIT_INSET_RATIO;
    double safe_min = joint_min[i] + inset;
    double safe_max = joint_max[i] - inset;

    /* 位置检查：使用内收后的安全限位。 */
    if (q[i] < safe_min) {
      STM_LOG_ERROR("[SAFETY] Arm%d J%d position low: %.4f < %.4f\n",
                    arm_side, i + 1, q[i], safe_min);
      return -1;
    }
    if (q[i] > safe_max) {
      STM_LOG_ERROR("[SAFETY] Arm%d J%d position high: %.4f > %.4f\n",
                    arm_side, i + 1, q[i], safe_max);
      return -1;
    }
    /* 速度检查 */
    if (qd[i] > joint_vel[i] || qd[i] < -joint_vel[i]) {
      STM_LOG_ERROR("[SAFETY] Arm%d J%d velocity error: %.4f (limit: %.4f)\n",
                    arm_side, i + 1, qd[i], joint_vel[i]);
      return -2;
    }
  }
  return 0;
}

void control_filter_velocities_arm(int side, const double qd_raw[ARM_JOINTS],
                                   double qd_filtered[ARM_JOINTS]) {
  ControlArmContext *ctx = control_get_context(side);
  control_init_context(side);
  for (int i = 0; i < ARM_JOINTS; i++) {
    qd_filtered[i] = kalman_filter1d_update(&ctx->vel_filters[i], qd_raw[i]);
  }
}

void control_compute_gravity_torque_arm(int side, const double q[ARM_JOINTS],
                                        double tau_g[ARM_JOINTS]) {
  int arm_side = normalize_side(side);
  ControlArmContext *ctx = control_get_context(arm_side);
  control_init_context(arm_side);
  rbdl_calc_gravity(&ctx->rbdl_model, q, tau_g);
}

void control_compute_gc_torque_arm(int side, const double q[ARM_JOINTS],
                                   const double qd[ARM_JOINTS],
                                   double tau_gc[ARM_JOINTS]) {
  int arm_side = normalize_side(side);
  ControlArmContext *ctx = control_get_context(arm_side);
  control_init_context(arm_side);
  rbdl_calc_gc(&ctx->rbdl_model, q, qd, tau_gc);
}
