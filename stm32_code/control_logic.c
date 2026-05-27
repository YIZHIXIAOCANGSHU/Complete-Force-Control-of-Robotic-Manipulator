#include "control_logic.h"
#include "config.h"
#include <stdio.h>
#include <string.h>

typedef struct {
  KinematicsSolver kin_solver;
  RBDLModel rbdl_model;
  KalmanFilter1D vel_filters[ARM_JOINTS];
  int initialized;
} ControlArmContext;

/* 左右臂独立控制上下文。单臂兼容接口默认使用左臂。 */
static ControlArmContext g_arm_contexts[NUM_ARMS];

static const double TCP_OFFSETS[NUM_ARMS][3] = {
    {TCP_LEFT_OFFSET_X, TCP_LEFT_OFFSET_Y, TCP_LEFT_OFFSET_Z},
    {TCP_RIGHT_OFFSET_X, TCP_RIGHT_OFFSET_Y, TCP_RIGHT_OFFSET_Z},
};

/* TCP 局部姿态 (xyzw)。左臂绕本地 Z 轴转 180 度，使左右 TCP 坐标轴方向统一。 */
static const double TCP_FRAME_QUATS_XYZW[NUM_ARMS][4] = {
    {0.0, 0.0, 1.0, 0.0},
    {0.0, 0.0, 0.0, 1.0},
};

/* AM-D02 的关节力矩限制 */
static const double TORQUE_LIMIT[ARM_JOINTS] = {
    JOINT_TORQUE_LIMIT_1, JOINT_TORQUE_LIMIT_2, JOINT_TORQUE_LIMIT_3,
    JOINT_TORQUE_LIMIT_4, JOINT_TORQUE_LIMIT_5, JOINT_TORQUE_LIMIT_6,
    JOINT_TORQUE_LIMIT_7};

static double g_body_gravity[3] = {0.0, 0.0, -9.81};

static void control_init_context(int side);

static int normalize_side(int side) {
  return side == ARM_RIGHT ? ARM_RIGHT : ARM_LEFT;
}

static ControlArmContext *control_get_context(int side) {
  return &g_arm_contexts[normalize_side(side)];
}

static double clamp_scalar(double value, double limit) {
  if (value > limit)
    return limit;
  if (value < -limit)
    return -limit;
  return value;
}

static double ramp_from_deadband(double value, double deadband,
                                 double full_scale) {
  if (value <= deadband)
    return 0.0;
  if (value >= full_scale)
    return 1.0;
  return (value - deadband) / (full_scale - deadband);
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

#if 0
/* 姿态规划的滤波器状态 */
static double g_q_ref_filtered[7] = {0};
static int g_planner_first_call = 1;
#endif

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

#if 0
/**
 * 姿态规划器计算：根据目标位姿和首选姿态计算参考关节角
 */
static void posture_planner_compute(const Pose *target_pose,
                                    const double current_q[7],
                                    const double q_preferred[7], double alpha,
                                    double q_ref_out[7]) {
  double q_init[7];
  for (int i = 0; i < 7; i++) {
    q_init[i] = current_q[i] + alpha * (q_preferred[i] - current_q[i]);
  }

  double q_result[7];
  uint8_t ok = kinematics_compute_inverse_pose_dls(&g_kin_solver, target_pose,
                                                   q_init, q_result, 30);

  if (!ok) {
    memcpy(q_ref_out, current_q, sizeof(double) * 7);
    return;
  }

  double smooth = 0.5;
  if (g_planner_first_call) {
    memcpy(g_q_ref_filtered, current_q, sizeof(double) * 7);
    g_planner_first_call = 0;
  }

  for (int i = 0; i < 7; i++) {
    g_q_ref_filtered[i] += smooth * (q_result[i] - g_q_ref_filtered[i]);
    q_ref_out[i] = g_q_ref_filtered[i];
  }
}
#endif

/* ================================================================
 *  公开 API
 * ================================================================ */

static void control_init_context(int side) {
  ControlArmContext *ctx = control_get_context(side);
  if (ctx->initialized)
    return;
  kinematics_init(&ctx->kin_solver);
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

void control_init_arm(int side) {
  control_init_context(side);
  vec3_copy(g_body_gravity, control_get_context(side)->rbdl_model.gravity);
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

void control_get_body_gravity(double gravity_out[3]) {
  if (gravity_out == NULL) {
    return;
  }
  vec3_copy(g_body_gravity, gravity_out);
}

/* 计算重力补偿 */
void control_calc_gravity_compensation_arm(int side,
                                           const double q[ARM_JOINTS],
                                           double G[ARM_JOINTS]) {
  ControlArmContext *ctx = control_get_context(side);
  control_init_context(side);
  rbdl_calc_gravity(&ctx->rbdl_model, q, G);
}

void control_calc_gravity_compensation(const double q[ARM_JOINTS],
                                       double G[ARM_JOINTS]) {
  control_calc_gravity_compensation_arm(ARM_LEFT, q, G);
}

/* 计算重力补偿 + 关节 PD */
void control_calc_gravity_pd_compensation(const double q[ARM_JOINTS],
                                          const double qd[ARM_JOINTS],
                                          const double q_target[ARM_JOINTS],
                                          double tau_out[ARM_JOINTS]) {
  ControlArmContext *ctx = control_get_context(ARM_LEFT);
  control_init_context(ARM_LEFT);

  double G[ARM_JOINTS];
  rbdl_calc_gravity(&ctx->rbdl_model, q, G);

  const double joint_kp[ARM_JOINTS] = {
      KP_JOINT_1, KP_JOINT_2, KP_JOINT_3, KP_JOINT_4,
      KP_JOINT_5, KP_JOINT_6, KP_JOINT_7};
  const double joint_kd[ARM_JOINTS] = {
      KD_JOINT_1, KD_JOINT_2, KD_JOINT_3, KD_JOINT_4,
      KD_JOINT_5, KD_JOINT_6, KD_JOINT_7};

  for (int i = 0; i < ARM_JOINTS; i++) {
    double eq = normalize_angle(q_target[i] - q[i]);
    tau_out[i] = G[i] + joint_kp[i] * eq - joint_kd[i] * qd[i];

    /* 施加关节力矩饱和限位 */
    if (tau_out[i] > TORQUE_LIMIT[i])
      tau_out[i] = TORQUE_LIMIT[i];
    if (tau_out[i] < -TORQUE_LIMIT[i])
      tau_out[i] = -TORQUE_LIMIT[i];
  }
}

/* 计算 重力补偿 + 关节 PD + 笛卡尔 PD */
void control_calc_cartesian_joint_pd_compensation(
    const double q[ARM_JOINTS], const double qd[ARM_JOINTS],
    const double q_target[ARM_JOINTS],
    const double target_pos[3], const double target_quat[4],
    double tau_out[ARM_JOINTS]) {
  ControlArmContext *ctx = control_get_context(ARM_LEFT);
  control_init_context(ARM_LEFT);

  // 1. 重力补偿
  double G[ARM_JOINTS];
  rbdl_calc_gravity(&ctx->rbdl_model, q, G);

  // 2. 关节空间 PD 增益与误差
  const double joint_kp[ARM_JOINTS] = {
      KP_JOINT_1, KP_JOINT_2, KP_JOINT_3, KP_JOINT_4,
      KP_JOINT_5, KP_JOINT_6, KP_JOINT_7};
  const double joint_kd[ARM_JOINTS] = {
      KD_JOINT_1, KD_JOINT_2, KD_JOINT_3, KD_JOINT_4,
      KD_JOINT_5, KD_JOINT_6, KD_JOINT_7};

  double tau_joint[ARM_JOINTS] = {0};
  for (int i = 0; i < ARM_JOINTS; i++) {
    double eq = normalize_angle(q_target[i] - q[i]);
    tau_joint[i] = joint_kp[i] * eq - joint_kd[i] * qd[i];
  }

  // 3. 笛卡尔空间 PD (参考 control_step_v2 的误差计算逻辑)
  const double cartesian_K[6] = {KP_CART_X,    KP_CART_Y,     KP_CART_Z,
                                 KP_CART_ROLL, KP_CART_PITCH, KP_CART_YAW};
  const double cartesian_D[6] = {KD_CART_X,    KD_CART_Y,     KD_CART_Z,
                                 KD_CART_ROLL, KD_CART_PITCH, KD_CART_YAW};

  double pos_ee[3], quat_xyzw[4], J_1D[42];
  rbdl_forward_kinematics(&ctx->rbdl_model, q, pos_ee, quat_xyzw);
  rbdl_calc_jacobian(&ctx->rbdl_model, q, J_1D);

  double J[6][ARM_JOINTS];
  for (int r = 0; r < 6; r++)
    for (int c = 0; c < ARM_JOINTS; c++)
      J[r][c] = J_1D[r + 6 * c];
  apply_tcp_offset(pos_ee, quat_xyzw, TCP_OFFSETS[ARM_LEFT], J);
  apply_tcp_frame_orientation(ARM_LEFT, quat_xyzw);

  double e6[6], v_ee[6] = {0};
  // A. 位置误差
  e6[0] = target_pos[0] - pos_ee[0];
  e6[1] = target_pos[1] - pos_ee[1];
  e6[2] = target_pos[2] - pos_ee[2];

  // B. 姿态误差 (轴角小量)
  quat_error_to_axis_angle(target_quat, quat_xyzw, &e6[3]);

  // C. 末端速度 v_ee = J * qd
  for (int i = 0; i < 6; i++) {
    for (int j = 0; j < ARM_JOINTS; j++) {
      v_ee[i] += J[i][j] * qd[j];
    }
  }

  // D. 笛卡尔力矩 tau_cart = J^T * (K*e - D*v)
  double tau_cart[ARM_JOINTS] = {0};
  for (int i = 0; i < ARM_JOINTS; i++) {
    for (int j = 0; j < 6; j++) {
      double F_j = cartesian_K[j] * e6[j] - cartesian_D[j] * v_ee[j];
      tau_cart[i] += J[j][i] * F_j;
    }
  }

  // 4. 总力矩融合: Tau = G + Tau_joint + Tau_cart
  for (int i = 0; i < ARM_JOINTS; i++) {
    tau_out[i] = G[i] + tau_joint[i] + tau_cart[i];

    /* 施加关节力矩饱和限位 */
    if (tau_out[i] > TORQUE_LIMIT[i])
      tau_out[i] = TORQUE_LIMIT[i];
    if (tau_out[i] < -TORQUE_LIMIT[i])
      tau_out[i] = -TORQUE_LIMIT[i];
  }
}

/* 计算离心力/科氏力补偿 */
void control_calc_coriolis_compensation_arm(int side,
                                            const double q[ARM_JOINTS],
                                            const double qd[ARM_JOINTS],
                                            double tau_c[ARM_JOINTS]) {
  ControlArmContext *ctx = control_get_context(side);
  control_init_context(side);
  double tau_gc[ARM_JOINTS];
  double G[ARM_JOINTS];
  rbdl_calc_gc(&ctx->rbdl_model, q, qd, tau_gc);
  rbdl_calc_gravity(&ctx->rbdl_model, q, G);
  for (int i = 0; i < ARM_JOINTS; i++)
    tau_c[i] = tau_gc[i] - G[i];
}

void control_calc_coriolis_compensation(const double q[ARM_JOINTS],
                                        const double qd[ARM_JOINTS],
                                        double tau_c[ARM_JOINTS]) {
  control_calc_coriolis_compensation_arm(ARM_LEFT, q, qd, tau_c);
}

/* 获取带有 TCP 偏移的正运动学位姿 */
void control_get_fk_with_offset_arm(int side, const double q[ARM_JOINTS],
                                    double pos[3], double quat[4]) {
  ControlArmContext *ctx = control_get_context(side);
  control_init_context(side);
  double quat_xyzw[4];
  int arm_side = normalize_side(side);
  rbdl_forward_kinematics(&ctx->rbdl_model, q, pos, quat_xyzw);
  apply_tcp_offset(pos, quat_xyzw, TCP_OFFSETS[arm_side], NULL);
  apply_tcp_frame_orientation(arm_side, quat_xyzw);
  /* 转换为 Python/MuJoCo 的顺序 (w, x, y, z) */
  quat[0] = quat_xyzw[3];
  quat[1] = quat_xyzw[0];
  quat[2] = quat_xyzw[1];
  quat[3] = quat_xyzw[2];
}

void control_get_fk_with_offset(const double q[ARM_JOINTS], double pos[3],
                                double quat[4]) {
  control_get_fk_with_offset_arm(ARM_LEFT, q, pos, quat);
}

/* ================================================================
 *  核心控制逻辑: 双空间阻抗控制 (control_step_v2)
 * ================================================================
 * 函数使用示例 包含默认的值
 *
 *
 * 函数输入的内容
 * target_pos: 目标末端位置 (Body0422 动态目标坐标系) 格式为[x,y,z]
 * target_quat: 目标末端姿态，坐标轴方向保持初始 URDF/base 方向，格式为[w,x,y,z]
 * current_q: 当前关节角度 格式为[q1,q2,q3,q4,q5,q6,q7]
 * current_qd: 当前关节速度
 * 格式为[qd1,qd2,qd3,qd4,qd5,qd6,qd7] cartesian_K: 笛卡尔空间位置/姿态比例增益
 * 格式为[kx,ky,kz,kroll,kpitch,kyaw] cartesian_D: 笛卡尔空间位置/姿态微分增益
 * 格式为[dx,dy,dz,droll,dpitch,dyaw] joint_kp: 关节空间位置比例增益
 * 格式为[kp1,kp2,kp3,kp4,kp5,kp6,kp7] joint_kd: 关节空间位置微分增益
 * 格式为[kd1,kd2,kd3,kd4,kd5,kd6,kd7] q_preferred: 首选关节姿态
 * 格式为[q1,q2,q3,q4,q5,q6,q7] posture_alpha: 首选姿态权重 格式为[alpha]
 * w_cart: 笛卡尔空间权重 格式为[w_cart]
 * w_joint: 关节空间权重 格式为[w_joint]
 *
 *输出
 * tau_out: 输出关节力矩 格式为[tau1,tau2,tau3,tau4,tau5,tau6,tau7]
 * ================================================================ */
void control_step_v2_arm(int side, const double target_pos[3],
                         const double target_quat[4],
                         const double current_q[ARM_JOINTS],
                         const double current_qd[ARM_JOINTS],
                         double tau_out[ARM_JOINTS]) {

  const double cartesian_K[6] = {KP_CART_X,    KP_CART_Y,     KP_CART_Z,
                                 KP_CART_ROLL, KP_CART_PITCH, KP_CART_YAW};
  const double cartesian_D[6] = {KD_CART_X,    KD_CART_Y,     KD_CART_Z,
                                 KD_CART_ROLL, KD_CART_PITCH, KD_CART_YAW};
  const double joint_kp[ARM_JOINTS] = {
      KP_JOINT_1, KP_JOINT_2, KP_JOINT_3, KP_JOINT_4,
      KP_JOINT_5, KP_JOINT_6, KP_JOINT_7};
  const double joint_kd[ARM_JOINTS] = {
      KD_JOINT_1, KD_JOINT_2, KD_JOINT_3, KD_JOINT_4,
      KD_JOINT_5, KD_JOINT_6, KD_JOINT_7};
  const double q_preferred[ARM_JOINTS] = {
      Q_PREF_1, Q_PREF_2, Q_PREF_3, Q_PREF_4,
      Q_PREF_5, Q_PREF_6, Q_PREF_7};
  double posture_alpha = POSTURE_ALPHA;
  double w_cart = W_CARTESIAN;
  double w_joint = W_JOINT;
  int arm_side = normalize_side(side);
  ControlArmContext *ctx = control_get_context(arm_side);

  control_init_context(arm_side);

  double pos_ee[3], quat_xyzw[4], J_1D[42];
  rbdl_forward_kinematics(&ctx->rbdl_model, current_q, pos_ee, quat_xyzw);
  rbdl_calc_jacobian(&ctx->rbdl_model, current_q, J_1D);

  double J[6][ARM_JOINTS];
  for (int r = 0; r < 6; r++)
    for (int c = 0; c < ARM_JOINTS; c++)
      J[r][c] = J_1D[r + 6 * c];
  apply_tcp_offset(pos_ee, quat_xyzw, TCP_OFFSETS[arm_side], J);
  apply_tcp_frame_orientation(arm_side, quat_xyzw);

  /* 1. 计算末端位置与姿态误差 (与 Python quaternion_conjugate 和 multiply 对齐)
   */
  double e6[6], v_ee[6] = {0};

  // 位置误差
  e6[0] = target_pos[0] - pos_ee[0];
  e6[1] = target_pos[1] - pos_ee[1];
  e6[2] = target_pos[2] - pos_ee[2];

  // 姿态误差使用轴角小量，与 pose_error_6d 对齐。
  quat_error_to_axis_angle(target_quat, quat_xyzw, &e6[3]);

  /* 2. 计算末端速度: v_ee = J * current_qd */
  for (int i = 0; i < 6; i++) {
    for (int j = 0; j < ARM_JOINTS; j++) {
      v_ee[i] += J[i][j] * current_qd[j];
    }
  }

  /* 3. 笛卡尔空间主任务力矩: tau_task = J^T * (K * e6 - D * v_ee) */
  double tau_task[ARM_JOINTS] = {0};
  for (int i = 0; i < ARM_JOINTS; i++) {
    for (int j = 0; j < 6; j++) {
      double F_j = cartesian_K[j] * e6[j] - cartesian_D[j] * v_ee[j];
      tau_task[i] += J[j][i] * F_j;
    }
  }

  /* 4. DLS 阻尼最小二乘法求 Jacobian 伪逆 */
  double A[36] = {
      0}; // A = J * J^T (6x6 矩阵, 列主序 / 行主序在此对称矩阵中无关紧要)
  for (int r = 0; r < 6; r++) {
    for (int c = 0; c < 6; c++) {
      for (int k = 0; k < ARM_JOINTS; k++) {
        A[r * 6 + c] += J[r][k] * J[c][k];
      }
    }
  }
  double dls_lambda = 0.05; // 阻尼因子
  for (int i = 0; i < 6; i++) {
    A[i * 6 + i] += dls_lambda * dls_lambda;
  }

  double invA[36] = {0};
  if (!mat6_inverse(A, invA)) {
    // 若奇异(理论上因lambda存在不会发生)，使用零矩阵降级
    memset(invA, 0, sizeof(invA));
  }

  // J_pinv (7x6 矩阵) = J^T * invA
  double J_pinv[ARM_JOINTS][6] = {0};
  for (int r = 0; r < ARM_JOINTS; r++) {   // r 是 7 个关节
    for (int c = 0; c < 6; c++) { // c 是 6 个空间维度
      for (int k = 0; k < 6; k++) {
        J_pinv[r][c] += J[k][r] * invA[k * 6 + c];
      }
    }
  }

  /* 5. 底层运动学姿态小步长偏好规划 (dq = J_pinv * e6) */
  double dq_ik[ARM_JOINTS] = {0};
  for (int i = 0; i < ARM_JOINTS; i++) {
    for (int j = 0; j < 6; j++) {
      dq_ik[i] += J_pinv[i][j] * e6[j];
    }
  }

  double q_f[ARM_JOINTS], q_ref[ARM_JOINTS];
  for (int i = 0; i < ARM_JOINTS; i++) {
    q_f[i] = current_q[i] + dq_ik[i];
    q_ref[i] = (1.0 - posture_alpha) * q_f[i] + posture_alpha * q_preferred[i];
  }

  /* 6. 关节空间力矩（仅作为零空间辅助力） */
  double tau_null_control[ARM_JOINTS] = {0};
  for (int i = 0; i < ARM_JOINTS; i++) {
    double eq = normalize_angle(q_ref[i] - current_q[i]);
    tau_null_control[i] = joint_kp[i] * eq - joint_kd[i] * current_qd[i];
  }

  /* 7. 零空间投影 N = I - J_pinv * J */
  double N[ARM_JOINTS][ARM_JOINTS] = {0};
  for (int i = 0; i < ARM_JOINTS; i++) {
    N[i][i] = 1.0;
    for (int j = 0; j < ARM_JOINTS; j++) {
      for (int k = 0; k < 6; k++) {
        N[i][j] -= J_pinv[i][k] * J[k][j];
      }
    }
  }

  // tau_null_projected = N^T * tau_null_control
  double tau_null_projected[ARM_JOINTS] = {0};
  for (int i = 0; i < ARM_JOINTS; i++) {
    for (int j = 0; j < ARM_JOINTS; j++) {
      tau_null_projected[i] += N[j][i] * tau_null_control[j]; // N[j][i] is N.T
    }
  }

  double pos_err_norm = sqrt(e6[0] * e6[0] + e6[1] * e6[1] + e6[2] * e6[2]);
  double ori_err_norm = sqrt(e6[3] * e6[3] + e6[4] * e6[4] + e6[5] * e6[5]);
  double null_scale =
      ramp_from_deadband(pos_err_norm, NULLSPACE_POS_DEADBAND,
                         NULLSPACE_POS_FULL_SCALE);
  double ori_null_scale =
      ramp_from_deadband(ori_err_norm, NULLSPACE_ORI_DEADBAND,
                         NULLSPACE_ORI_FULL_SCALE);
  if (ori_null_scale > null_scale)
    null_scale = ori_null_scale;
  for (int i = 0; i < ARM_JOINTS; i++) {
    tau_null_projected[i] =
        clamp_scalar(null_scale * tau_null_projected[i], NULLSPACE_TORQUE_LIMIT);
  }

  /* 8. 动力学补偿: 包含科氏力(Coriolis)和重力(Gravity) */
  double tau_gc[ARM_JOINTS];
  rbdl_calc_gc(&ctx->rbdl_model, current_q, current_qd, tau_gc);

  /* 9. 融合总力矩并限制输出 */
  for (int i = 0; i < ARM_JOINTS; i++) {
    tau_out[i] =
        w_cart * tau_task[i] + w_joint * tau_null_projected[i] + tau_gc[i];

    /* 施加关节力矩饱和限位 */
    if (tau_out[i] > TORQUE_LIMIT[i])
      tau_out[i] = TORQUE_LIMIT[i];
    if (tau_out[i] < -TORQUE_LIMIT[i])
      tau_out[i] = -TORQUE_LIMIT[i];
  }
}

void control_step_v2(const double target_pos[3], const double target_quat[4],
                     const double current_q[ARM_JOINTS],
                     const double current_qd[ARM_JOINTS],
                     double tau_out[ARM_JOINTS]) {
  control_step_v2_arm(ARM_LEFT, target_pos, target_quat, current_q, current_qd,
                      tau_out);
}

void control_step_v2_dual(const double target_pos[NUM_ARMS][3],
                          const double target_quat[NUM_ARMS][4],
                          const double current_q[NUM_JOINTS],
                          const double current_qd[NUM_JOINTS],
                          double tau_out[NUM_JOINTS]) {
  for (int arm = 0; arm < NUM_ARMS; ++arm) {
    int offset = arm * ARM_JOINTS;
    control_step_v2_arm(arm, target_pos[arm], target_quat[arm],
                        current_q + offset, current_qd + offset,
                        tau_out + offset);
  }
}

int control_check_safety_arm(int side, const double q[ARM_JOINTS],
                             const double qd[ARM_JOINTS]) {
  const double joint_min_left[ARM_JOINTS] = {
      JOINT_POS_MIN_1, JOINT_POS_MIN_2, JOINT_POS_MIN_3, JOINT_POS_MIN_4,
      JOINT_POS_MIN_5, JOINT_POS_MIN_6, JOINT_POS_MIN_7};
  const double joint_max_left[ARM_JOINTS] = {
      JOINT_POS_MAX_1, JOINT_POS_MAX_2, JOINT_POS_MAX_3, JOINT_POS_MAX_4,
      JOINT_POS_MAX_5, JOINT_POS_MAX_6, JOINT_POS_MAX_7};
  const double joint_min_right[ARM_JOINTS] = {
      -2.405, -0.6605, -1.763, -0.0165, -1.5935, -0.6015, -1.1075};
  const double joint_max_right[ARM_JOINTS] = {
      2.2175, 2.203, 1.594, 2.3235, 1.574, 0.6755, 1.068};
  const double *joint_min =
      normalize_side(side) == ARM_RIGHT ? joint_min_right : joint_min_left;
  const double *joint_max =
      normalize_side(side) == ARM_RIGHT ? joint_max_right : joint_max_left;

  for (int i = 0; i < ARM_JOINTS; i++) {
    /* 位置检查 (允许 0.01 rad 的容差以避免边界抖动) */
    if (q[i] < joint_min[i] - 0.01) {
      fprintf(stderr, "[SAFETY] Arm%d J%d position low: %.4f < %.4f\n",
              normalize_side(side), i + 1, q[i], joint_min[i]);
      return -1;
    }
    if (q[i] > joint_max[i] + 0.01) {
      fprintf(stderr, "[SAFETY] Arm%d J%d position high: %.4f > %.4f\n",
              normalize_side(side), i + 1, q[i], joint_max[i]);
      return -1;
    }
    /* 速度检查 */
    if (qd[i] > JOINT_VEL_LIMIT || qd[i] < -JOINT_VEL_LIMIT) {
      fprintf(stderr, "[SAFETY] Arm%d J%d velocity error: %.4f (limit: %.4f)\n",
              normalize_side(side), i + 1, qd[i], JOINT_VEL_LIMIT);
      return -2;
    }
  }
  return 0;
}

int control_check_safety(const double q[ARM_JOINTS],
                         const double qd[ARM_JOINTS]) {
  return control_check_safety_arm(ARM_LEFT, q, qd);
}

void control_filter_velocities_arm(int side, const double qd_raw[ARM_JOINTS],
                                   double qd_filtered[ARM_JOINTS]) {
  ControlArmContext *ctx = control_get_context(side);
  control_init_context(side);
  for (int i = 0; i < ARM_JOINTS; i++) {
    qd_filtered[i] = kalman_filter1d_update(&ctx->vel_filters[i], qd_raw[i]);
  }
}

void control_filter_velocities(const double qd_raw[ARM_JOINTS],
                               double qd_filtered[ARM_JOINTS]) {
  control_filter_velocities_arm(ARM_LEFT, qd_raw, qd_filtered);
}
