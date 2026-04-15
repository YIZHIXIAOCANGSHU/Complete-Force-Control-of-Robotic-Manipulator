#include "control_logic.h"
#include "config.h"
#include <string.h>

/* 全局运动学求解器和 RBDL 模型实例 */
static KinematicsSolver g_kin_solver;
static RBDLModel g_rbdl_model;
static KalmanFilter1D g_vel_filters[7];
static int g_initialized = 0;

/* TCP 偏移量 (米) */
static const double TCP_OFFSET[3] = {TCP_OFFSET_X, TCP_OFFSET_Y, TCP_OFFSET_Z};

/* AM-D02 的关节力矩限制 */
static const double TORQUE_LIMIT[7] = {
    JOINT_TORQUE_LIMIT_1, JOINT_TORQUE_LIMIT_2, JOINT_TORQUE_LIMIT_3,
    JOINT_TORQUE_LIMIT_4, JOINT_TORQUE_LIMIT_5, JOINT_TORQUE_LIMIT_6,
    JOINT_TORQUE_LIMIT_7};

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
                             double J_6x7[6][7]) {
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
  mat3_mul_vec3(R, TCP_OFFSET, delta);

  pos_ee[0] += delta[0];
  pos_ee[1] += delta[1];
  pos_ee[2] += delta[2];

  if (J_6x7) {
    for (int c = 0; c < 7; c++) {
      double w[3] = {J_6x7[3][c], J_6x7[4][c], J_6x7[5][c]};
      double cross[3];
      vec3_cross(delta, w, cross);
      J_6x7[0][c] -= cross[0];
      J_6x7[1][c] -= cross[1];
      J_6x7[2][c] -= cross[2];
    }
  }
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

/* 初始化控制系统 */
void control_init(void) {
  if (g_initialized)
    return;
  kinematics_init(&g_kin_solver);
  build_am_d02_model(&g_rbdl_model);
  
  for (int i = 0; i < 7; i++) {
    kalman_filter1d_init(&g_vel_filters[i], KALMAN_Q_VEL, KALMAN_R_VEL);
  }

  g_initialized = 1;
}

/* 计算重力补偿 */
void control_calc_gravity_compensation(const double q[7], double G[7]) {
  if (!g_initialized)
    control_init();
  rbdl_calc_gravity(&g_rbdl_model, q, G);
}

/* 计算重力补偿 + 关节 PD */
void control_calc_gravity_pd_compensation(const double q[7], const double qd[7],
                                          const double q_target[7],
                                          double tau_out[7]) {
  if (!g_initialized)
    control_init();

  double G[7];
  rbdl_calc_gravity(&g_rbdl_model, q, G);

  const double joint_kp[7] = {KP_JOINT_1, KP_JOINT_2, KP_JOINT_3, KP_JOINT_4,
                              KP_JOINT_5, KP_JOINT_6, KP_JOINT_7};
  const double joint_kd[7] = {KD_JOINT_1, KD_JOINT_2, KD_JOINT_3, KD_JOINT_4,
                              KD_JOINT_5, KD_JOINT_6, KD_JOINT_7};

  for (int i = 0; i < 7; i++) {
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
    const double q[7], const double qd[7], const double q_target[7],
    const double target_pos[3], const double target_quat[4],
    double tau_out[7]) {
  if (!g_initialized)
    control_init();

  // 1. 重力补偿
  double G[7];
  rbdl_calc_gravity(&g_rbdl_model, q, G);

  // 2. 关节空间 PD 增益与误差
  const double joint_kp[7] = {KP_JOINT_1, KP_JOINT_2, KP_JOINT_3, KP_JOINT_4,
                              KP_JOINT_5, KP_JOINT_6, KP_JOINT_7};
  const double joint_kd[7] = {KD_JOINT_1, KD_JOINT_2, KD_JOINT_3, KD_JOINT_4,
                              KD_JOINT_5, KD_JOINT_6, KD_JOINT_7};

  double tau_joint[7] = {0};
  for (int i = 0; i < 7; i++) {
    double eq = normalize_angle(q_target[i] - q[i]);
    tau_joint[i] = joint_kp[i] * eq - joint_kd[i] * qd[i];
  }

  // 3. 笛卡尔空间 PD (参考 control_step_v2 的误差计算逻辑)
  const double cartesian_K[6] = {KP_CART_X,    KP_CART_Y,     KP_CART_Z,
                                 KP_CART_ROLL, KP_CART_PITCH, KP_CART_YAW};
  const double cartesian_D[6] = {KD_CART_X,    KD_CART_Y,     KD_CART_Z,
                                 KD_CART_ROLL, KD_CART_PITCH, KD_CART_YAW};

  double pos_ee[3], quat_xyzw[4], J_1D[42];
  rbdl_forward_kinematics(&g_rbdl_model, q, pos_ee, quat_xyzw);
  rbdl_calc_jacobian(&g_rbdl_model, q, J_1D);

  double J[6][7];
  for (int r = 0; r < 6; r++)
    for (int c = 0; c < 7; c++)
      J[r][c] = J_1D[r + 6 * c];
  apply_tcp_offset(pos_ee, quat_xyzw, J);

  double e6[6], v_ee[6] = {0};
  // A. 位置误差
  e6[0] = target_pos[0] - pos_ee[0];
  e6[1] = target_pos[1] - pos_ee[1];
  e6[2] = target_pos[2] - pos_ee[2];

  // B. 姿态误差 (q_target * q_current_inv)
  double target_xyzw[4] = {target_quat[1], target_quat[2], target_quat[3],
                           target_quat[0]};
  double q_cur_inv[4], q_err[4];
  quat_conjugate(quat_xyzw, q_cur_inv);
  quat_mul(target_xyzw, q_cur_inv, q_err);
  if (q_err[3] < 0.0) {
    for (int i = 0; i < 4; i++)
      q_err[i] = -q_err[i];
  }
  e6[3] = q_err[0]; // Roll 误差分量
  e6[4] = q_err[1]; // Pitch 误差分量
  e6[5] = q_err[2]; // Yaw 误差分量

  // C. 末端速度 v_ee = J * qd
  for (int i = 0; i < 6; i++) {
    for (int j = 0; j < 7; j++) {
      v_ee[i] += J[i][j] * qd[j];
    }
  }

  // D. 笛卡尔力矩 tau_cart = J^T * (K*e - D*v)
  double tau_cart[7] = {0};
  for (int i = 0; i < 7; i++) {
    for (int j = 0; j < 6; j++) {
      double F_j = cartesian_K[j] * e6[j] - cartesian_D[j] * v_ee[j];
      tau_cart[i] += J[j][i] * F_j;
    }
  }

  // 4. 总力矩融合: Tau = G + Tau_joint + Tau_cart
  for (int i = 0; i < 7; i++) {
    tau_out[i] = G[i] + tau_joint[i] + tau_cart[i];

    /* 施加关节力矩饱和限位 */
    if (tau_out[i] > TORQUE_LIMIT[i])
      tau_out[i] = TORQUE_LIMIT[i];
    if (tau_out[i] < -TORQUE_LIMIT[i])
      tau_out[i] = -TORQUE_LIMIT[i];
  }
}

/* 计算离心力/科氏力补偿 */
void control_calc_coriolis_compensation(const double q[7], const double qd[7],
                                        double tau_c[7]) {
  if (!g_initialized)
    control_init();
  double tau_gc[7];
  double G[7];
  rbdl_calc_gc(&g_rbdl_model, q, qd, tau_gc);
  rbdl_calc_gravity(&g_rbdl_model, q, G);
  for (int i = 0; i < 7; i++)
    tau_c[i] = tau_gc[i] - G[i];
}

/* 获取带有 TCP 偏移的正运动学位姿 */
void control_get_fk_with_offset(const double q[7], double pos[3],
                                double quat[4]) {
  if (!g_initialized)
    control_init();
  double quat_xyzw[4];
  rbdl_forward_kinematics(&g_rbdl_model, q, pos, quat_xyzw);
  apply_tcp_offset(pos, quat_xyzw, NULL);
  /* 转换为 Python/MuJoCo 的顺序 (w, x, y, z) */
  quat[0] = quat_xyzw[3];
  quat[1] = quat_xyzw[0];
  quat[2] = quat_xyzw[1];
  quat[3] = quat_xyzw[2];
}

/* ================================================================
 *  核心控制逻辑: 双空间阻抗控制 (control_step_v2)
 * ================================================================
 * 函数使用示例 包含默认的值
 *
 *
 * 函数输入的内容
 * target_pos: 目标末端位置 (URDF Base 坐标系) 格式为[x,y,z]
 * target_quat: 目标末端姿态 (URDF Base 坐标系) 格式为[w,x,y,z]
 * current_q: 当前关节角度 (URDF Base 坐标系) 格式为[q1,q2,q3,q4,q5,q6,q7]
 * current_qd: 当前关节速度 (URDF Base 坐标系)
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
void control_step_v2(const double target_pos[3], const double target_quat[4],
                     const double current_q[7], const double current_qd[7],
                     double tau_out[7]) {

  const double cartesian_K[6] = {KP_CART_X,    KP_CART_Y,     KP_CART_Z,
                                 KP_CART_ROLL, KP_CART_PITCH, KP_CART_YAW};
  const double cartesian_D[6] = {KD_CART_X,    KD_CART_Y,     KD_CART_Z,
                                 KD_CART_ROLL, KD_CART_PITCH, KD_CART_YAW};
  const double joint_kp[7] = {KP_JOINT_1, KP_JOINT_2, KP_JOINT_3, KP_JOINT_4,
                              KP_JOINT_5, KP_JOINT_6, KP_JOINT_7};
  const double joint_kd[7] = {KD_JOINT_1, KD_JOINT_2, KD_JOINT_3, KD_JOINT_4,
                              KD_JOINT_5, KD_JOINT_6, KD_JOINT_7};
  const double q_preferred[7] = {Q_PREF_1, Q_PREF_2, Q_PREF_3, Q_PREF_4,
                                 Q_PREF_5, Q_PREF_6, Q_PREF_7};
  double posture_alpha = POSTURE_ALPHA;
  double w_cart = W_CARTESIAN;
  double w_joint = W_JOINT;

  if (!g_initialized)
    control_init();

  double pos_ee[3], quat_xyzw[4], J_1D[42];
  rbdl_forward_kinematics(&g_rbdl_model, current_q, pos_ee, quat_xyzw);
  rbdl_calc_jacobian(&g_rbdl_model, current_q, J_1D);

  double J[6][7];
  for (int r = 0; r < 6; r++)
    for (int c = 0; c < 7; c++)
      J[r][c] = J_1D[r + 6 * c];
  apply_tcp_offset(pos_ee, quat_xyzw, J);

  /* 1. 计算末端位置与姿态误差 (与 Python quaternion_conjugate 和 multiply 对齐)
   */
  double e6[6], v_ee[6] = {0};

  // 位置误差
  e6[0] = target_pos[0] - pos_ee[0];
  e6[1] = target_pos[1] - pos_ee[1];
  e6[2] = target_pos[2] - pos_ee[2];

  // 姿态误差 (四元数乘法: q_target * q_current_conjugate, 统一使用 xyzw 格式)
  double target_xyzw[4] = {target_quat[1], target_quat[2], target_quat[3], target_quat[0]};
  double q_cur_inv[4], q_err[4];
  quat_conjugate(quat_xyzw, q_cur_inv);
  quat_mul(target_xyzw, q_cur_inv, q_err);

  if (q_err[3] < 0.0) { // w < 0 时翻转，保证最短路径旋转
    q_err[0] = -q_err[0];
    q_err[1] = -q_err[1];
    q_err[2] = -q_err[2];
    q_err[3] = -q_err[3];
  }
  
  // 误差矢量部分 (x, y, z) 对应 Roll, Pitch, Yaw 的瞬时旋转分量
  e6[3] = q_err[0];
  e6[4] = q_err[1];
  e6[5] = q_err[2];

  /* 2. 计算末端速度: v_ee = J * current_qd */
  for (int i = 0; i < 6; i++) {
    for (int j = 0; j < 7; j++) {
      v_ee[i] += J[i][j] * current_qd[j];
    }
  }

  /* 3. 笛卡尔空间主任务力矩: tau_task = J^T * (K * e6 - D * v_ee) */
  double tau_task[7] = {0};
  for (int i = 0; i < 7; i++) {
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
      for (int k = 0; k < 7; k++) {
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
  double J_pinv[7][6] = {0};
  for (int r = 0; r < 7; r++) {   // r 是 7 个关节
    for (int c = 0; c < 6; c++) { // c 是 6 个空间维度
      for (int k = 0; k < 6; k++) {
        J_pinv[r][c] += J[k][r] * invA[k * 6 + c];
      }
    }
  }

  /* 5. 底层运动学姿态小步长偏好规划 (dq = J_pinv * e6) */
  double dq_ik[7] = {0};
  for (int i = 0; i < 7; i++) {
    for (int j = 0; j < 6; j++) {
      dq_ik[i] += J_pinv[i][j] * e6[j];
    }
  }

  double q_f[7], q_ref[7];
  for (int i = 0; i < 7; i++) {
    q_f[i] = current_q[i] + dq_ik[i];
    q_ref[i] = (1.0 - posture_alpha) * q_f[i] + posture_alpha * q_preferred[i];
  }

  /* 6. 关节空间力矩（仅作为零空间辅助力） */
  double tau_null_control[7] = {0};
  for (int i = 0; i < 7; i++) {
    double eq = normalize_angle(q_ref[i] - current_q[i]);
    tau_null_control[i] = joint_kp[i] * eq - joint_kd[i] * current_qd[i];
  }

  /* 7. 零空间投影 N = I - J_pinv * J */
  double N[7][7] = {0};
  for (int i = 0; i < 7; i++) {
    N[i][i] = 1.0;
    for (int j = 0; j < 7; j++) {
      for (int k = 0; k < 6; k++) {
        N[i][j] -= J_pinv[i][k] * J[k][j];
      }
    }
  }

  // tau_null_projected = N^T * tau_null_control
  double tau_null_projected[7] = {0};
  for (int i = 0; i < 7; i++) {
    for (int j = 0; j < 7; j++) {
      tau_null_projected[i] += N[j][i] * tau_null_control[j]; // N[j][i] is N.T
    }
  }

  /* 8. 动力学补偿: 包含科氏力(Coriolis)和重力(Gravity) */
  double tau_gc[7];
  rbdl_calc_gc(&g_rbdl_model, current_q, current_qd, tau_gc);

  /* 9. 融合总力矩并限制输出 */
  for (int i = 0; i < 7; i++) {
    tau_out[i] =
        w_cart * tau_task[i] + w_joint * tau_null_projected[i] + tau_gc[i];

    /* 施加关节力矩饱和限位 */
    if (tau_out[i] > TORQUE_LIMIT[i])
      tau_out[i] = TORQUE_LIMIT[i];
    if (tau_out[i] < -TORQUE_LIMIT[i])
      tau_out[i] = -TORQUE_LIMIT[i];
  }
}

int control_check_safety(const double q[7], const double qd[7]) {
  const double joint_min[7] = {
      JOINT_POS_MIN_1, JOINT_POS_MIN_2, JOINT_POS_MIN_3, JOINT_POS_MIN_4,
      JOINT_POS_MIN_5, JOINT_POS_MIN_6, JOINT_POS_MIN_7};
  const double joint_max[7] = {
      JOINT_POS_MAX_1, JOINT_POS_MAX_2, JOINT_POS_MAX_3, JOINT_POS_MAX_4,
      JOINT_POS_MAX_5, JOINT_POS_MAX_6, JOINT_POS_MAX_7};

  for (int i = 0; i < 7; i++) {
    /* 位置检查 (允许 0.01 rad 的容差以避免边界抖动) */
    if (q[i] < joint_min[i] - 0.01) {
      STM_LOG_ERROR("[SAFETY] J%d position low: %.4f < %.4f\n", i + 1, q[i],
                    joint_min[i]);
      return -1;
    }
    if (q[i] > joint_max[i] + 0.01) {
      STM_LOG_ERROR("[SAFETY] J%d position high: %.4f > %.4f\n", i + 1, q[i],
                    joint_max[i]);
      return -1;
    }
    /* 速度检查 */
    if (qd[i] > JOINT_VEL_LIMIT || qd[i] < -JOINT_VEL_LIMIT) {
      STM_LOG_ERROR("[SAFETY] J%d velocity error: %.4f (limit: %.4f)\n",
                    i + 1, qd[i], JOINT_VEL_LIMIT);
      return -2;
    }
  }
  return 0;
}

void control_filter_velocities(const double qd_raw[7], double qd_filtered[7]) {
  if (!g_initialized) control_init();
  for (int i = 0; i < 7; i++) {
    qd_filtered[i] = kalman_filter1d_update(&g_vel_filters[i], qd_raw[i]);
  }
}
