#include "control_logic.h"
#include "config.h"
#include <stdio.h>
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

/* ================================================================
 *  内部辅助函数
 * ================================================================ */

/**
 * 应用 TCP 偏移到末端执行器位置
 */
static void apply_tcp_position_offset(double pos_ee[3],
                                      const double quat_xyzw[4]) {
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
}

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
  apply_tcp_position_offset(pos, quat_xyzw);
  /* 转换为 Python/MuJoCo 的顺序 (w, x, y, z) */
  quat[0] = quat_xyzw[3];
  quat[1] = quat_xyzw[0];
  quat[2] = quat_xyzw[1];
  quat[3] = quat_xyzw[2];
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
      fprintf(stderr, "[SAFETY] J%d position low: %.4f < %.4f\n", i + 1, q[i], joint_min[i]);
      return -1;
    }
    if (q[i] > joint_max[i] + 0.01) {
      fprintf(stderr, "[SAFETY] J%d position high: %.4f > %.4f\n", i + 1, q[i], joint_max[i]);
      return -1;
    }
    /* 速度检查 */
    if (qd[i] > JOINT_VEL_LIMIT || qd[i] < -JOINT_VEL_LIMIT) {
      fprintf(stderr, "[SAFETY] J%d velocity error: %.4f (limit: %.4f)\n", i + 1, qd[i], JOINT_VEL_LIMIT);
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
