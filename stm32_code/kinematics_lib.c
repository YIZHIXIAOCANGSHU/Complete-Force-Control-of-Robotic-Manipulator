#include "kinematics_lib.h"
#include <string.h>

/* ================================================================
 *  基于 RBDL 的正向运动学与雅可比结算
 * ================================================================ */

/**
 * @brief 基于RBDL算法计算正向运动学
 * @param model RBDL模型
 * @param q 关节角度数组
 * @param pos_ee 输出的末端位置 (x, y, z)
 * @param quat_ee 输出的末端姿态四元数 (x, y, z, w)
 */
void rbdl_forward_kinematics(RBDLModel *model, const double *q,
                             double pos_ee[3], double quat_ee[4]) {
  double T[16], T_fix[16], T_jnt[16], T_temp[16];
  double R_fix[9], R_jnt[9];
  int N = model->num_bodies;

  mat4_identity(T);

  for (int i = 0; i < N; i++) {
    const Body *body = &model->bodies[i];
    mat3_transpose(body->E_p, R_fix);
    mat4_from_rot_trans(R_fix, body->r_p, T_fix);

    axis_angle_to_rotmat(body->axis, q[i], R_jnt);
    double zero3[3] = {0, 0, 0};
    mat4_from_rot_trans(R_jnt, zero3, T_jnt);

    mat4_mul(T, T_fix, T_temp);
    mat4_mul(T_temp, T_jnt, T);
  }

  mat4_get_position(T, pos_ee);
  double R_ee[9];
  mat4_get_rotation(T, R_ee);
  rotmat_to_quat(R_ee, quat_ee);
}

/**
 * @brief 计算机器人末端相对于各关节的雅可比矩阵
 * @param model RBDL模型
 * @param q 关节角度数组
 * @param J 输出的雅可比矩阵 (6 x num_bodies，列优先存储)
 */
void rbdl_calc_jacobian(RBDLModel *model, const double *q, double *J) {
  double T[16], T_fix[16], T_jnt[16], T_temp[16];
  double R_fix[9], R_jnt[9];
  int N = model->num_bodies;

  double z[MAX_BODIES][3];
  double p[MAX_BODIES][3];

  mat4_identity(T);

  for (int i = 0; i < N; i++) {
    const Body *body = &model->bodies[i];
    mat3_transpose(body->E_p, R_fix);
    mat4_from_rot_trans(R_fix, body->r_p, T_fix);
    mat4_mul(T, T_fix, T_temp);

    mat4_rot_vec3(T_temp, body->axis, z[i]);
    mat4_get_position(T_temp, p[i]);

    axis_angle_to_rotmat(body->axis, q[i], R_jnt);
    double zero3[3] = {0, 0, 0};
    mat4_from_rot_trans(R_jnt, zero3, T_jnt);
    mat4_mul(T_temp, T_jnt, T);
  }

  double p_ee[3];
  mat4_get_position(T, p_ee);

  memset(J, 0, 6 * N * sizeof(double));
  /* 构建雅可比矩阵：计算每个关节对应的线速度和角速度贡献 */
  for (int j = 0; j < N; j++) {
    double dp[3], cross[3];
    /* dp = 末端位置 - 第 j 个关节的位置 */
    vec3_sub(p_ee, p[j], dp);
    /* 关节转动引起的末端线速度: v = omega x r (即 z_axis x dp) */
    vec3_cross(z[j], dp, cross);

    J[0 + 6 * j] = cross[0];
    J[1 + 6 * j] = cross[1];
    J[2 + 6 * j] = cross[2];
    J[3 + 6 * j] = z[j][0];
    J[4 + 6 * j] = z[j][1];
    J[5 + 6 * j] = z[j][2];
  }
}

/**
 * @brief 将角度归一化到 [-PI, PI] 范围内
 * @param angle 输入角度（弧度）
 * @return 归一化后的角度
 */
double normalize_angle(double angle) {
  while (angle > M_PI)
    angle -= 2.0 * M_PI;
  while (angle < -M_PI)
    angle += 2.0 * M_PI;
  return angle;
}
