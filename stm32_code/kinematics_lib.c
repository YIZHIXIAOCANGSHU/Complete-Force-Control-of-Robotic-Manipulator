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
    rpy_to_rotmat(AM_D02_JOINT_RPY[i][0], AM_D02_JOINT_RPY[i][1],
                  AM_D02_JOINT_RPY[i][2], R_fix);
    mat4_from_rot_trans(R_fix, AM_D02_JOINT_XYZ[i], T_fix);

    axis_angle_to_rotmat(AM_D02_JOINT_AXIS[i], q[i], R_jnt);
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
    rpy_to_rotmat(AM_D02_JOINT_RPY[i][0], AM_D02_JOINT_RPY[i][1],
                  AM_D02_JOINT_RPY[i][2], R_fix);
    mat4_from_rot_trans(R_fix, AM_D02_JOINT_XYZ[i], T_fix);
    mat4_mul(T, T_fix, T_temp);

    mat4_rot_vec3(T_temp, AM_D02_JOINT_AXIS[i], z[i]);
    mat4_get_position(T_temp, p[i]);

    axis_angle_to_rotmat(AM_D02_JOINT_AXIS[i], q[i], R_jnt);
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

/* ================================================================
 *  运动学求解器通用逻辑 (Consolidated Solver Logic)
 * ================================================================ */

/**
 * @brief 初始化运动学求解器
 * @param solver 运动学求解器对象
 */
void kinematics_init(KinematicsSolver *solver) {
  if (!solver)
    return;
  memset(solver, 0, sizeof(KinematicsSolver));
  solver->end_effector_pose.orientation.w = 1.0;
  solver->ik_config.lambda = IK_DAMPING;
  solver->ik_config.use_joint_limits = 1;
  solver->ik_config.joint_limits_min[0] = JOINT_POS_MIN_1;
  solver->ik_config.joint_limits_min[1] = JOINT_POS_MIN_2;
  solver->ik_config.joint_limits_min[2] = JOINT_POS_MIN_3;
  solver->ik_config.joint_limits_min[3] = JOINT_POS_MIN_4;
  solver->ik_config.joint_limits_min[4] = JOINT_POS_MIN_5;
  solver->ik_config.joint_limits_min[5] = JOINT_POS_MIN_6;
  solver->ik_config.joint_limits_min[6] = JOINT_POS_MIN_7;
  solver->ik_config.joint_limits_max[0] = JOINT_POS_MAX_1;
  solver->ik_config.joint_limits_max[1] = JOINT_POS_MAX_2;
  solver->ik_config.joint_limits_max[2] = JOINT_POS_MAX_3;
  solver->ik_config.joint_limits_max[3] = JOINT_POS_MAX_4;
  solver->ik_config.joint_limits_max[4] = JOINT_POS_MAX_5;
  solver->ik_config.joint_limits_max[5] = JOINT_POS_MAX_6;
  solver->ik_config.joint_limits_max[6] = JOINT_POS_MAX_7;
  solver->initialized = 1;
}

/**
 * @brief 设置当前关节角度
 * @param solver 运动学求解器对象
 * @param angles 关节角度数组
 */
void kinematics_set_joint_angles(KinematicsSolver *solver,
                                 const double *angles) {
  if (!solver || !angles)
    return;
  memcpy(solver->joint_angles, angles, sizeof(double) * NUM_JOINTS);
}

/**
 * @brief 计算正向运动学并更新连杆变换和末端姿态
 * @param solver 运动学求解器对象
 * @return 成功返回1，未初始化返回0
 */
uint8_t kinematics_compute_forward(KinematicsSolver *solver) {
  if (!solver || !solver->initialized)
    return 0;

  double T[16];
  mat4_identity(T);

  for (int i = 0; i < NUM_JOINTS; i++) {
    double R_fix[9], R_jnt[9], T_fix[16], T_jnt[16], T_temp[16];
    rpy_to_rotmat(AM_D02_JOINT_RPY[i][0], AM_D02_JOINT_RPY[i][1],
                  AM_D02_JOINT_RPY[i][2], R_fix);
    mat4_from_rot_trans(R_fix, AM_D02_JOINT_XYZ[i], T_fix);

    axis_angle_to_rotmat(AM_D02_JOINT_AXIS[i], solver->joint_angles[i], R_jnt);
    double zero3[3] = {0, 0, 0};
    mat4_from_rot_trans(R_jnt, zero3, T_jnt);

    mat4_mul(T, T_fix, T_temp);
    mat4_mul(T_temp, T_jnt, T);
    memcpy(solver->link_transforms[i].m, T, sizeof(double) * 16);
  }

  mat4_get_position(T, solver->end_effector_pose.position);
  double R_ee[9];
  mat4_get_rotation(T, R_ee);
  rotmat_to_quat(R_ee, (double *)&solver->end_effector_pose.orientation);

  return 1;
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

/**
 * @brief 根据设定的关节限制，对计算得出的关节角度进行限幅
 * @param solver 运动学求解器对象
 * @param joints 需要限幅的关节角度数组
 */
void kinematics_clamp_joints(const KinematicsSolver *solver, double *joints) {
  for (int i = 0; i < NUM_JOINTS; i++) {
    if (joints[i] < solver->ik_config.joint_limits_min[i])
      joints[i] = solver->ik_config.joint_limits_min[i];
    if (joints[i] > solver->ik_config.joint_limits_max[i])
      joints[i] = solver->ik_config.joint_limits_max[i];
  }
}

/**
 * @brief 计算目标位姿和当前位姿之间的六维误差（包含位置误差和姿态误差）
 * @param target 目标位姿
 * @param current 当前位姿
 * @param e6 输出的6维误差向量 [dx, dy, dz, wx, wy, wz]
 */
void pose_error_6d(const Pose *target, const Pose *current, double e6[6]) {
  vec3_sub(target->position, current->position, e6);

  double q_target[4] = {target->orientation.x, target->orientation.y,
                        target->orientation.z, target->orientation.w};
  double q_curr[4] = {current->orientation.x, current->orientation.y,
                      current->orientation.z, current->orientation.w};
  double q_inv[4], q_err[4];
  quat_conjugate(q_curr, q_inv);
  quat_mul(q_target, q_inv, q_err);

  if (q_err[3] < 0.0) {
    for (int i = 0; i < 4; i++)
      q_err[i] = -q_err[i];
  }

  double s =
      sqrt(q_err[0] * q_err[0] + q_err[1] * q_err[1] + q_err[2] * q_err[2]);
  if (s < 1e-6) {
    e6[3] = 2.0 * q_err[0];
    e6[4] = 2.0 * q_err[1];
    e6[5] = 2.0 * q_err[2];
  } else {
    double angle = 2.0 * atan2(s, q_err[3]);
    double k = angle / s;
    e6[3] = k * q_err[0];
    e6[4] = k * q_err[1];
    e6[5] = k * q_err[2];
  }
}

/**
 * @brief 阻尼最小二乘法 (DLS) 逆运动学求解
 * @param solver 运动学求解器对象
 * @param target_pose 目标位姿
 * @param initial_joints
 * 初始关节参考角度（可选，传NULL则使用solver中的当前角度）
 * @param result_joints 输出的逆解关节角度
 * @param max_iterations 最大迭代次数
 * @return 成功在容差内收敛返回1，达到最大迭代次数依然未收敛返回0
 */
uint8_t kinematics_compute_inverse_pose_dls(KinematicsSolver *solver,
                                            const Pose *target_pose,
                                            const double *initial_joints,
                                            double *result_joints,
                                            uint16_t max_iterations) {
  double q[NUM_JOINTS];
  if (initial_joints)
    memcpy(q, initial_joints, sizeof(q));
  else
    memcpy(q, solver->joint_angles, sizeof(q));

  for (uint16_t iter = 0; iter < max_iterations; iter++) {
    kinematics_set_joint_angles(solver, q);
    kinematics_compute_forward(solver);

    double e6[6];
    pose_error_6d(target_pose, &solver->end_effector_pose, e6);

    double pos_err = sqrt(e6[0] * e6[0] + e6[1] * e6[1] + e6[2] * e6[2]);
    double rot_err = sqrt(e6[3] * e6[3] + e6[4] * e6[4] + e6[5] * e6[5]);

    if (pos_err < IK_TOL_POS && rot_err < IK_T_ORI) {
      memcpy(result_joints, q, sizeof(q));
      return 1;
    }

    double J[42]; /* 6x7 雅可比矩阵 (列主序 Column Major) */
    RBDLModel
        dummy_model; /* 雅可比计算无需真实的质量惯量参数，使用空模型即可 */
    dummy_model.num_bodies = NUM_JOINTS;
    rbdl_calc_jacobian(&dummy_model, q, J);

    /* 将列主序的 J[6*7] 转换为 A[6*6] = J * J^T，用于 DLS 伪逆计算 */
    double A[36] = {0};
    for (int r = 0; r < 6; r++) {
      for (int c = 0; c < 6; c++) {
        for (int k = 0; k < NUM_JOINTS; k++) {
          A[r * 6 + c] += J[r + 6 * k] * J[c + 6 * k];
        }
      }
    }
    double l2 = solver->ik_config.lambda * solver->ik_config.lambda;
    for (int i = 0; i < 6; i++)
      A[i * 6 + i] += l2;

    double invA[36];
    if (!mat6_inverse(A, invA))
      return 0;

    /* 计算关节更新步长 dq = J^T * invA * e6 */
    double b[6] = {0};
    for (int i = 0; i < 6; i++) {
      for (int j = 0; j < 6; j++)
        b[i] += invA[i * 6 + j] * e6[j];
    }

    for (int i = 0; i < NUM_JOINTS; i++) {
      double dq_i = 0;
      for (int k = 0; k < 6; k++)
        dq_i += J[k + 6 * i] * b[k];

      if (dq_i > IK_MAX_STEP)
        dq_i = IK_MAX_STEP;
      if (dq_i < -IK_MAX_STEP)
        dq_i = -IK_MAX_STEP;

      q[i] += 0.5 * dq_i; /* 设定积分增益为 0.5，确保迭代稳定性 */
      q[i] = normalize_angle(q[i]);
    }

    if (solver->ik_config.use_joint_limits)
      kinematics_clamp_joints(solver, q);
  }

  memcpy(result_joints, q, sizeof(q));
  return 0;
}
