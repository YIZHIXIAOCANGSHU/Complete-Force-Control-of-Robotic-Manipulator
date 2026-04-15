/*
 * AM-D02 机器人运动学库
 * 整合了 kinematics_solver (DH) 和 rbdl_kinematics (RBDL)。
 * 针对 STM32H7 统一使用 'double' 类型。
 */

#ifndef KINEMATICS_LIB_H
#define KINEMATICS_LIB_H

#include "model_lib.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 逆运动学 (IK) 求解器配置参数
 */
typedef struct {
  uint8_t use_joint_limits;            /* 是否启用关节角度限幅 */
  double lambda;                       /* DLS (阻尼最小二乘法) 阻尼因子 */
  double joint_limits_min[NUM_JOINTS]; /* 关节角度下限 */
  double joint_limits_max[NUM_JOINTS]; /* 关节角度上限 */
} IKConfig;

/**
 * @brief 运动学求解器状态管理器 (用于数值解法)
 */
typedef struct {
  DHParameters dh_params[NUM_JOINTS]; /* 内部使用的 DH 参数缓冲 */
  double joint_angles[NUM_JOINTS];    /* 当前缓存的关节角度 */
  TransformMatrix
      link_transforms[NUM_JOINTS]; /* 各连杆相对于基座的齐次变换矩阵 */
  Pose end_effector_pose;          /* 末端位姿缓存 */
  IKConfig ik_config;              /* 求解器配置 */
  uint8_t initialized;             /* 初始化完成标志 */
} KinematicsSolver;

/* ================================================================
 *  基于 RBDL 的运动学 (高性能)
 * ================================================================ */

/* 正运动学: 根据 q 计算所有连杆的 pos/quat */
void rbdl_forward_kinematics(RBDLModel *model, const double *q,
                             double pos_ee[3], double quat_ee[4]);

/* 几何雅可比矩阵: 6xN 矩阵 (列主序) */
void rbdl_calc_jacobian(RBDLModel *model, const double *q, double *J);

/* ================================================================
 *  基于 DH 的运动学求解器 (数值逆运动学)
 * ================================================================ */

void kinematics_init(KinematicsSolver *solver);
void kinematics_set_joint_angles(KinematicsSolver *solver,
                                 const double *angles);
uint8_t kinematics_compute_forward(KinematicsSolver *solver);

/* 数值逆运动学 */
uint8_t kinematics_compute_inverse_pose_dls(KinematicsSolver *solver,
                                            const Pose *target_pose,
                                            const double *initial_joints,
                                            double *result_joints,
                                            uint16_t max_iterations);

/* 工具函数 */
void pose_error_6d(const Pose *target, const Pose *current, double e6[6]);
double normalize_angle(double angle);
void kinematics_clamp_joints(const KinematicsSolver *solver, double *joints);

#ifdef __cplusplus
}
#endif

#endif /* KINEMATICS_LIB_H */
