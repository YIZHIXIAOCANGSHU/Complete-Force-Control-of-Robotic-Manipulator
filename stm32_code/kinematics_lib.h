/*
 * AM-D02 机器人运动学库
 * 保留当前双臂控制使用的 RBDL 正运动学与雅可比。
 * 针对 STM32H7 统一使用 'double' 类型。
 */

#ifndef KINEMATICS_LIB_H
#define KINEMATICS_LIB_H

#include "model_lib.h"

#ifdef __cplusplus
extern "C" {
#endif

/* 正运动学: 根据 q 计算所有连杆的 pos/quat */
void rbdl_forward_kinematics(RBDLModel *model, const double *q,
                             double pos_ee[3], double quat_ee[4]);

/* 几何雅可比矩阵: 6xN 矩阵 (列主序) */
void rbdl_calc_jacobian(RBDLModel *model, const double *q, double *J);

double normalize_angle(double angle);

#ifdef __cplusplus
}
#endif

#endif /* KINEMATICS_LIB_H */
