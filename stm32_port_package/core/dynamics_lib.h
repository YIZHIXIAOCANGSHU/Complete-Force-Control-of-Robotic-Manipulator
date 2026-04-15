/*
 * AM-D02 机器人动力学库
 * 整合了 RBDL-Lite RNEA 实现。
 */

#ifndef DYNAMICS_LIB_H
#define DYNAMICS_LIB_H

#include "model_lib.h"

#ifdef __cplusplus
extern "C" {
#endif

/* 递归牛顿-欧拉算法 (RNEA)
 * tau = ID(q, qd, qdd)
 * qdd 可以为 NULL (默认为 0) */
void rbdl_inverse_dynamics(const RBDLModel *model, const double *q,
                           const double *qd, const double *qdd, double *tau);

/* 重力补偿: G(q) = RNEA(q, 0, 0) */
void rbdl_calc_gravity(const RBDLModel *model, const double *q, double *G);

/* 重力 + 离心力/科氏力: tau_gc = RNEA(q, qd, 0) */
void rbdl_calc_gc(const RBDLModel *model, const double *q, const double *qd,
                  double *tau_gc);

#ifdef __cplusplus
}
#endif

#endif /* DYNAMICS_LIB_H */
