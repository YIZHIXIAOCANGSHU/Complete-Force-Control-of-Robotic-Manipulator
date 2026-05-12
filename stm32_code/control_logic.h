/*
 * AM-D02 机器人控制逻辑库
 * 整合了 fusion_controller 并暴露了补偿函数。
 * 针对 STM32H7 统一使用 'double' 类型。
 */

/**
 * @file control_logic.h
 * @brief STM32 机械臂主控制辅助逻辑封装
 *
 * 提供基础运动学封装、动力学补偿、速度滤波和安全检查。
 */
#ifndef CONTROL_LOGIC_H
#define CONTROL_LOGIC_H

#include "dynamics_lib.h"
#include "kinematics_lib.h"

#ifdef __cplusplus
extern "C" {
#endif

/* 初始化控制器和底层模型 */
void control_init(void);

/* 显式暴露的补偿函数 */
void control_calc_gravity_compensation(const double q[7], double G[7]);
void control_calc_gravity_pd_compensation(const double q[7], const double qd[7],
                                          const double q_target[7],
                                          double tau_out[7]);
void control_calc_coriolis_compensation(const double q[7], const double qd[7],
                                        double tau_c[7]);

/* 带有 TCP 偏移的正运动学辅助函数 */
void control_get_fk_with_offset(const double q[7], double pos[3],
                                double quat[4]);

/**
 * @brief 检查关节位置和速度是否在安全限位内
 * @param q 当前各关节角度
 * @param qd 当前各关节角速度
 * @return 0 if safe, -1 if position limit violated, -2 if velocity limit violated
 */
int control_check_safety(const double q[7], const double qd[7]);

/**
 * @brief 对关节速度进行 1D 卡尔曼滤波
 * @param qd_raw 原始输入速度
 * @param qd_filtered 滤波后的输出速度
 */
void control_filter_velocities(const double qd_raw[7], double qd_filtered[7]);

#ifdef __cplusplus
}
#endif

#endif /* CONTROL_LOGIC_H */
