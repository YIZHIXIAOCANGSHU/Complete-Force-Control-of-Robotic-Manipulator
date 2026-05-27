/*
 * AM-D02 机器人控制逻辑库
 * 整合了 fusion_controller 并暴露了补偿函数。
 * 针对 STM32H7 统一使用 'double' 类型。
 */

/**
 * @file control_logic.h
 * @brief STM32 机械臂双空间阻抗控制逻辑封装
 *
 * 提供基础的运动学封装、动力学补偿以及核心的双臂控制步进函数。
 */
#ifndef CONTROL_LOGIC_H
#define CONTROL_LOGIC_H

#include "dynamics_lib.h"
#include "kinematics_lib.h"
#include "config.h"

#ifdef __cplusplus
extern "C" {
#endif

/* 初始化控制器和底层模型 */
void control_init(void);
void control_init_arm(int side);
void control_update_body_gravity(const double body_q[NUM_BODY_JOINTS]);
void control_get_body_gravity(double gravity_out[3]);

/* 步进 V2: 规划 + 双空间阻抗控制
 * 注意: target_pos 使用 Body0422 动态目标坐标系；该坐标系原点跟随 Body0422_Link
 * 平移，坐标轴跟随 Body0422 相对零位旋转。target_quat 使用同一个动态目标坐标系。
 */
/**
 * @brief 单侧手臂控制逻辑，由 control_step_v2_dual 调用。
 *
 * 将当前机械臂末端与目标末端的差异投射到关节空间，同时在零空间(Null-space)维护首选姿态，
 * 最终输出经过重力与科氏力补偿的控制力矩 tau_out。
 *
 * @param target_pos 目标笛卡尔空间位置 [x, y, z] (Body0422 动态目标坐标)
 * @param target_quat 目标姿态四元数 [w, x, y, z]
 * @param current_q 当前各关节角度 [q1..q7]
 * @param current_qd 当前各关节角速度 [qd1..qd7]
 * @param tau_out 传出计算得到的关节期望力矩 [tau1..tau7]
 */
void control_step_v2_arm(int side, const double target_pos[3],
                         const double target_quat[4],
                         const double current_q[ARM_JOINTS],
                         const double current_qd[ARM_JOINTS],
                         double tau_out[ARM_JOINTS]);
void control_step_v2_dual(const double target_pos[NUM_ARMS][3],
                          const double target_quat[NUM_ARMS][4],
                          const double current_q[NUM_JOINTS],
                          const double current_qd[NUM_JOINTS],
                          double tau_out[NUM_JOINTS]);

/* 显式暴露的补偿函数 */
void control_calc_gravity_compensation_arm(int side, const double q[ARM_JOINTS],
                                           double G[ARM_JOINTS]);
void control_calc_coriolis_compensation_arm(int side,
                                            const double q[ARM_JOINTS],
                                            const double qd[ARM_JOINTS],
                                            double tau_c[ARM_JOINTS]);

/* 带有 TCP 偏移的正运动学辅助函数 */
void control_get_fk_with_offset_arm(int side, const double q[ARM_JOINTS],
                                    double pos[3], double quat[4]);

/**
 * @brief 检查关节位置和速度是否在安全限位内
 * @param q 当前各关节角度
 * @param qd 当前各关节角速度
 * @return 0 if safe, -1 if position limit violated, -2 if velocity limit violated
 */
int control_check_safety_arm(int side, const double q[ARM_JOINTS],
                             const double qd[ARM_JOINTS]);

/**
 * @brief 对关节速度进行 1D 卡尔曼滤波
 * @param qd_raw 原始输入速度
 * @param qd_filtered 滤波后的输出速度
 */
void control_filter_velocities_arm(int side, const double qd_raw[ARM_JOINTS],
                                   double qd_filtered[ARM_JOINTS]);

#ifdef __cplusplus
}
#endif

#endif /* CONTROL_LOGIC_H */
