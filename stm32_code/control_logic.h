/*
 * AM-D02 机器人控制逻辑库
 * 整合了双臂闭环控制所需的运动学、动力学和安全检查。
 * 针对 STM32H7 统一使用 'double' 类型。
 */

/**
 * @file control_logic.h
 * @brief STM32 机械臂双空间阻抗控制逻辑封装
 *
 * 提供 stm_controller 内部使用的运动学封装、动力学补偿以及单/双臂控制内核。
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
void control_update_body_gravity(const double body_q[NUM_BODY_JOINTS]);

typedef struct {
  double pos[3];
  double quat_wxyz[4];
  double quat_xyzw[4];
  double J[6][ARM_JOINTS];
} control_arm_kinematics_t;

/* 双空间阻抗控制内核
 * 注意: target_pos 使用 Body0422 动态目标坐标系；该坐标系原点跟随 Body0422_Link
 * 平移，坐标轴跟随 Body0422 相对零位旋转。target_quat 使用同一个动态目标坐标系。
 */
/**
 * @brief 单侧手臂控制逻辑，由 stm_controller_step_elapsed() 按 active_arm_mask 调用。
 *
 * 将当前机械臂末端与参考末端的差异投射到关节空间，并用 J*qd 估计 TCP 实际速度
 * 以闭环跟踪参考 twist，最终输出经过重力与科氏力补偿的控制力矩 tau_out。
 *
 * @param ref_pos 参考笛卡尔空间位置 [x, y, z] (Body0422 动态目标坐标)
 * @param ref_quat 参考姿态四元数 [w, x, y, z]
 * @param ref_twist 参考 TCP twist [vx, vy, vz, wx, wy, wz]
 * @param current_q 当前各关节角度 [q1..q7]
 * @param current_qd 当前各关节角速度 [qd1..qd7]
 * @param tau_out 传出计算得到的关节期望力矩 [tau1..tau7]
 */
void control_step_v2_arm_with_reference(
    int side, const double ref_pos[3], const double ref_quat[4],
    const double ref_twist[6],
    const double current_q[ARM_JOINTS],
    const double current_qd[ARM_JOINTS],
    const control_arm_kinematics_t *kinematics,
    double tau_out[ARM_JOINTS]);

/* 兼容旧调用：直接把最终目标当作参考点，并按目标方向生成默认线速度。 */
void control_step_v2_arm_with_state(
    int side, const double target_pos[3], const double target_quat[4],
    const double current_q[ARM_JOINTS],
    const double current_qd[ARM_JOINTS],
    const control_arm_kinematics_t *kinematics,
    double tau_out[ARM_JOINTS]);
void control_step_v2_dual_with_state(
    const double target_pos[NUM_ARMS][3],
    const double target_quat[NUM_ARMS][4],
    const double current_q[NUM_JOINTS],
    const double current_qd[NUM_JOINTS],
    const control_arm_kinematics_t kinematics[NUM_ARMS],
    double tau_out[NUM_JOINTS]);

/* 带有 TCP 偏移和 TCP 坐标系重标定的运动学状态。 */
void control_get_arm_kinematics_with_offset(
    int side, const double q[ARM_JOINTS],
    control_arm_kinematics_t *kinematics);

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
