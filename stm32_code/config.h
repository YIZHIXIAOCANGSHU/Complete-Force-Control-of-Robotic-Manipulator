/*
 * AM-D02 机器人控制器 - 配置参数
 * 集中管理参数，方便在 STM32H7 上进行调优。
 */

#ifndef CONFIG_LIB_H
#define CONFIG_LIB_H

/* ================================================================
 *  数学辅助宏
 * ================================================================ */
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#ifndef DEG2RAD
#define DEG2RAD(d) ((d) * M_PI / 180.0)
#endif

/* ================================================================
 *  平台日志钩子
 * ================================================================ */
/* 默认关闭日志；如需串口打印，可在工程里重定义为 printf/UART 宏 */
#ifndef STM_LOG_ERROR
#define STM_LOG_ERROR(...) ((void)0)
#endif

/* ================================================================
 *  基础运行参数
 * ================================================================ */
/* CONTROL_DT 只作为名义控制周期和调参参考；实际路径推进时间由
 * stm_controller_step_elapsed(..., elapsed_s) 的 elapsed_s 传入。
 */
#define CONTROL_DT 0.001  /* 名义控制周期 (s)，默认按 1 kHz 设计 */
/* elapsed_s 的单步上限保护。调大后暂停/卡顿恢复时路径会跳得更远；
 * 调小后大延迟会被压平，但路径实际推进会更保守。
 */
#define CONTROL_MAX_ELAPSED_S 0.02 /* 单步路径规划最大推进时间 (s) */
/* 安全锁定恢复只看激活臂 qd 是否接近 0，不要求 q 回零。阈值调大恢复更快
 * 但可能在轻微运动中恢复；调小更严格但恢复更慢。
 */
#define SAFETY_RECOVERY_QD_ZERO_TOL 0.02 /* 安全恢复静止阈值 (rad/s) */
#define SAFETY_RECOVERY_HOLD_S 0.1       /* 速度静止保持时间 (s) */
#define ARM_JOINTS 7     /* 单臂关节数量 */
#define NUM_ARMS 2       /* 左右双臂 */
#define NUM_JOINTS 14    /* 机器人受控关节数量 */
#define NUM_BODY_JOINTS 3 /* 仿真躯干外部命令关节数量 */
#define ARM_LEFT 0
#define ARM_RIGHT 1
/* Host bridge 仍使用该偏移量做 MuJoCo 坐标转换，保留兼容定义 */
#define MUJOCO_Z_OFFSET 1.0 /* MuJoCo 仿真中机器人基座的高度偏移 */

/* ================================================================
 *  TCP 偏移
 * ================================================================ */
/* TCP 偏移量相对于 ArmL07Output_Link / ArmR07Output_Link 本地坐标系，单位 m。
 * 当前控制实际使用 TCP_LEFT_OFFSET_* 和 TCP_RIGHT_OFFSET_*；通用 TCP_OFFSET_*
 * 保留兼容。左右 Y/Z 符号不同，用于保持两侧 TCP 坐标系和夹爪几何一致。
 */
#define TCP_OFFSET_X 0.0
#define TCP_OFFSET_Y 0.07
#define TCP_OFFSET_Z -0.03
#define TCP_LEFT_OFFSET_X 0.0
#define TCP_LEFT_OFFSET_Y 0.07
#define TCP_LEFT_OFFSET_Z -0.03
#define TCP_RIGHT_OFFSET_X 0.0
#define TCP_RIGHT_OFFSET_Y -0.07
#define TCP_RIGHT_OFFSET_Z 0.03

/* Body0422 动态目标坐标系零位原点（相对于 URDF base_link，单位 m）。
 * C 控制内部的 7 轴模型以该原点为根；目标位置也使用该坐标系。
 * 这个零位必须和 python/config.py 的 TARGET_FRAME_ORIGIN_BASE_ZERO 保持一致；
 * 如果只改 C 或只改 Python，会造成目标方块显示和 C 端 FK/控制坐标不一致。
 */
#define TARGET_FRAME_ORIGIN_BASE_X 0.0
#define TARGET_FRAME_ORIGIN_BASE_Y 0.0715607946769668
#define TARGET_FRAME_ORIGIN_BASE_Z 0.213

/* ================================================================
 *  笛卡尔空间 PD 增益
 * ================================================================ */
/* 终点跟踪增强档：作用在 TCP 位置/姿态误差上，输出笛卡尔任务力再通过 J^T 转成关节力矩。
 * KP_CART_* 调大: 末端跟踪更硬、误差收敛更快，但更容易振荡/饱和。
 * KD_CART_* 调大: 阻尼更强、超调更小，但过大可能放大速度噪声并让动作发涩。
 * XYZ 单位近似 N/m；ROLL/PITCH/YAW 作用在轴角姿态误差上。
 */
#define KP_CART_X 260.0
#define KP_CART_Y 260.0
#define KP_CART_Z 260.0

#define KD_CART_X 70.0
#define KD_CART_Y 70.0
#define KD_CART_Z 70.0

#define KP_CART_ROLL 12.0
#define KP_CART_PITCH 12.0
#define KP_CART_YAW 12.0

#define KD_CART_ROLL 4.0
#define KD_CART_PITCH 4.0
#define KD_CART_YAW 4.0

/* ================================================================
 *  关节空间 PD 增益
 * ================================================================ */
/* 当前关节 PD 主要用于零空间首选姿态，不是独立的关节位置闭环。
 * KP_JOINT_* 调大: 关节更积极靠近 Q_PREF_*；KD_JOINT_* 调大: 零空间动作更稳。
 * 若末端跟踪优先级不足，优先检查 W_CARTESIAN/W_JOINT 和 NULLSPACE_TORQUE_LIMIT。
 */
#define KP_JOINT_1 115.0
#define KP_JOINT_2 100.0
#define KP_JOINT_3 30.0
#define KP_JOINT_4 40.0
#define KP_JOINT_5 20.0
#define KP_JOINT_6 20.0
#define KP_JOINT_7 20.0

#define KD_JOINT_1 5
#define KD_JOINT_2 5
#define KD_JOINT_3 2
#define KD_JOINT_4 2
#define KD_JOINT_5 1
#define KD_JOINT_6 1
#define KD_JOINT_7 1

/* ================================================================
 *  力矩限制
 * ================================================================ */
/* 关节力矩限制 (N.m)。control_logic 中用于最终输出饱和，stm_controller 中也用于
 * 力矩安全检查；调小会更安全但更容易跟踪失败，调大前需要确认驱动器/减速器能力。
 */
#define JOINT_TORQUE_LIMIT_1 40.0
#define JOINT_TORQUE_LIMIT_2 40.0
#define JOINT_TORQUE_LIMIT_3 27.0
#define JOINT_TORQUE_LIMIT_4 27.0
#define JOINT_TORQUE_LIMIT_5 7.0
#define JOINT_TORQUE_LIMIT_6 7.0
#define JOINT_TORQUE_LIMIT_7 9.0

/* ================================================================
 *  偏好姿态与双空间权重 (stm_controller_step_elapsed 控制内核)
 * ================================================================ */
/* Q_PREF_* 是零空间希望靠近的 7 轴姿态，单位 rad。
 * POSTURE_ALPHA 越大，零空间参考越偏向 Q_PREF_*；越小，越偏向由 TCP 误差反推的小步。
 */
#define Q_PREF_1 0.0
#define Q_PREF_2 0.0
#define Q_PREF_3 0.0
#define Q_PREF_4 DEG2RAD(60.0)
#define Q_PREF_5 0.0
#define Q_PREF_6 0.0
#define Q_PREF_7 0.0

#define POSTURE_ALPHA 0.35
/* 总力矩融合: tau = W_CARTESIAN * 笛卡尔任务 + W_JOINT * 零空间 + 动力学补偿。
 * W_CARTESIAN 调大增强末端跟踪；W_JOINT 调大增强回偏好姿态能力。
 */
#define W_CARTESIAN 0.75
#define W_JOINT 0.25
/* deadband 内禁用零空间，避免末端接近目标时关节还在“抢控制权”。
 * full_scale 表示误差大到多少时零空间完全恢复；TORQUE_LIMIT 是零空间单轴限幅。
 */
#define NULLSPACE_POS_DEADBAND 0.001 /* m，目标附近禁用零空间偏置 */
#define NULLSPACE_ORI_DEADBAND 0.002 /* rad，目标附近禁用零空间偏置 */
#define NULLSPACE_POS_FULL_SCALE 0.02 /* m，远离目标后恢复轻度零空间 */
#define NULLSPACE_ORI_FULL_SCALE 0.05 /* rad，远离目标后恢复轻度零空间 */
#define NULLSPACE_TORQUE_LIMIT 0.08  /* N.m，温和零空间补偿限幅 */

/* ================================================================
 *  运动学与逆解参数
 * ================================================================ */
/* 保留兼容参数：当前 stm_controller 主闭环不直接调用旧数值 IK，
 * 末端运动主要由 LinearPathPlanner + 笛卡尔 PD + 零空间完成。
 * 调这些 IK_* 不会直接改变当前主控制链路的末端跟踪表现。
 */
#define IK_MAX_ITERATIONS 50
#define IK_TOL_POS 0.005 /* 5mm 容差 */
#define IK_T_ORI 0.01    /* ~0.57 度容差 */
#define IK_MAX_STEP 0.2  /* 每次迭代最大关节步进 (弧度) */
#define IK_DAMPING 0.1   /* DLS 阻尼因子 lambda */

/* ================================================================
 *  关节安全限位
 * ================================================================ */
/* 左臂位置限位使用 degree 记录，下面 JOINT_POS_MIN/MAX_* 会转换为 rad。
 * control_check_safety_arm() 会按 CONTROL_JOINT_LIMIT_INSET_RATIO 将每轴
 * [min,max] 行程向内收缩，留出机械/控制保护余量。
 */
#define CONTROL_JOINT_LIMIT_INSET_RATIO 0.01 /* 位置安全限位向内收 1% */
#define JOINT_POS_MIN_1_DEG (-89.971835)
#define JOINT_POS_MAX_1_DEG (89.971835)
#define JOINT_POS_MIN_2_DEG (-20.587610)
#define JOINT_POS_MAX_2_DEG (89.954374)
#define JOINT_POS_MIN_3_DEG (-45.836624)
#define JOINT_POS_MAX_3_DEG (68.754935)
#define JOINT_POS_MIN_4_DEG (-119.748454)
#define JOINT_POS_MAX_4_DEG (119.954374)
#define JOINT_POS_MIN_5_DEG (-45.836624)
#define JOINT_POS_MAX_5_DEG (45.836624)
#define JOINT_POS_MIN_6_DEG (-45.263666)
#define JOINT_POS_MAX_6_DEG (61.306275)
#define JOINT_POS_MIN_7_DEG (-61.306275)
#define JOINT_POS_MAX_7_DEG (61.306275)

#define JOINT_POS_MIN_1 DEG2RAD(JOINT_POS_MIN_1_DEG)
#define JOINT_POS_MAX_1 DEG2RAD(JOINT_POS_MAX_1_DEG)
#define JOINT_POS_MIN_2 DEG2RAD(JOINT_POS_MIN_2_DEG)
#define JOINT_POS_MAX_2 DEG2RAD(JOINT_POS_MAX_2_DEG)
#define JOINT_POS_MIN_3 DEG2RAD(JOINT_POS_MIN_3_DEG)
#define JOINT_POS_MAX_3 DEG2RAD(JOINT_POS_MAX_3_DEG)
#define JOINT_POS_MIN_4 DEG2RAD(JOINT_POS_MIN_4_DEG)
#define JOINT_POS_MAX_4 DEG2RAD(JOINT_POS_MAX_4_DEG)
#define JOINT_POS_MIN_5 DEG2RAD(JOINT_POS_MIN_5_DEG)
#define JOINT_POS_MAX_5 DEG2RAD(JOINT_POS_MAX_5_DEG)
#define JOINT_POS_MIN_6 DEG2RAD(JOINT_POS_MIN_6_DEG)
#define JOINT_POS_MAX_6 DEG2RAD(JOINT_POS_MAX_6_DEG)
#define JOINT_POS_MIN_7 DEG2RAD(JOINT_POS_MIN_7_DEG)
#define JOINT_POS_MAX_7 DEG2RAD(JOINT_POS_MAX_7_DEG)

/* 右臂位置限位来自当前双臂 URDF（单位 rad），不再经过 degree 转换。
 * 修改 URDF 或真机机械限位后，需要同步检查这里和左臂限位的安全范围。
 */
#define RIGHT_JOINT_POS_MIN_1 (-2.405)
#define RIGHT_JOINT_POS_MAX_1 (2.2175)
#define RIGHT_JOINT_POS_MIN_2 (-0.6605)
#define RIGHT_JOINT_POS_MAX_2 (2.203)
#define RIGHT_JOINT_POS_MIN_3 (-1.763)
#define RIGHT_JOINT_POS_MAX_3 (1.594)
#define RIGHT_JOINT_POS_MIN_4 (-0.0165)
#define RIGHT_JOINT_POS_MAX_4 (2.3235)
#define RIGHT_JOINT_POS_MIN_5 (-1.5935)
#define RIGHT_JOINT_POS_MAX_5 (1.574)
#define RIGHT_JOINT_POS_MIN_6 (-0.6015)
#define RIGHT_JOINT_POS_MAX_6 (0.6755)
#define RIGHT_JOINT_POS_MIN_7 (-1.1075)
#define RIGHT_JOINT_POS_MAX_7 (1.068)

/* 速度限位 (rad/s)。当前左右臂每轴都统一为 4.0 rad/s；
 * URDF 中 velocity="0" 视为未有效指定，不能直接作为速度上限。
 * 超过该阈值会进入安全锁定并输出当前激活臂全 0 力矩。
 */
#define JOINT_VEL_LIMIT 4.0
#define JOINT_VEL_LIMIT_1 JOINT_VEL_LIMIT
#define JOINT_VEL_LIMIT_2 JOINT_VEL_LIMIT
#define JOINT_VEL_LIMIT_3 JOINT_VEL_LIMIT
#define JOINT_VEL_LIMIT_4 JOINT_VEL_LIMIT
#define JOINT_VEL_LIMIT_5 JOINT_VEL_LIMIT
#define JOINT_VEL_LIMIT_6 JOINT_VEL_LIMIT
#define JOINT_VEL_LIMIT_7 JOINT_VEL_LIMIT

#define RIGHT_JOINT_VEL_LIMIT_1 JOINT_VEL_LIMIT
#define RIGHT_JOINT_VEL_LIMIT_2 JOINT_VEL_LIMIT
#define RIGHT_JOINT_VEL_LIMIT_3 JOINT_VEL_LIMIT
#define RIGHT_JOINT_VEL_LIMIT_4 JOINT_VEL_LIMIT
#define RIGHT_JOINT_VEL_LIMIT_5 JOINT_VEL_LIMIT
#define RIGHT_JOINT_VEL_LIMIT_6 JOINT_VEL_LIMIT
#define RIGHT_JOINT_VEL_LIMIT_7 JOINT_VEL_LIMIT

/* 卡尔曼滤波参数 (速度)。
 * KALMAN_Q_VEL 调大: 更信任速度变化，响应更快但更吵。
 * KALMAN_R_VEL 调大: 更不信任测量，速度更平滑但滞后更大。
 */
#define KALMAN_Q_VEL 0.001 /* 过程噪声: 信任模型程度 (较小值增加滤波强度) */
#define KALMAN_R_VEL 0.1   /* 测量噪声: 信任传感器程度 (较大值增加滤波强度) */

/* ================================================================
 *  路径规划参数（末端笛卡尔直线路径）
 * ================================================================ */
/* LinearPathPlanner 生成末端参考点，速度由 elapsed_s 推进。
 * TRAJ_PLAN_SPEED/ACCEL 决定“参考点”的定速/加速，不保证实际 TCP 在力矩饱和、
 * 限位、安全锁定或外力扰动下仍严格达到该速度。
 */
#define TRAJ_PLAN_SPEED 0.5    /* 末端运动速度 (m/s) */
#define TRAJ_PLAN_ACCEL 0.2    /* 加速度 (m/s^2) */
/* Real/Sim 共用的路径跟随门控。真实 TCP 跟不上参考点时，路径时间会平滑减速
 * 或暂停，避免参考点继续跑远；只约束路径推进，不替代关节速度硬保护。
 */
#define CONTROL_PATH_GATE_FULL_ERROR_M 0.005
#define CONTROL_PATH_GATE_STOP_ERROR_M 0.020
#define CONTROL_PATH_GATE_RISE_TIME_S 0.080
#define CONTROL_PATH_GATE_FALL_TIME_S 0.030
#define CONTROL_PATH_LOOKAHEAD_M 0.008
#define CONTROL_PATH_LOOKAHEAD_RAMP_S 0.30
#define CONTROL_TARGET_REPLAN_POS_EPS_M 0.001
#define CONTROL_TARGET_REPLAN_ORI_EPS_RAD 0.002
/* 当前主控制链路不直接使用 TRAJ_REACH_THRESH；保留给路径到达判定辅助函数。 */
#define TRAJ_REACH_THRESH 0.005 /* 到达目标的位置阈值 (m) */

#endif /* CONFIG_LIB_H */
