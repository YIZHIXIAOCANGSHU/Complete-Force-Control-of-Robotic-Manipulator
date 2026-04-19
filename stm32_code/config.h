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
#define CONTROL_DT 0.001    /* 控制步长 (秒) */
#define NUM_JOINTS 7        /* 机器人关节数量 */
/* Host bridge 仍使用该偏移量做 MuJoCo 坐标转换，保留兼容定义 */
#define MUJOCO_Z_OFFSET 1.0 /* MuJoCo 仿真中机器人基座的高度偏移 */

/* ================================================================
 *  TCP 偏移
 * ================================================================ */
/* TCP 偏移量 (相对于 ArmLseventh_Joint，本地坐标系，单位 m) */
#define TCP_OFFSET_X 0.0
#define TCP_OFFSET_Y 0.07
#define TCP_OFFSET_Z -0.03

/* ================================================================
 *  笛卡尔空间 PD 增益
 * ================================================================ */
#define KP_CART_X 60.0
#define KP_CART_Y 60.0
#define KP_CART_Z 60.0
#define KP_CART_ROLL 2.5
#define KP_CART_PITCH 2.5
#define KP_CART_YAW 2.5

#define KD_CART_X 15.0
#define KD_CART_Y 15.0
#define KD_CART_Z 15.0
#define KD_CART_ROLL 0.8
#define KD_CART_PITCH 0.8
#define KD_CART_YAW 0.8

/* ================================================================
 *  关节空间 PD 增益
 * ================================================================ */
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
/* 关节力矩限制 (N.m)，与 AM-D02-AemLURDF0413 URDF limit effort 对齐 */
#define JOINT_TORQUE_LIMIT_1 40.0
#define JOINT_TORQUE_LIMIT_2 40.0
#define JOINT_TORQUE_LIMIT_3 27.0
#define JOINT_TORQUE_LIMIT_4 27.0
#define JOINT_TORQUE_LIMIT_5 7.0
#define JOINT_TORQUE_LIMIT_6 7.0
#define JOINT_TORQUE_LIMIT_7 9.0

/* ================================================================
 *  偏好姿态与双空间权重 (control_step_v2)
 * ================================================================ */
#define Q_PREF_1 0.0
#define Q_PREF_2 0.0
#define Q_PREF_3 0.0
#define Q_PREF_4 DEG2RAD(60.0)
#define Q_PREF_5 0.0
#define Q_PREF_6 0.0
#define Q_PREF_7 0.0

#define POSTURE_ALPHA 0.3
#define W_CARTESIAN 0.8
#define W_JOINT 0.2

/* ================================================================
 *  运动学与逆解参数
 * ================================================================ */
#define IK_MAX_ITERATIONS 50
#define IK_TOL_POS 0.005 /* 5mm 容差 */
#define IK_T_ORI 0.01    /* ~0.57 度容差 */
#define IK_MAX_STEP 0.2  /* 每次迭代最大关节步进 (弧度) */
#define IK_DAMPING 0.1   /* DLS 阻尼因子 lambda */

/* ================================================================
 *  关节安全限位
 * ================================================================ */
/* 位置限位单位为 rad，直接供控制与运动学模块使用 */
#define JOINT_POS_MIN_1 (-0.830)
#define JOINT_POS_MAX_1 (0.829)
#define JOINT_POS_MIN_2 (-0.819)
#define JOINT_POS_MAX_2 (0.0)
#define JOINT_POS_MIN_3 (-1.592)
#define JOINT_POS_MAX_3 (1.681)
#define JOINT_POS_MIN_4 (0.0)
#define JOINT_POS_MAX_4 (1.523)
#define JOINT_POS_MIN_5 (-1.428)
#define JOINT_POS_MAX_5 (1.536)
#define JOINT_POS_MIN_6 (-0.7384)
#define JOINT_POS_MAX_6 (0.6513)
#define JOINT_POS_MIN_7 (-0.8899)
#define JOINT_POS_MAX_7 (1.622)

/* 速度限位 (rad/s) */
#define JOINT_VEL_LIMIT 5.0

/* 卡尔曼滤波参数 (速度) */
#define KALMAN_Q_VEL 0.001 /* 过程噪声: 信任模型程度 (较小值增加滤波强度) */
#define KALMAN_R_VEL 0.1   /* 测量噪声: 信任传感器程度 (较大值增加滤波强度) */

/* ================================================================
 *  路径规划参数（末端笛卡尔直线路径）
 * ================================================================ */
#define TRAJ_PLAN_SPEED 3.0     /* 末端运动速度 (m/s) */
#define TRAJ_PLAN_ACCEL 4.0     /* 加速度 (m/s^2) */
#define TRAJ_REACH_THRESH 0.005 /* 到达目标的位置阈值 (m) */

#endif /* CONFIG_LIB_H */
