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
/* 控制核心没有固定周期；S 曲线路径时间、traj_t 和安全恢复计时全部由
 * stm_controller_step_elapsed(..., elapsed_s) 的平台层实测 elapsed_s 推进。
 */
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
#define KP_CART_X 270.0
#define KP_CART_Y 270.0
#define KP_CART_Z 270.0

#define KD_CART_X 80.0
#define KD_CART_Y 80.0
#define KD_CART_Z 80.0

#define KP_CART_ROLL 8.0
#define KP_CART_PITCH 8.0
#define KP_CART_YAW 8.0

#define KD_CART_ROLL 4.0
#define KD_CART_PITCH 4.0
#define KD_CART_YAW 4.0

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
 *  运动学与逆解参数
 * ================================================================ */
/* 保留兼容参数：当前 stm_controller 主闭环不直接调用旧数值 IK，
 * 末端运动由最终 TCP 目标 + 基于 qd 反馈的 TCP 速度闭环完成。
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

/* 速度限位 (rad/s)。当前左右臂每轴都统一为 5.0 rad/s；
 * URDF 中 velocity="0" 视为未有效指定，不能直接作为速度上限。
 * 超过该阈值会进入安全锁定并输出当前激活臂全 0 力矩。
 */
#define JOINT_VEL_LIMIT 5.0
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
 *  末端速度闭环参数
 * ================================================================ */
/* C 端内部按最终目标生成五次 S 曲线 TCP 参考点。elapsed_s 由平台层传入；
 * END_EFFECTOR_LINEAR_SPEED_MPS 表示 S 曲线峰值线速度。实际 TCP 速度由
 * v_tcp = J(q) * qd 估计，并在笛卡尔阻尼项中闭环跟踪 ref_twist。
 */
#define END_EFFECTOR_LINEAR_SPEED_MPS 0.05
#define END_EFFECTOR_REAL_SPEED_LIMIT_MPS 0.05
#define END_EFFECTOR_TARGET_POS_TOL_M 0.0025
#define END_EFFECTOR_TARGET_ORI_TOL_RAD 0.005
#define END_EFFECTOR_ARRIVAL_SPEED_TOL_MPS 0.02
#define TRAJ_TARGET_CHANGE_POS_EPS_M 0.0001
#define TRAJ_TARGET_CHANGE_ORI_EPS_RAD 0.0005

#endif /* CONFIG_LIB_H */
