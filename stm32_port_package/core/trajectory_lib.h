/*
 * AM-D02 机器人轨迹规划库
 * 简化以匹配 Python 的梯形速度规划。
 * 扩展了三维直线路径规划器 (LinearPathPlanner)。
 */

#ifndef TRAJECTORY_LIB_H
#define TRAJECTORY_LIB_H

#include "model_lib.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ================================================================
 *  单轴往复运动轨迹规划器 (原有，保持不变)
 * ================================================================ */

/**
 * @brief 轨迹规划器配置与状态 (梯形速度曲线, 单轴往复)
 */
typedef struct {
  double start_pos[3]; /* 运动起点坐标 */
  double amplitude;    /* 往复运动的单边振幅 */
  double speed;        /* 设定的最大运行速度 */
  double accel;        /* 设定的加速度 */
  int axis;            /* 运动轴: 0=X, 1=Y, 2=Z */
  int direction;       /* 运动方向标志位: 1 或 -1 */

  /* 预计算的梯形参数 */
  double L;           /* 规划总距离 */
  double v_max;       /* 实际能达到的最大速度 */
  double a;           /* 实际使用的加速度 */
  double t_a;         /* 加速段时间 */
  double t_c;         /* 匀速段时间 */
  double d_a;         /* 加速段距离 */
  double d_c;         /* 匀速段距离 */
  double half_period; /* 半周期运动时间 (单程) */
  double period;      /* 完整的往复周期时间 */
} TrajectoryPlanner;

/* 初始化单轴往复运动轨迹 */
void trajectory_init(TrajectoryPlanner *tp, const double start_pos[3], int axis,
                     double amplitude, double speed, double accel);

/* 获取在时间 t 时的期望位置和速度 */
void trajectory_evaluate(TrajectoryPlanner *tp, double t, double pos[3],
                         double vel[3]);

/* ================================================================
 *  三维直线路径规划器 (梯形速度曲线)
 *  从 start_pos 到 end_pos 规划一段直线路径，支持实时重规划。
 * ================================================================ */

/**
 * @brief 3D直线路径规划器状态
 */
typedef struct {
  double start_pos[3];  /* 路径起点 (m) */
  double end_pos[3];    /* 路径终点 (m) */
  double start_quat[4]; /* 姿态起点 */
  double end_quat[4];   /* 姿态终点 */
  double dir[3];        /* 单位方向向量 (end - start) */
  double L;             /* 总距离 (m) */
  /* 梯形参数 */
  double v_max;      /* 实际最大速度 (m/s) */
  double a;          /* 加速度 (m/s^2) */
  double t_a;        /* 加速段时长 (s) */
  double t_c;        /* 匀速段时长 (s) */
  double d_a;        /* 加速段距离 (m) */
  double total_time; /* 全程时长 (s) */
} LinearPathPlanner;

/**
 * @brief 初始化/重规划3D直线路径
 * @param lp        规划器实例
 * @param start_pos 当前末端实际位置（路径起点）
 * @param end_pos   路径终点（目标位置）
 * @param speed     期望运动速度 (m/s)
 * @param accel     加速度 (m/s^2)
 */
void linear_path_init(LinearPathPlanner *lp, const double start_pos[3],
                      const double start_quat[4], const double end_pos[3],
                      const double end_quat[4], double speed, double accel);

/**
 * @brief 查询路径上 t 时刻的期望位置（超出total_time后保持终点）
 * @param lp   规划器实例
 * @param t    自路径开始的时间 (s)
 * @param pos  输出: 期望位置 [x,y,z]
 * @param quat 输出: 期望姿态 [w,x,y,z]
 */
void linear_path_evaluate(const LinearPathPlanner *lp, double t, double pos[3],
                          double quat[4]);

/**
 * @brief 判断当前末端是否已接近路径终点
 * @param lp        规划器实例
 * @param ee_pos    当前末端实际位置
 * @param threshold 到达阈值 (m)
 * @return 1=已到达, 0=未到达
 */
int linear_path_reached(const LinearPathPlanner *lp, const double ee_pos[3],
                        double threshold);

#ifdef __cplusplus
}
#endif

#endif /* TRAJECTORY_LIB_H */
