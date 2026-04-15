#ifndef STM_CONTROLLER_H
#define STM_CONTROLLER_H

#include "config.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 单次控制循环的输入
 *
 * 所有量都应使用控制器内部约定的单位:
 * - 关节角: rad
 * - 关节角速度: rad/s
 * - 末端位置: m
 * - 姿态四元数: (w, x, y, z)
 */
typedef struct {
  double q[NUM_JOINTS];    /* 当前关节角 */
  double qd[NUM_JOINTS];   /* 当前关节角速度 */
  double target_pos[3];    /* 目标末端位置 */
  double target_quat[4];   /* 目标末端姿态四元数 (w, x, y, z) */
} stm_input_t;

/**
 * @brief 单次控制循环的输出
 */
typedef struct {
  double tau[NUM_JOINTS];  /* 输出关节力矩 */
  int status;              /* 0=正常, 负数=安全检查失败 */
  double ee_pos[3];        /* 当前末端位置 */
  double ee_quat[4];       /* 当前末端姿态四元数 (w, x, y, z) */
  double traj_t;           /* 控制器内部累计轨迹时间 */
  int step_count;          /* 累计步数 */
  double calc_time_ms;     /* 单步计算耗时，需提供平台时钟后才有效 */
} stm_output_t;

/* 平台毫秒计时函数类型；可对接 HAL_GetTick 或高精度定时器 */
typedef double (*stm_now_ms_fn)(void *user_ctx);

/**
 * @brief 平台相关钩子
 *
 * 控制核心本身不依赖具体 MCU 平台，通过这个结构挂接可选能力。
 */
typedef struct {
  stm_now_ms_fn now_ms; /* 获取当前毫秒时间 */
  void *user_ctx;       /* 传给 now_ms 的上下文指针 */
} stm_platform_hooks_t;

/**
 * @brief 初始化控制器
 *
 * 在系统启动时调用一次即可。重复调用是安全的。
 */
void stm_controller_init(void);

/**
 * @brief 执行一步控制计算
 *
 * 常规使用时在固定周期中断或控制线程中循环调用。
 */
void stm_controller_step(const stm_input_t *in, stm_output_t *out);

/**
 * @brief 执行一步“仅重力补偿”计算
 *
 * 这个接口保留与 `stm_controller_step()` 相同的输入输出形式，但不会执行
 * 笛卡尔目标跟踪和路径规划，只输出当前姿态下的重力补偿力矩。
 */
void stm_controller_step_gravity_only(const stm_input_t *in, stm_output_t *out);

/**
 * @brief 清空内部轨迹和计数状态
 *
 * 适合在急停恢复、重新规划起点时调用。
 */
void stm_controller_reset(void);

/**
 * @brief 设置平台钩子
 *
 * 这是可选接口。若不设置，控制器仍可正常工作，只是 `calc_time_ms`
 * 会保持为 0。
 */
void stm_controller_set_platform_hooks(const stm_platform_hooks_t *hooks);

#ifdef __cplusplus
}
#endif

#endif /* STM_CONTROLLER_H */
