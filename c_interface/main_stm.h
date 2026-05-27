/**
 * @file main_stm.h
 * @brief 兼容旧入口的 STM 控制器包装层
 *
 * 真正的可移植控制核心位于 stm32_code/stm_controller.*。
 * 本头文件继续保留 stm_init/stm_step 接口，避免上层调用方改动过大。
 */

#ifndef MAIN_STM_H
#define MAIN_STM_H

#include "stm_controller.h"

/**
 * @brief 初始化底层控制器
 */
void stm_init(void);

/**
 * @brief 执行一步控制循环
 * 
 * @param in  传感器输入与目标
 * @param out 力矩输出与状态
 */
void stm_step(const stm_input_t* in, stm_output_t* out);

/**
 * @brief 执行一步控制循环，路径规划使用 host/H7 本地 elapsed time。
 *
 * @param in        传感器输入与目标
 * @param out       力矩输出与状态
 * @param elapsed_s 距离上一轮 sim 控制计算的 H7 本地时间，单位秒
 */
void stm_step_elapsed(const stm_input_t* in, stm_output_t* out,
                      double elapsed_s);

/**
 * @brief 重置步数和内部时间状态
 */
void stm_reset(void);

/**
 * @brief 设置平台相关钩子，例如毫秒计时源
 */
void stm_set_platform_hooks(const stm_platform_hooks_t *hooks);

#endif /* MAIN_STM_H */
