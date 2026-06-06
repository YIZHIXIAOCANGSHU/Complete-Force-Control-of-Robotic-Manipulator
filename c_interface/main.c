/**
 * @file main.c
 * @brief C Controller: MuJoCo Simulation Wrapper
 * 
 * This file handles simulation-specific tasks:
 * 1. Connecting to the MuJoCo server.
 * 2. Forwarding MuJoCo feedback/target packets to the C closed loop.
 * 3. Applying the control output returned by main_stm.
 */

#include "main_stm.h"
#include "h7_clock_sim.h"
#include "sim_interface.h"
#include <stdio.h>
#include <string.h>

int main(void) {
  printf("=========================================\n");
  printf("   C Controller: MuJoCo Wrapper (Sim)    \n");
  printf("=========================================\n");

  /* 1. 连接仿真服务器 */
  if (sim_init("127.0.0.1", 9876) < 0) {
    printf("[ERROR] Failed to connect to simulation server!\n");
    return -1;
  }
  printf("[INFO] Connected to simulation server.\n\n");

  /* 2. 初始化底层控制器 */
  stm_init();
  printf("[INFO] Controller ready.\n\n");

  /* 状态变量 */
  double q[NUM_JOINTS], qd[NUM_JOINTS];
  double body_q[NUM_BODY_JOINTS];
  double target_pos[NUM_ARMS][3], target_quat[NUM_ARMS][4];
  
  stm_input_t stm_in;
  stm_output_t stm_out;
  h7_clock_sim_t clock;
  int last_status = STM_STATUS_OK;

  memset(&stm_out, 0, sizeof(stm_out));
  h7_clock_sim_init(&clock);

  printf("[INFO] Starting control loop...\n\n");

  /* ================================================================
   *  主控制循环
   * ================================================================ */
  while (1) {
    /* -- 3a. 获取 Python 转发的反馈和 Body0422 动态目标坐标 -- */
    sim_get_control_input(q, qd, body_q, target_pos, target_quat);

    /* -- 3b. C 控制器执行完整闭环，Python 不参与控制逻辑 -- */
    memcpy(stm_in.q, q, sizeof(q));
    memcpy(stm_in.qd, qd, sizeof(qd));
    stm_in.active_arm_mask = STM_ARM_MASK_BOTH;
    memcpy(stm_in.body_q, body_q, sizeof(body_q));
    memcpy(stm_in.target_pos, target_pos, sizeof(target_pos));
    memcpy(stm_in.target_quat, target_quat, sizeof(target_quat));

    /* -- 3c. 使用 host/H7 本地墙钟 elapsed 推进控制器内部时间。
     * C 端内部用该时间推进 S 曲线参考点，同时用于 traj_t 和安全恢复计时。
     */
    double elapsed_s = h7_clock_sim_elapsed_s(&clock);
    stm_step_elapsed(&stm_in, &stm_out, elapsed_s);

    /* -- 3d. 安全状态提示。仿真不退出，继续发送控制器给出的 0 力矩。 -- */
    if (stm_out.status != last_status) {
      if (stm_out.status == STM_STATUS_SAFETY_LATCHED) {
        printf("[SAFETY] Entered safety latch. Sending zero torque while waiting for recovery.\n");
      } else if (stm_out.status == STM_STATUS_WAITING_ZERO) {
        printf("[SAFETY] Waiting for all arm joint velocities to stay near zero.\n");
      } else if (stm_out.status == STM_STATUS_OK && last_status < 0) {
        printf("[SAFETY] Recovery complete. Control resumed.\n");
      }
      last_status = stm_out.status;
    }

    /* -- 3e. 发送控制输出，步进仿真 -- */
    if (sim_apply_control_output(stm_out.tau, stm_out.ref_pos,
                                 stm_out.ref_quat) < 0) {
      printf("[ERROR] Connection lost or simulation closed.\n");
      break;
    }

    /* -- 3f. 定期打印状态 -- */
    if (stm_out.step_count % 500 == 0) {
      printf("[Step %6d] t=%.3fs | L=[%.3f %.3f %.3f] R=[%.3f %.3f %.3f] | tauL0=%.3f tauR0=%.3f\n",
             stm_out.step_count, stm_out.traj_t, 
             stm_out.ee_pos[ARM_LEFT][0], stm_out.ee_pos[ARM_LEFT][1],
             stm_out.ee_pos[ARM_LEFT][2],
             stm_out.ee_pos[ARM_RIGHT][0], stm_out.ee_pos[ARM_RIGHT][1],
             stm_out.ee_pos[ARM_RIGHT][2],
             stm_out.tau[0], stm_out.tau[ARM_JOINTS]);
    }
  }

  sim_close();
  printf("[INFO] Control application exited.\n");
  return 0;
}
