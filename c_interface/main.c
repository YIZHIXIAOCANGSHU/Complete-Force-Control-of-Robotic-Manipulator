/**
 * @file main.c
 * @brief C Controller: MuJoCo Simulation Wrapper
 * 
 * This file handles simulation-specific tasks:
 * 1. Connecting to the MuJoCo server.
 * 2. Forwarding MuJoCo feedback/target packets to the C closed loop.
 * 3. Applying the seven joint torques returned by main_stm.
 */

#include "main_stm.h"
#include "sim_interface.h"
#include <math.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <stdlib.h>

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
  double q[7], qd[7];
  double target_pos[3], target_quat[4];
  double dt_s;
  
  stm_input_t stm_in;
  stm_output_t stm_out;

  printf("[INFO] Starting control loop...\n\n");

  /* ================================================================
   *  主控制循环
   * ================================================================ */
  while (1) {
    /* -- 3a. 获取 Python 转发的反馈和 base_link 目标 -- */
    sim_get_control_input(q, qd, target_pos, target_quat, &dt_s);

    /* -- 3b. C 控制器执行完整闭环，Python 不参与控制逻辑 -- */
    memcpy(stm_in.q, q, sizeof(q));
    memcpy(stm_in.qd, qd, sizeof(qd));
    memcpy(stm_in.target_pos, target_pos, sizeof(target_pos));
    memcpy(stm_in.target_quat, target_quat, sizeof(target_quat));
    stm_in.dt_s = dt_s;

    /* -- 3c. 执行底层控制步进 (Portable Logic) -- */
    stm_step(&stm_in, &stm_out);

    /* -- 3d. 安全检查处理 -- */
    if (stm_out.status < 0) {
      printf("[SAFETY] Limit violated! EMERGENCY STOP.\n");
      exit(1);
    }

    /* -- 3e. 发送力矩，步进仿真 -- */
    if (sim_apply_torque(stm_out.tau) < 0) {
      printf("[ERROR] Connection lost or simulation closed.\n");
      break;
    }

    /* -- 3f. 定期打印状态 -- */
    if (stm_out.step_count % 500 == 0) {
      printf("[Step %6d] t=%.3fs | ee=[%.3f %.3f %.3f] | tau[0]=%.3f\n",
             stm_out.step_count, stm_out.traj_t, 
             stm_out.ee_pos[0], stm_out.ee_pos[1], stm_out.ee_pos[2],
             stm_out.tau[0]);
    }
  }

  sim_close();
  printf("[INFO] Control application exited.\n");
  return 0;
}
