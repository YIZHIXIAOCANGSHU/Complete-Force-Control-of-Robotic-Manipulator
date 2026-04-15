/**
 * @file main.c
 * @brief C Controller: MuJoCo Simulation Wrapper
 * 
 * This file handles simulation-specific tasks:
 * 1. Connecting to the MuJoCo server.
 * 2. Bridging coordinate systems (Z-up vs Y-up).
 * 3. Calling the portable Robot Controller (main_stm).
 */

#include "main_stm.h"
#include "sim_interface.h"
#include "sim_bridge.h"
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
  double ee_pos[3], ee_quat[4];
  double target_pos[3], target_quat[4];
  
  stm_input_t stm_in;
  stm_output_t stm_out;

  printf("[INFO] Starting control loop...\n\n");

  /* ================================================================
   *  主控制循环
   * ================================================================ */
  while (1) {
    /* -- 3a. 获取仿真状态 (MuJoCo 坐标系: Z-up) -- */
    sim_get_state(q, qd, ee_pos, ee_quat, target_pos, target_quat);

    /* -- 3b. 坐标系桥接: MuJoCo -> Robot Coord (URDF/RBDL) -- */
    memcpy(stm_in.q, q, sizeof(q));
    memcpy(stm_in.qd, qd, sizeof(qd));
    control_mujoco_to_rbdl(target_pos, target_quat, stm_in.target_pos, stm_in.target_quat);

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
