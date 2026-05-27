/**
 * @file main.c
 * @brief C Controller: MuJoCo Simulation Wrapper
 * 
 * This file handles simulation-specific tasks:
 * 1. Connecting to the MuJoCo server.
 * 2. Forwarding MuJoCo feedback/target packets to the C closed loop.
 * 3. Applying the fourteen joint torques returned by main_stm.
 */

#include "main_stm.h"
#include "sim_interface.h"
#include "h7_clock_sim.h"
#include <math.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <stdlib.h>

static int wait_for_h7_due_ticks(h7_clock_sim_t *h7_clock) {
  int due_ticks = h7_clock_sim_due_ticks(h7_clock);
  while (due_ticks == 0) {
    usleep(100);
    due_ticks = h7_clock_sim_due_ticks(h7_clock);
  }
  return due_ticks;
}

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
  h7_clock_sim_t h7_clock;
  double last_tau[NUM_JOINTS] = {0};

  h7_clock_sim_init(&h7_clock, CONTROL_DT);
  memset(&stm_out, 0, sizeof(stm_out));

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
    memcpy(stm_in.body_q, body_q, sizeof(body_q));
    memcpy(stm_in.target_pos, target_pos, sizeof(target_pos));
    memcpy(stm_in.target_quat, target_quat, sizeof(target_quat));

    /* -- 3c. 按独立 H7 控制时钟补跑控制 tick，只保留最后一次力矩 -- */
    int due_ticks = wait_for_h7_due_ticks(&h7_clock);
    for (int tick = 0; tick < due_ticks; ++tick) {
      stm_step(&stm_in, &stm_out);
    }
    if (due_ticks > 0) {
      memcpy(last_tau, stm_out.tau, sizeof(last_tau));
    }

    /* -- 3d. 安全检查处理 -- */
    if (due_ticks > 0 && stm_out.status < 0) {
      printf("[SAFETY] Limit violated! EMERGENCY STOP.\n");
      exit(1);
    }

    /* -- 3e. 发送力矩，步进仿真 -- */
    if (sim_apply_torque(last_tau) < 0) {
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
