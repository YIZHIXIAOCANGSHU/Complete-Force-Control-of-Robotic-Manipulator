#include "sim_interface.h"
#include <arpa/inet.h>
#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <unistd.h>

static int sock = -1;
static struct sockaddr_in server_addr;
static socklen_t addr_len = sizeof(server_addr);

enum {
  CONTROL_INPUT_PACKET_DOUBLES = 45,
  NUM_ARMS_SIM = 2,
  ARM_JOINTS_SIM = 7,
  NUM_JOINTS_SIM = 14,
  NUM_BODY_JOINTS_SIM = 3,
  CONTROL_OUTPUT_TAU_OFFSET = 0,
  CONTROL_OUTPUT_REF_POS_OFFSET = NUM_JOINTS_SIM,
  CONTROL_OUTPUT_REF_QUAT_OFFSET =
      CONTROL_OUTPUT_REF_POS_OFFSET + NUM_ARMS_SIM * 3,
  CONTROL_OUTPUT_PACKET_DOUBLES =
      CONTROL_OUTPUT_REF_QUAT_OFFSET + NUM_ARMS_SIM * 4,
  TORQUE_OUTPUT_PACKET_DOUBLES = CONTROL_OUTPUT_PACKET_DOUBLES,
  Q_OFFSET = 0,
  QD_OFFSET = 14,
  BODY_Q_OFFSET = 28,
  LEFT_TARGET_POS_OFFSET = 31,
  LEFT_TARGET_QUAT_OFFSET = 34,
  RIGHT_TARGET_POS_OFFSET = 38,
  RIGHT_TARGET_QUAT_OFFSET = 41,
};

// 缓存 Python 转发给 C 闭环控制器的输入：
// qpos(14), qvel(14), body_q(3), left_pos(3), left_quat(4), right_pos(3), right_quat(4)
static double cached_state[CONTROL_INPUT_PACKET_DOUBLES];
static double control_output_packet[CONTROL_OUTPUT_PACKET_DOUBLES];

// 辅助函数，阻塞等待接收控制输入包
static int wait_for_state(void) {
  int expected = CONTROL_INPUT_PACKET_DOUBLES * sizeof(double);
  int n = recvfrom(sock, cached_state, expected, 0, NULL, NULL);
  if (n < 0) {
    printf("[sim_interface] recvfrom failed while waiting for state: %s\n",
           strerror(errno));
    return -1;
  }
  if (n != expected) {
    printf("[sim_interface] Received incomplete/invalid state: %d bytes "
           "(expected %d)\n",
           n, expected);
    return -1;
  }
  return 0;
}

int sim_init(const char *ip, int port) {
  sock = socket(AF_INET, SOCK_DGRAM, 0);
  if (sock < 0) {
    perror("socket creation failed");
    return -1;
  }

  memset(&server_addr, 0, sizeof(server_addr));
  server_addr.sin_family = AF_INET;
  server_addr.sin_port = htons(port);
  server_addr.sin_addr.s_addr = inet_addr(ip);

  // 设置接收超时（防止死锁）
  struct timeval tv;
  tv.tv_sec = 2; // 2秒超时
  tv.tv_usec = 0;
  setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

  const char *init_msg = "INIT";
  const int max_attempts = 6;

  // Python 端启动 MuJoCo / Viewer 可能超过固定 sleep 时长。
  // 对 UDP 初始化握手进行重试，避免首个 INIT 在服务端就绪前丢失。
  for (int attempt = 1; attempt <= max_attempts; ++attempt) {
    if (sendto(sock, init_msg, strlen(init_msg), 0,
               (const struct sockaddr *)&server_addr, addr_len) < 0) {
      perror("sim_init: sendto INIT failed");
      return -1;
    }

    if (wait_for_state() == 0) {
      if (attempt > 1) {
        printf("[sim_interface] Connected after %d INIT attempts.\n", attempt);
      }
      return 0;
    }

    if (attempt < max_attempts) {
      printf("[sim_interface] Server not ready yet, retrying INIT (%d/%d)...\n",
             attempt + 1, max_attempts);
    }
  }

  return -1;
}

void sim_get_state(double *qpos, double *qvel, double ee_pos[2][3],
                   double ee_quat[2][4], double target_pos[2][3],
                   double target_quat[2][4]) {
  sim_get_control_input(qpos, qvel, NULL, target_pos, target_quat);
  if (ee_pos)
    memset(ee_pos, 0, NUM_ARMS_SIM * 3 * sizeof(double));
  if (ee_quat) {
    memset(ee_quat, 0, NUM_ARMS_SIM * 4 * sizeof(double));
    ee_quat[0][0] = 1.0;
    ee_quat[1][0] = 1.0;
  }
}

void sim_get_control_input(double *qpos, double *qvel, double *body_q,
                           double target_pos[2][3],
                           double target_quat[2][4]) {
  if (qpos)
    memcpy(qpos, cached_state + Q_OFFSET, NUM_JOINTS_SIM * sizeof(double));
  if (qvel)
    memcpy(qvel, cached_state + QD_OFFSET, NUM_JOINTS_SIM * sizeof(double));
  if (body_q)
    memcpy(body_q, cached_state + BODY_Q_OFFSET,
           NUM_BODY_JOINTS_SIM * sizeof(double));
  if (target_pos) {
    memcpy(target_pos[0], cached_state + LEFT_TARGET_POS_OFFSET, 3 * sizeof(double));
    memcpy(target_pos[1], cached_state + RIGHT_TARGET_POS_OFFSET, 3 * sizeof(double));
  }
  if (target_quat) {
    memcpy(target_quat[0], cached_state + LEFT_TARGET_QUAT_OFFSET, 4 * sizeof(double));
    memcpy(target_quat[1], cached_state + RIGHT_TARGET_QUAT_OFFSET, 4 * sizeof(double));
  }
}

int sim_apply_control_output(const double *tau, const double ref_pos[2][3],
                             const double ref_quat[2][4]) {
  if (sock < 0)
    return -1;

  memset(control_output_packet, 0, sizeof(control_output_packet));
  if (tau) {
    memcpy(control_output_packet + CONTROL_OUTPUT_TAU_OFFSET, tau,
           NUM_JOINTS_SIM * sizeof(double));
  }
  if (ref_pos) {
    memcpy(control_output_packet + CONTROL_OUTPUT_REF_POS_OFFSET, ref_pos,
           NUM_ARMS_SIM * 3 * sizeof(double));
  }
  if (ref_quat) {
    memcpy(control_output_packet + CONTROL_OUTPUT_REF_QUAT_OFFSET, ref_quat,
           NUM_ARMS_SIM * 4 * sizeof(double));
  }

  // 发送控制器输出包：14 轴力矩 + 左右参考点位置/姿态。
  int n = sendto(sock, control_output_packet,
                 CONTROL_OUTPUT_PACKET_DOUBLES * sizeof(double), 0,
                 (const struct sockaddr *)&server_addr, addr_len);
  if (n != CONTROL_OUTPUT_PACKET_DOUBLES * sizeof(double)) {
    return -1;
  }

  // Python 端收到 tau 后会由于环境推演往前步进，并传回下一帧的状态
  return wait_for_state();
}

int sim_apply_torque(const double *tau) {
  return sim_apply_control_output(tau, NULL, NULL);
}

void sim_close() {
  if (sock >= 0)
    close(sock);
  sock = -1;
}
