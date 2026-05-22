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
  CONTROL_INPUT_PACKET_DOUBLES = 22,
  TORQUE_OUTPUT_PACKET_DOUBLES = 7,
};

// 缓存 Python 转发给 C 闭环控制器的输入：
// qpos(7), qvel(7), target_pos_base(3), target_quat_base(4), dt_s(1)
static double cached_state[CONTROL_INPUT_PACKET_DOUBLES];

// 辅助函数，阻塞等待接收28个double
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

void sim_get_state(double *qpos, double *qvel, double *ee_pos, double *ee_quat,
                   double *target_pos, double *target_quat) {
  double unused_dt = 0.0;
  sim_get_control_input(qpos, qvel, target_pos, target_quat, &unused_dt);
  if (ee_pos)
    memset(ee_pos, 0, 3 * sizeof(double));
  if (ee_quat) {
    ee_quat[0] = 1.0;
    ee_quat[1] = 0.0;
    ee_quat[2] = 0.0;
    ee_quat[3] = 0.0;
  }
}

void sim_get_control_input(double *qpos, double *qvel, double *target_pos,
                           double *target_quat, double *dt_s) {
  if (qpos)
    memcpy(qpos, cached_state, 7 * sizeof(double));
  if (qvel)
    memcpy(qvel, cached_state + 7, 7 * sizeof(double));
  if (target_pos)
    memcpy(target_pos, cached_state + 14, 3 * sizeof(double));
  if (target_quat)
    memcpy(target_quat, cached_state + 17, 4 * sizeof(double));
  if (dt_s)
    *dt_s = cached_state[21];
}

int sim_apply_torque(const double *tau) {
  if (sock < 0)
    return -1;

  // 发送 7 个 double 到服务器：C 闭环控制器输出的七轴力矩。
  int n = sendto(sock, tau, TORQUE_OUTPUT_PACKET_DOUBLES * sizeof(double), 0,
                 (const struct sockaddr *)&server_addr, addr_len);
  if (n != TORQUE_OUTPUT_PACKET_DOUBLES * sizeof(double)) {
    return -1;
  }

  // Python 端收到 tau 后会由于环境推演往前步进，并传回下一帧的状态
  return wait_for_state();
}

void sim_close() {
  if (sock >= 0)
    close(sock);
  sock = -1;
}
