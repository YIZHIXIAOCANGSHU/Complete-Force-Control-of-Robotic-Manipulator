#ifndef SIM_INTERFACE_H
#define SIM_INTERFACE_H

#ifdef __cplusplus
extern "C" {
#endif

/**
 * 初始化并阻塞等待 Python 服务端的初始状态数据
 * @param ip 服务器IP地址 (如 "127.0.0.1")
 * @param port 服务器通信端口 (如 9876)
 * @return 成功返回0, 失败返回-1
 */
int sim_init(const char *ip, int port);

/**
 * 获取最新状态
 * 所有参数均为输出数组。如果不需要某个值可以传 NULL。
 * 状态在底层通过 sim_apply_torque 时会自动与 Python 端同步并缓存。
 *
 * @param qpos 关节位置 (7维)
 * @param qvel 关节速度 (7维)
 * @param ee_pos 末端当前位置 (3维)
 * @param ee_quat 末端当前姿态四元数 (4维)
 * @param target_pos 目标末端位置 (3维)
 * @param target_quat 目标末端姿态四元数 (4维)
 */
void sim_get_state(double *qpos, double *qvel, double *ee_pos, double *ee_quat,
                   double *target_pos, double *target_quat);

/**
 * 发送控制力矩到仿真环境，并步进仿真。
 * 阻塞等待下一个仿真步的状态返回，从而更新内部的缓存状态。
 * @param tau 关节力矩 (7维)
 * @return 成功返回0, 超时或失败返回-1
 */
int sim_apply_torque(const double *tau);

/**
 * 关闭连接
 */
void sim_close();

#ifdef __cplusplus
}
#endif

#endif // SIM_INTERFACE_H
