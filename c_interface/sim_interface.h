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
 * 获取最新 C 控制输入
 * 所有参数均为输出数组。如果不需要某个值可以传 NULL。
 * 状态在底层通过 sim_apply_torque 时会自动与 Python 端同步并缓存。
 *
 * @param qpos 关节位置 (14维)
 * @param qvel 关节速度 (14维)
 * @param body_q 躯干关节角度 (3维: Waist01, Waist02, Body0422)
 * @param target_pos 左右目标 TCP 位置 (Body0422 动态目标坐标, 2x3)
 * @param target_quat 左右目标 TCP 姿态四元数 [w,x,y,z] (2x4)
 * @param dt_s Python/MuJoCo 仿真步长 (秒)
 */
void sim_get_control_input(double *qpos, double *qvel, double *body_q,
                           double target_pos[2][3],
                           double target_quat[2][4], double *dt_s);

void sim_get_state(double *qpos, double *qvel, double ee_pos[2][3],
                   double ee_quat[2][4], double target_pos[2][3],
                   double target_quat[2][4]);

/**
 * 发送控制力矩到仿真环境，并步进仿真。
 * 阻塞等待下一个仿真步的状态返回，从而更新内部的缓存状态。
 * @param tau 关节力矩 (14维)
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
