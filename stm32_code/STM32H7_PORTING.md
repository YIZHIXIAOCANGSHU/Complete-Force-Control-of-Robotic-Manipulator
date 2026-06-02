# STM32H7 移植须知

`stm32_code` 是可直接移植到 STM32H7 工程的控制核心。它只负责根据关节状态、躯干角、末端目标和外部传入的真实时间间隔计算力矩，不负责串口、CAN、UDP、MuJoCo、Rerun 或 PC 时钟。

唯一高层入口是：

```c
stm_controller_step_elapsed(&in, &out, elapsed_s);
```

外部 STM32H7 工程负责：

- 读取左右臂 14 轴 `q/qd`
- 读取或估计 3 个躯干角 `body_q`
- 选择本轮控制左臂、右臂或双臂
- 填充左右末端目标
- 用本地硬件时钟计算 `elapsed_s`
- 根据 `out.status` 决定是否下发 `out.tau[14]`

## 移植文件清单

建议只把 `stm32_code` 下列文件加入 STM32H7 工程：

- `config.h`
- `stm_controller.h/.c`
- `control_logic.h/.c`
- `dynamics_lib.h/.c`
- `kinematics_lib.h/.c`
- `model_lib.h/.c`
- `math_lib.h/.c`

不要移植：

- `c_interface/*`
- `python/*`
- `tests/*`
- MuJoCo / Rerun 相关文件
- `c_interface/h7_clock_sim/*`

`h7_clock_sim` 只是 PC 仿真用的 1 MHz 微秒时基示例，不是真机必需模块。真机 UART、CAN、USB、RTOS 消息队列等通信协议也不属于 `stm32_code` 控制核心，请在 STM32H7 平台工程里单独维护。

## 初始化与主循环

系统启动时初始化一次：

```c
stm_controller_init();
```

每一轮控制循环：

```c
stm_input_t in = {0};
stm_output_t out = {0};

/* 1. 读取传感器并填充 in.q / in.qd / in.body_q */
/* 2. 设置 in.active_arm_mask */
/* 3. 填充 in.target_pos / in.target_quat */
/* 4. 用本地硬件时钟计算 elapsed_s */

stm_controller_step_elapsed(&in, &out, elapsed_s);

if (out.status == STM_STATUS_OK) {
  /* 下发 out.tau[0..13] */
} else {
  /* 安全锁定或恢复中：停止正常力矩，或下发 out.tau 中的 0 力矩 */
}
```

需要清空控制器内部状态和安全状态时调用：

```c
stm_controller_reset();
```

## 时基传入

`stm32_code` 不读取硬件定时器，也不假设 1 MHz。控制核心只接收本轮真实经过时间：

```c
double elapsed_s;
```

语义：

| 字段 | 含义 |
| --- | --- |
| 单位 | `s` |
| 来源 | STM32H7 平台层根据本地定时器换算 |
| 作用 | 推进内部 TCP 参考点、累计 `out.traj_t`，并用于安全恢复计时 |

典型写法：

```c
uint32_t now_ticks = read_timer_ticks();
uint32_t dt_ticks = now_ticks - last_ticks;
last_ticks = now_ticks;

double elapsed_s = (double)dt_ticks / timer_frequency_hz;
stm_controller_step_elapsed(&in, &out, elapsed_s);
```

`timer_frequency_hz` 由你的平台工程决定：

| 时钟源 | `timer_frequency_hz` 示例 |
| --- | --- |
| TIM 配成 1 MHz | `1000000.0` |
| TIM 配成 10 kHz | `10000.0` |
| DWT cycle counter | `(double)SystemCoreClock` |
| RTOS tick | RTOS tick 频率 |

控制核心没有固定周期，也不会对大的 `elapsed_s` 做上限裁剪：

- 负数或非有限值：按 `0.0` 处理
- 其他有限非负值：按平台层传入的真实时间完整累计

如果你的平台需要处理调试暂停或通信卡顿后的大时间跳变，请在平台层决定是否丢弃、限幅或重置控制器。

## 输入结构体

定义位置：`stm32_code/stm_controller.h`

```c
typedef struct {
  double q[NUM_JOINTS];
  double qd[NUM_JOINTS];
  uint8_t active_arm_mask;
  double body_q[NUM_BODY_JOINTS];
  double target_pos[NUM_ARMS][3];
  double target_quat[NUM_ARMS][4];
} stm_input_t;
```

每一帧必须由平台层完整填充 `stm_input_t`。控制核心不会主动读取电机、CAN、定时器或上位机状态。

| 字段 | 是否必填 | 单位/格式 | 控制核心如何使用 |
| --- | --- | --- | --- |
| `q[14]` | 必填 | `rad` | FK、Jacobian、动力学补偿、关节位置安全检查 |
| `qd[14]` | 必填 | `rad/s` | 速度滤波、`J(q)*qd` TCP 速度估计、速度安全检查、安全恢复 |
| `active_arm_mask` | 必填，`0` 兼容为双臂 | bit mask | 决定本轮哪些臂参与 FK、控制和安全检查 |
| `body_q[3]` | 建议必填 | `rad` | 更新躯干姿态导致的重力方向；非有限值时沿用上次重力上下文 |
| `target_pos[2][3]` | 必填 | `m` | 每臂最终 TCP 目标位置，内部会从当前 TCP 重新规划 S 曲线 |
| `target_quat[2][4]` | 必填 | `wxyz` 四元数 | 每臂最终 TCP 目标姿态，内部归一化后用于 slerp 和姿态阻抗 |

调用方还必须单独传入 `elapsed_s`。它不在 `stm_input_t` 里，是
`stm_controller_step_elapsed(&in, &out, elapsed_s)` 的第三个参数。

当前常量：

| 宏 | 值 | 含义 |
| --- | --- | --- |
| `ARM_JOINTS` | `7` | 单臂关节数 |
| `NUM_ARMS` | `2` | 左右两臂 |
| `NUM_JOINTS` | `14` | 受控双臂总关节数 |
| `NUM_BODY_JOINTS` | `3` | 躯干角输入数量 |
| `ARM_LEFT` | `0` | 左臂索引 |
| `ARM_RIGHT` | `1` | 右臂索引 |

### `active_arm_mask`

本轮控制哪只手臂。未激活臂会完全跳过 FK/Jacobian、安全检查和控制计算，输出力矩保持 0。

| 宏 | 值 | 行为 |
| --- | --- | --- |
| `STM_ARM_MASK_LEFT` | `1 << ARM_LEFT` | 只控制左臂 |
| `STM_ARM_MASK_RIGHT` | `1 << ARM_RIGHT` | 只控制右臂 |
| `STM_ARM_MASK_BOTH` | 左右按位或 | 同时控制左右臂 |
| `0` | 默认兼容 | 等价于 `STM_ARM_MASK_BOTH` |

示例：

```c
in.active_arm_mask = STM_ARM_MASK_LEFT;  /* 只输出 tau[0..6] */
in.active_arm_mask = STM_ARM_MASK_RIGHT; /* 只输出 tau[7..13] */
in.active_arm_mask = STM_ARM_MASK_BOTH;  /* 输出 tau[0..13] */
```

注意：未激活臂的 `tau`、`ee_pos`、`ee_quat` 会保持 0；如果上层需要显示未激活臂状态，应在平台层自行保留上一帧或单独计算。

### `q[14]`

左右臂 14 轴关节角，单位 `rad`。

| 索引 | 含义 |
| --- | --- |
| `q[0]..q[6]` | 左臂 J1..J7 |
| `q[7]..q[13]` | 右臂 J1..J7 |

代码索引：

```c
double *q_left = &in.q[ARM_LEFT * ARM_JOINTS];   /* q[0] */
double *q_right = &in.q[ARM_RIGHT * ARM_JOINTS]; /* q[7] */
```

### `qd[14]`

左右臂 14 轴关节角速度，单位 `rad/s`。

排列与 `q[14]` 一致：

| 索引 | 含义 |
| --- | --- |
| `qd[0]..qd[6]` | 左臂 J1..J7 |
| `qd[7]..qd[13]` | 右臂 J1..J7 |

速度限位和安全恢复都使用这个数组。单臂模式下，未激活臂的速度不会参与本轮安全判断。

### `body_q[3]`

躯干 3 轴角度，单位 `rad`。控制器用它在 C 端更新左右臂模型的重力方向。

| 索引 | 关节 |
| --- | --- |
| `body_q[0]` | `Waist01_Joint` |
| `body_q[1]` | `Waist02_Joint` |
| `body_q[2]` | `Body0422_Joint` |

如果真机暂时没有躯干运动，可以填 `{0.0, 0.0, 0.0}`。当前 PC SocketCAN real C 后端就是固定填零位；后续如果真机躯干可动，只需要把真实 `body_q` 接入同一字段。

### `target_pos[2][3]`

左右末端目标位置，单位 `m`。

| 第一维 | 含义 |
| --- | --- |
| `target_pos[ARM_LEFT]` | 左臂目标位置 |
| `target_pos[ARM_RIGHT]` | 右臂目标位置 |

| 第二维 | 含义 |
| --- | --- |
| `[0]` | X |
| `[1]` | Y |
| `[2]` | Z |

坐标系是 Body0422 动态目标坐标系：

- 原点跟随 `Body0422_Link`
- 坐标轴跟随 Body0422 相对零位旋转
- 不是 MuJoCo world，也不是 viewer 屏幕坐标

### `target_quat[2][4]`

左右末端目标姿态，四元数顺序为 `wxyz`，无单位。

| 第一维 | 含义 |
| --- | --- |
| `target_quat[ARM_LEFT]` | 左臂目标姿态 |
| `target_quat[ARM_RIGHT]` | 右臂目标姿态 |

| 第二维 | 含义 |
| --- | --- |
| `[0]` | w |
| `[1]` | x |
| `[2]` | y |
| `[3]` | z |

坐标系与 `target_pos` 相同。控制器内部会对目标四元数做归一化；如果目标为非有限值，会退回到上一帧目标或当前 TCP 姿态。

### 目标合法化和回退策略

目标输入允许短时间异常，但平台层不要依赖这个机制长期兜底：

- `target_pos[arm]` 全部有限：使用本帧目标位置
- `target_pos[arm]` 非有限：优先沿用该臂上一帧有效目标；没有历史目标时退回当前 TCP 位置
- `target_quat[arm]` 全部有限且范数有效：归一化后使用
- `target_quat[arm]` 非有限或范数过小：优先沿用上一帧有效目标姿态；没有历史目标时退回当前 TCP 姿态
- 新目标与当前路径目标差异超过 `TRAJ_TARGET_CHANGE_POS_EPS_M` 或 `TRAJ_TARGET_CHANGE_ORI_EPS_RAD` 时，控制器立即从当前实际 TCP 重规划

## 输出结构体

定义位置：`stm32_code/stm_controller.h`

```c
typedef struct {
  double tau[NUM_JOINTS];
  int status;
  double ee_pos[NUM_ARMS][3];
  double ee_quat[NUM_ARMS][4];
  double traj_t;
  int step_count;
} stm_output_t;
```

每次调用都会重写 `stm_output_t`。移植时建议平台层只在 `status == STM_STATUS_OK` 时下发正常力矩；`status < 0` 时按安全状态处理。

| 字段 | 单位/格式 | 含义 | 平台层建议动作 |
| --- | --- | --- | --- |
| `tau[14]` | `Nm` | 左 7 轴 + 右 7 轴期望力矩 | `status == OK` 时下发激活臂力矩；非激活臂和安全态为 0 |
| `status` | 枚举整数 | 本轮控制/安全状态 | `< 0` 时禁止继续正常力矩控制 |
| `ee_pos[2][3]` | `m` | 控制器 FK 得到的实际 TCP 位置 | 用于调试、日志、上位机显示 |
| `ee_quat[2][4]` | `wxyz` | 控制器 FK 得到的实际 TCP 姿态 | 用于调试、日志、上位机显示 |
| `traj_t` | `s` | 控制器内部累计时间 | 用于诊断 elapsed 是否正确传入 |
| `step_count` | 次 | 控制器调用次数 | 用于诊断主循环是否稳定运行 |

### `tau[14]`

左右臂 14 轴输出力矩，单位 `Nm`。

| 索引 | 含义 |
| --- | --- |
| `tau[0]..tau[6]` | 左臂 J1..J7 力矩 |
| `tau[7]..tau[13]` | 右臂 J1..J7 力矩 |

单臂模式：

- `STM_ARM_MASK_LEFT`：`tau[0..6]` 正常计算，`tau[7..13] = 0`
- `STM_ARM_MASK_RIGHT`：`tau[0..6] = 0`，`tau[7..13]` 正常计算
- `STM_ARM_MASK_BOTH` 或 `0`：左右臂都计算

当 `status < 0` 时，控制器会输出全 0 力矩。真机平台层仍应把 `status < 0` 当作安全状态处理。

### `status`

| 值 | 含义 |
| --- | --- |
| `STM_STATUS_OK` (`0`) | 本轮控制成功，激活臂的 `tau` 可正常下发 |
| `STM_STATUS_SAFETY_LATCHED` (`-1`) | 本轮刚触发安全锁定，`tau[14]` 已置 0 |
| `STM_STATUS_WAITING_ZERO` (`-2`) | 安全恢复中，等待触发锁定的激活臂速度静止，`tau[14]` 继续为 0 |

### `ee_pos[2][3]`

控制器计算出的左右 TCP 实际位置，单位 `m`，坐标系与目标一致。

单臂模式下，只有激活臂的 `ee_pos` 会更新；未激活臂保持 0。

### `ee_quat[2][4]`

控制器计算出的左右 TCP 实际姿态，四元数顺序为 `wxyz`。

单臂模式下，只有激活臂的 `ee_quat` 会更新；未激活臂保持 0。

### `traj_t`

控制累计时间，单位 `s`。它由每轮传入的 `elapsed_s` 累加得到，用于调试、日志、内部参考推进和安全恢复计时观察。

### `step_count`

控制器步数计数。每调用一次 `stm_controller_step_elapsed()` 增加 1。

## PC real_controller 二进制桥接说明

`c_interface/real/real_controller.c` 是 PC/Python 真机链路使用的子进程包装层，不是 STM32H7 必须移植的接口。它通过 stdin/stdout 收发 packed 二进制包，再把字段搬运到 `stm_input_t/stm_output_t`。

当前 PC 输入包比 STM 核心多三个包装字段：

| 字段 | 含义 |
| --- | --- |
| `magic = 0xAA55` | 包同步/合法性检查 |
| `mode` | `REAL_MODE_CONTROL` 执行控制，`REAL_MODE_FK_ONLY` 只算 FK |
| `elapsed_s` | 转交给 `stm_controller_step_elapsed()` 的真实时间 |

其余数据与 STM 核心输入一致：`active_arm_mask`、`q[14]`、`qd[14]`、`body_q[3]`、`target_pos[2][3]`、`target_quat[2][4]`。

当前 PC 输出包也只是包装了核心输出：`status`、`tau[14]`、`ee_pos[2][3]`、`ee_quat[2][4]`、`traj_t`、`step_count`。如果你直接移植到 STM32H7，不需要保留 `magic/mode` 或 stdin/stdout 逻辑。

## 默认躯干角和目标位姿

下面这组默认值可用于 STM32H7 移植初期的上电测试：躯干保持零位，左右目标 TCP 使用当前模型 `INIT_QPOS` 的正向运动学结果。坐标系是 C 控制核心使用的 Body0422 动态目标坐标系，不是 MuJoCo world/base 坐标。

生成该目标的参考关节角为：

```c
/* 每臂 INIT_QPOS: [0, 0, 0, pi/2, 0, 0, 0] */
```

可直接放到平台层作为默认输入：

```c
static const double DEFAULT_BODY_Q[NUM_BODY_JOINTS] = {
  0.0, 0.0, 0.0
};

static const double DEFAULT_TARGET_POS[NUM_ARMS][3] = {
  /* left TCP, m */
  {0.382107999999, 0.143800000008, 0.091749500000},
  /* right TCP, m */
  {0.384108008443, -0.283898698965, 0.092348813107},
};

static const double DEFAULT_TARGET_QUAT[NUM_ARMS][4] = {
  /* left TCP, wxyz */
  {0.0, 0.707106781187, 0.0, 0.707106781187},
  /* right TCP, wxyz */
  {0.0, 0.707108079850, 0.000005194672, 0.707105482502},
};
```

填入 `stm_input_t`：

```c
memcpy(in.body_q, DEFAULT_BODY_Q, sizeof(in.body_q));
memcpy(in.target_pos, DEFAULT_TARGET_POS, sizeof(in.target_pos));
memcpy(in.target_quat, DEFAULT_TARGET_QUAT, sizeof(in.target_quat));
```

如果真机启动时机械臂不在这个目标附近，控制器会从当前实际 TCP 到该目标重新规划 S 曲线。首次真机验证建议先确认关节角、TCP 坐标系和力矩方向正确，再逐步启用真实力矩输出。

## 最小填充示例

```c
stm_input_t in = {0};
stm_output_t out = {0};

in.active_arm_mask = STM_ARM_MASK_BOTH;

/* 左臂 7 轴 */
in.q[0] = q_l1;
in.q[1] = q_l2;
/* ... */
in.q[6] = q_l7;

/* 右臂 7 轴 */
in.q[7] = q_r1;
in.q[8] = q_r2;
/* ... */
in.q[13] = q_r7;

/* 速度同样按左 7 + 右 7 填入 */
in.qd[0] = qd_l1;
in.qd[7] = qd_r1;

/* 躯干角，单位 rad */
in.body_q[0] = waist01;
in.body_q[1] = waist02;
in.body_q[2] = body0422;

/* 左臂目标，单位 m + wxyz */
in.target_pos[ARM_LEFT][0] = left_x_m;
in.target_pos[ARM_LEFT][1] = left_y_m;
in.target_pos[ARM_LEFT][2] = left_z_m;
in.target_quat[ARM_LEFT][0] = left_qw;
in.target_quat[ARM_LEFT][1] = left_qx;
in.target_quat[ARM_LEFT][2] = left_qy;
in.target_quat[ARM_LEFT][3] = left_qz;

/* 右臂目标，单位 m + wxyz */
in.target_pos[ARM_RIGHT][0] = right_x_m;
in.target_pos[ARM_RIGHT][1] = right_y_m;
in.target_pos[ARM_RIGHT][2] = right_z_m;
in.target_quat[ARM_RIGHT][0] = right_qw;
in.target_quat[ARM_RIGHT][1] = right_qx;
in.target_quat[ARM_RIGHT][2] = right_qy;
in.target_quat[ARM_RIGHT][3] = right_qz;

stm_controller_step_elapsed(&in, &out, elapsed_s);

if (out.status == STM_STATUS_OK) {
  send_left_joint_torque(0, out.tau[0]);
  send_right_joint_torque(0, out.tau[7]);
} else {
  send_all_joint_torque_zero();
}
```

只控制左臂时：

```c
in.active_arm_mask = STM_ARM_MASK_LEFT;
stm_controller_step_elapsed(&in, &out, elapsed_s);
/* 只使用 out.tau[0..6]，out.tau[7..13] 为 0 */
```

只控制右臂时：

```c
in.active_arm_mask = STM_ARM_MASK_RIGHT;
stm_controller_step_elapsed(&in, &out, elapsed_s);
/* 只使用 out.tau[7..13]，out.tau[0..6] 为 0 */
```

## 核心算法逻辑

当前主闭环是“外部最终目标 + C 端五次 S 曲线参考 + 笛卡尔阻抗 + `g+c`”。外部每臂只给最终 TCP 位姿，控制器内部每周期生成一个连续参考点，不会直接对远处最终点做一次大 PD。

### 每周期数据流

`stm_controller_step_elapsed()` 的正常路径按下面顺序执行：

1. 清理 `elapsed_s`：负数、NaN、Inf 按 `0.0`；其他有限非负值完整使用，不做最大步长裁剪。
2. 归一化 `active_arm_mask`：`0` 等价 `STM_ARM_MASK_BOTH`。
3. 如果处于 safety latch，只检查相关激活臂 `qd` 是否静止并累计恢复时间；本轮直接输出 0 力矩。
4. 如果 `body_q[3]` 有效，根据躯干姿态更新左右臂动力学模型里的重力方向。
5. 对每个激活臂执行：
   - 检查该臂 7 个 `q/qd` 是否有限
   - 对 `qd` 做速度滤波
   - 计算带 TCP 偏移的 FK、TCP 姿态和 `6x7` Jacobian
   - 将实际 TCP 写入 `out.ee_pos / out.ee_quat`
   - 合法化最终目标位姿
   - 用 `v_tcp_actual = J(q) * qd_filtered` 估计实际 TCP 速度
   - 做关节位置/速度安全检查
6. 对每个激活臂更新内部路径参考并计算力矩。
7. 对输出力矩做非有限和绝对力矩限位检查；失败则进入 safety latch 并输出 0。
8. 累加 `traj_t += elapsed_s`，`step_count++`。

未激活臂不会做 FK/Jacobian、安全检查或力矩计算，输出保持 0；重新激活时会从当前实际 TCP 到最新目标重新规划。

### 五次 S 曲线路径参考

每臂维护一个内部 `CartesianPathState`，包含起点、目标、当前参考、路径长度、单位方向、路径时间和规划时长。

新目标到来或路径未初始化时，控制器使用当前实际 TCP 作为路径起点：

```c
L = norm(target_pos - start_pos)
direction = (target_pos - start_pos) / L
duration_s = 1.875 * L / END_EFFECTOR_LINEAR_SPEED_MPS
```

其中 `END_EFFECTOR_LINEAR_SPEED_MPS` 是五次 S 曲线的峰值线速度。当前配置为 `0.05 m/s`，不是平均速度。

每周期按传入的真实 `elapsed_s` 推进路径时间：

```c
path_time_s += elapsed_s
u = clamp(path_time_s / duration_s, 0, 1)
s = 10u^3 - 15u^4 + 6u^5
ds_dt = 30u^2(1-u)^2 / duration_s

ref_pos = start_pos + direction * (L * s)
ref_quat = slerp(start_quat, target_quat, s)
ref_twist[0..2] = direction * (L * ds_dt)
ref_twist[3..5] = 0
```

如果位置路径长度小于 `END_EFFECTOR_TARGET_POS_TOL_M`，位置参考直接放到目标位置，姿态仍按目标姿态由阻抗收敛。路径到末端后 `ref_twist` 置 0。

### 到点判定

到点不是只看参考走完，还要求实际机械臂也收敛：

```c
pos_err <= END_EFFECTOR_TARGET_POS_TOL_M
ori_err <= END_EFFECTOR_TARGET_ORI_TOL_RAD
abs(dot(v_tcp_actual[0..2], direction)) <= END_EFFECTOR_ARRIVAL_SPEED_TOL_MPS
```

当前配置：

| 参数 | 当前值 | 含义 |
| --- | --- | --- |
| `END_EFFECTOR_TARGET_POS_TOL_M` | `0.0025` | TCP 位置到点容差 |
| `END_EFFECTOR_TARGET_ORI_TOL_RAD` | `0.005` | TCP 姿态到点容差 |
| `END_EFFECTOR_ARRIVAL_SPEED_TOL_MPS` | `0.02` | 到点切向速度容差 |
| `TRAJ_TARGET_CHANGE_POS_EPS_M` | `0.0001` | 新位置目标重规划阈值 |
| `TRAJ_TARGET_CHANGE_ORI_EPS_RAD` | `0.0005` | 新姿态目标重规划阈值 |

满足到点后，参考点保持最终目标，参考速度为 0。

### 笛卡尔阻抗 + TCP 速度反馈

单臂控制内核是 `control_step_v2_arm_with_reference()`。它不直接接收最终目标，而是接收内部生成的 `ref_pos/ref_quat/ref_twist`。

核心误差和速度：

```c
e_pos = ref_pos - tcp_pos
e_ori = axis_angle_error(ref_quat, tcp_quat)
v_tcp_actual = J(q) * qd_filtered
```

笛卡尔任务力：

```c
F = K * [e_pos, e_ori] + D * (ref_twist - v_tcp_actual)
tau_task = J(q)^T * F
```

平移轴的 `ref_twist[0..2]` 来自 S 曲线切向速度；姿态轴本轮使用 `ref_twist[3..5] = 0`，姿态靠 slerp 参考姿态和阻抗误差收敛。
若 `norm(v_tcp_actual[0..2])` 超过 `END_EFFECTOR_REAL_SPEED_LIMIT_MPS`，控制器会去掉沿当前 TCP 线速度方向继续加速的线性任务力，并按平移阻尼均值追加反向制动力。这是 C 核心内部的真实 TCP 速度保护，不替代驱动器自身速度/力矩安全。

### 动力学补偿、限幅和输出

任务力矩之后叠加动力学补偿：

```c
tau_gc = rbdl_calc_gc(q, qd_filtered)   /* gravity + coriolis */
tau = tau_task + tau_gc
tau = saturate(tau, JOINT_TORQUE_LIMIT_*)
```

当前控制核心保留绝对力矩限幅和真实 TCP 速度制动保护，但没有力矩斜率限制或预判式关节速度限幅。关节速度超过 `JOINT_VEL_LIMIT = 5.0 rad/s` 会触发 safety latch。

## 核心配置参数调节

主要参数集中在 `stm32_code/config.h`。移植初期建议一次只改一组参数，并记录：最大关节速度、最大力矩、是否触发 safety、最终 TCP 误差和实际运动是否抖动。

### 坐标和 TCP 几何

这些参数决定 FK/Jacobian、目标坐标和动力学模型的一致性，真机上不要把它们当调参旋钮随便改。

| 参数 | 当前值 | 含义 | 调节影响 |
| --- | --- | --- | --- |
| `TCP_LEFT_OFFSET_X/Y/Z` | `0.0, 0.07, -0.03` | 左 TCP 相对末端输出 Link 的偏移，单位 `m` | 改错会导致 TCP 位置、Jacobian 和力矩方向一起偏 |
| `TCP_RIGHT_OFFSET_X/Y/Z` | `0.0, -0.07, 0.03` | 右 TCP 相对末端输出 Link 的偏移，单位 `m` | 左右符号不同，用于匹配夹爪几何 |
| `TARGET_FRAME_ORIGIN_BASE_X/Y/Z` | `0.0, 0.0715607946769668, 0.213` | Body0422 动态目标坐标系零位原点 | 必须和上位机/仿真目标坐标转换一致 |

如果真机 TCP 安装位置、夹爪长度或 Body0422 零位定义变了，应先同步这些几何参数，再调 PD。

### 笛卡尔 PD 增益

这些参数直接作用在任务空间力：

```c
F = K * error + D * (ref_twist - J(q) * qd)
```

| 参数 | 当前值 | 含义 | 调大 | 调小 |
| --- | --- | --- | --- | --- |
| `KP_CART_X/Y/Z` | `170` | TCP 平移刚度，近似 `N/m` | 跟踪更硬、误差更小，但更容易超速/振荡/力矩饱和 | 更柔顺、更安全，但误差变大、收敛慢 |
| `KD_CART_X/Y/Z` | `65` | TCP 平移阻尼，近似 `N/(m/s)` | 抑制超调和速度误差，但过大会放大速度噪声、动作发涩 | 阻尼不足，容易冲过目标 |
| `KP_CART_ROLL/PITCH/YAW` | `8` | 姿态轴角误差刚度 | 姿态收敛更快，但腕部力矩可能更大 | 姿态误差收敛慢 |
| `KD_CART_ROLL/PITCH/YAW` | `4.0` | 姿态角速度阻尼 | 姿态更稳，但速度噪声更敏感 | 姿态容易晃 |

推荐真机调参顺序：

1. 先保持较低 `KP/KD`，确认力矩方向、TCP 坐标和重力补偿正确。
2. 先调平移 `KP_CART_X/Y/Z` 到能稳定跟踪，再逐步加 `KD_CART_X/Y/Z` 抑制超调。
3. 姿态 `KP/KD` 最后调，尤其注意 J5/J6/J7 速度和力矩峰值。
4. 如果出现关节速度超限，不要只盲目降 `KP`；同时看 `END_EFFECTOR_LINEAR_SPEED_MPS`、目标距离、`qd` 噪声和速度滤波。

### 路径速度和到点阈值

这些参数控制内部五次 S 曲线和到点判定。

| 参数 | 当前值 | 含义 | 调节影响 |
| --- | --- | --- | --- |
| `END_EFFECTOR_LINEAR_SPEED_MPS` | `0.05` | 五次 S 曲线峰值 TCP 线速度，单位 `m/s` | 调大路径更快，但加速度、力矩和关节速度峰值都会上升 |
| `END_EFFECTOR_REAL_SPEED_LIMIT_MPS` | `0.05` | 基于 `J(q) * qd_filtered` 的真实 TCP 线速度限幅，单位 `m/s` | 调大允许实际末端更快；调小会更早进入制动 |
| `END_EFFECTOR_TARGET_POS_TOL_M` | `0.0025` | 位置到点容差，单位 `m` | 调小更精确但更难判定到点；调大更容易结束 |
| `END_EFFECTOR_TARGET_ORI_TOL_RAD` | `0.005` | 姿态到点容差，单位 `rad` | 调小姿态要求更严，可能停留更久 |
| `END_EFFECTOR_ARRIVAL_SPEED_TOL_MPS` | `0.02` | 到点时 TCP 切向速度阈值，单位 `m/s` | 调小要求更稳才到点；调大可能还在滑动时判定到点 |
| `TRAJ_TARGET_CHANGE_POS_EPS_M` | `0.0001` | 目标位置变化重规划阈值，单位 `m` | 调小更敏感但可能被上位机噪声频繁重规划 |
| `TRAJ_TARGET_CHANGE_ORI_EPS_RAD` | `0.0005` | 目标姿态变化重规划阈值，单位 `rad` | 调小更敏感，调大可过滤微小姿态抖动 |

`END_EFFECTOR_LINEAR_SPEED_MPS` 是峰值速度，不是平均速度。五次曲线的规划时长近似：

```c
duration_s = 1.875 * path_length / END_EFFECTOR_LINEAR_SPEED_MPS
```

### 关节位置、速度和力矩安全

这些参数是安全边界，不建议为了“跑得起来”随意放大。

| 参数 | 当前值 | 含义 | 调节影响 |
| --- | --- | --- | --- |
| `CONTROL_JOINT_LIMIT_INSET_RATIO` | `0.01` | 位置限位向内收缩比例 | 调大更保守；调小更接近机械极限 |
| `JOINT_POS_MIN/MAX_*` | 左臂 degree 转 rad | 左臂关节位置安全限位 | 应匹配真实机械限位 |
| `RIGHT_JOINT_POS_MIN/MAX_*` | 右臂 rad | 右臂关节位置安全限位 | 应匹配真实机械限位 |
| `JOINT_VEL_LIMIT` | `5.0` | 左右臂统一关节速度上限，单位 `rad/s` | 超过会 safety latch；真机不建议随意放大 |
| `JOINT_TORQUE_LIMIT_1..7` | `40,40,27,27,7,7,9` | 单臂各轴绝对力矩限幅，单位 `Nm` | 调小更安全但跟踪弱；调大前必须确认电机/减速器能力 |

注意：`control_logic.c` 会先对 `tau` 做绝对限幅；`stm_controller.c` 还会检查输出是否非有限或超过检查限位。当前没有力矩斜率限制，所以下发端要确认驱动器能承受瞬时力矩变化。

### 速度滤波和 safety 恢复

| 参数 | 当前值 | 含义 | 调大 | 调小 |
| --- | --- | --- | --- | --- |
| `KALMAN_Q_VEL` | `0.001` | 速度滤波过程噪声 | 更相信速度变化，响应更快但更吵 | 更平滑但滞后更大 |
| `KALMAN_R_VEL` | `0.1` | 速度测量噪声 | 更不相信测量，更平滑但更慢 | 更相信测量，响应快但噪声大 |
| `SAFETY_RECOVERY_QD_ZERO_TOL` | `0.02` | safety 恢复静止速度阈值，`rad/s` | 更容易恢复，但可能在轻微运动中恢复 | 更严格，恢复更慢 |
| `SAFETY_RECOVERY_HOLD_S` | `0.1` | 静止保持时间，`s` | 恢复更保守 | 恢复更快但更激进 |

如果出现“仿真/真机明明没动但 qd 抖动导致无法恢复”，优先检查速度单位、编码器差分周期和滤波参数，再考虑放宽 `SAFETY_RECOVERY_QD_ZERO_TOL`。

### 兼容但当前主闭环不用的参数

`IK_MAX_ITERATIONS`、`IK_TOL_POS`、`IK_T_ORI`、`IK_MAX_STEP`、`IK_DAMPING` 是旧数值 IK 兼容参数。当前 `stm_controller_step_elapsed()` 主闭环不调用旧 IK，末端运动由最终 TCP 目标、五次 S 曲线参考和 `J(q)*qd` 速度反馈完成。调这些 IK 参数不会改变当前主控制链路的末端跟踪表现。

## 安全锁定与恢复

触发条件：

- 激活臂任意关节位置超过限位
- 激活臂任意关节速度超过限位
- 激活臂任意输出力矩非有限或超过检查限位
- 激活臂输入 `q/qd` 出现非有限值

触发后控制器会立即：

- 设置 `out.status = STM_STATUS_SAFETY_LATCHED`
- 将 `out.tau[0..13]` 全部置 0
- 记录触发锁定时的 `active_arm_mask`

恢复条件只看触发锁定时激活的手臂速度：

- 左臂模式触发：只要求 `qd[0..6]` 静止
- 右臂模式触发：只要求 `qd[7..13]` 静止
- 双臂模式触发：要求 `qd[0..13]` 静止

静止阈值：

```c
SAFETY_RECOVERY_QD_ZERO_TOL = 0.02 /* rad/s */
SAFETY_RECOVERY_HOLD_S = 0.1       /* s */
```

也就是说，相关轴速度都满足 `fabs(qd[i]) <= SAFETY_RECOVERY_QD_ZERO_TOL`，并连续保持 `0.1s` 后，控制器才解除安全锁定。

安全恢复不要求关节角 `q` 回到 0 位，`body_q[3]` 也不参与恢复判定。恢复完成后，控制器会清空上次目标缓存；下一次正常控制会使用当前输入目标。

## 实时性测量与优化边界

当前热路径主要包含：

- FK 和 Jacobian
- RNEA 重力/科氏补偿
- 笛卡尔 PD 和 TCP 速度反馈
- 内部连续小步参考点推进
- 位置、速度、力矩安全检查

已经做过的低风险实时性优化：

- 安全锁定期间快速返回，只检查相关激活臂速度是否静止并累计保持时间
- 正常控制时每臂 FK/Jacobian 只计算一次，同时用于 `ee_pos/ee_quat` 和控制内核
- 控制增益、左右限位数组使用静态常量，减少每 tick 栈初始化
- 单臂模式下未激活臂完全跳过计算

当前刻意保留：

- `double`
- RNEA 动力学
- 笛卡尔 PD
- 基于 `qd` 反馈的 TCP 速度闭环

建议在 STM32H7 真机工程里用 DWT cycle counter 或 TIM 对 `stm_controller_step_elapsed()` 做包围测量：

```c
uint32_t cycle0 = DWT->CYCCNT;
stm_controller_step_elapsed(&in, &out, elapsed_s);
uint32_t cycle1 = DWT->CYCCNT;

uint32_t control_cycles = cycle1 - cycle0;
double control_us = (double)control_cycles * 1000000.0 / (double)SystemCoreClock;
```

实测耗时应小于你的目标闭环周期预算。如果按 1 kHz 闭环运行，单次控制计算必须稳定低于 1 ms，并且还要给采样、通信、驱动下发和其他任务留出余量。

## 日志

`stm32_code` 不使用 `printf`、`fprintf`、`stderr`、`clock_gettime` 或 `<time.h>`。安全日志通过 `STM_LOG_ERROR(...)` 输出，默认关闭：

```c
#ifndef STM_LOG_ERROR
#define STM_LOG_ERROR(...) ((void)0)
#endif
```

真机工程如需串口日志，可以在工程配置中重定义：

```c
#define STM_LOG_ERROR(...) my_uart_printf(__VA_ARGS__)
```

## 移植检查清单

- `stm_controller_step_elapsed()` 是唯一高层控制入口。
- `active_arm_mask = 0` 等价双臂控制，建议真机工程显式填写。
- 只控制单臂时，未激活臂输出 0，且不参与安全检查或恢复判定。
- `q/qd/tau` 都是左臂 7 轴在前，右臂 7 轴在后。
- 所有角度输入都是 `rad`，不要传入 degree。
- 所有位置输入都是 `m`，不要传入 mm。
- `target_quat` 和 `ee_quat` 都使用 `wxyz`。
- `elapsed_s` 由平台层传入，不要在 `stm32_code` 内部读取定时器。
- `status < 0` 时不要继续正常下发力矩；控制器已输出全 0，平台层应进入安全处理。
- UART/CAN/USB/RTOS 通信协议属于平台层，不属于 `stm32_code` 控制核心。
