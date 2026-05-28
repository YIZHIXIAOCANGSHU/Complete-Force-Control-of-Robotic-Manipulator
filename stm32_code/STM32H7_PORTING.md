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
- `trajectory_lib.h/.c`
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

需要清空控制器内部路径规划和安全状态时调用：

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
| 作用 | 推进末端路径规划，并累计 `out.traj_t` |

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

`CONTROL_DT` 只是名义控制周期和调参参考，不是控制核心的真实时间源。

控制核心会保护 `elapsed_s`：

- 负数或非有限值：按 `0.0` 处理
- 超过 `CONTROL_MAX_ELAPSED_S`：默认限制到 `0.02s`

这样可以避免调试暂停或通信卡顿后路径参考突然跳太远。

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

本轮控制哪只手臂。未激活臂会完全跳过 FK/Jacobian、路径规划、安全检查和控制计算，输出力矩保持 0。

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

如果真机暂时没有躯干运动，可以填 `{0.0, 0.0, 0.0}`。

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

路径规划累计时间，单位 `s`。它由每轮传入的 `elapsed_s` 累加得到，用于调试和观察路径推进。

### `step_count`

控制器步数计数。每调用一次 `stm_controller_step_elapsed()` 增加 1。

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

## 路径规划和控制内容

每轮正常控制内部包含：

- 根据 `body_q[3]` 更新重力方向
- 对激活臂计算 FK 和 Jacobian
- 对激活臂做目标合法化和路径规划参考更新
- `LinearPathPlanner` 按 `TRAJ_PLAN_SPEED` / `TRAJ_PLAN_ACCEL` 推进
- 对激活臂计算重力补偿、科氏补偿、笛卡尔 PD 和零空间姿态偏置
- 对激活臂执行位置、速度、力矩安全检查

路径推进只依赖 `elapsed_s`。如果通信或主循环变慢，路径参考会按真实经过时间推进；如果 `elapsed_s = 0`，路径参考不推进，但仍会根据当前状态计算控制力矩。

未激活臂不会做 FK/Jacobian、路径规划、安全检查或力矩计算。

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

安全恢复不要求关节角 `q` 回到 0 位，`body_q[3]` 也不参与恢复判定。恢复完成后，控制器会清空旧路径规划状态；下一次正常控制会从当前 TCP 位置重新规划到当前目标。

## 实时性测量与优化边界

当前热路径主要包含：

- FK 和 Jacobian
- `LinearPathPlanner` 末端直线路径参考更新
- 6x6 DLS 矩阵求逆
- RNEA 重力/科氏补偿
- 笛卡尔 PD 和零空间投影
- 位置、速度、力矩安全检查

已经做过的低风险实时性优化：

- 安全锁定期间快速返回，只检查相关激活臂速度是否静止并累计保持时间
- 正常控制时每臂 FK/Jacobian 只计算一次，同时用于 `ee_pos/ee_quat`、路径起点和控制内核
- 控制增益、偏好姿态、左右限位数组使用静态常量，减少每 tick 栈初始化
- 单臂模式下未激活臂完全跳过计算

当前刻意保留：

- `double`
- RNEA 动力学
- 笛卡尔 PD
- 零空间控制
- 末端路径规划

建议在 STM32H7 真机工程里用 DWT cycle counter 或 TIM 对 `stm_controller_step_elapsed()` 做包围测量：

```c
uint32_t cycle0 = DWT->CYCCNT;
stm_controller_step_elapsed(&in, &out, elapsed_s);
uint32_t cycle1 = DWT->CYCCNT;

uint32_t control_cycles = cycle1 - cycle0;
double control_us = (double)control_cycles * 1000000.0 / (double)SystemCoreClock;
```

实测耗时应小于你的控制周期预算。`CONTROL_DT` 默认是 `0.001s`，如果按 1 kHz 闭环运行，单次控制计算必须稳定低于 1 ms，并且还要给采样、通信、驱动下发和其他任务留出余量。

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
