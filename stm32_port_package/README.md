# STM32 Port Package

这个目录已经收成仅面向 STM32 使用的版本。

## 你需要加入工程的文件

把 `core/` 目录下所有 `.c` 和 `.h` 加进 STM32 工程即可。

其中最重要的是：

- `stm_controller.*`：对外主入口
- `control_logic.*`：核心控制逻辑
- `kinematics_lib.*` / `dynamics_lib.*` / `model_lib.*` / `math_lib.*`：底层算法
- `trajectory_lib.*`：末端直线轨迹规划
- `config.h`：控制参数
- `uart_protocol.*`：如果你要保留上位机串口协议，就一起加进去

## 你实际只需要调用哪些函数

日常控制只需要这 3 个函数：

1. `stm_controller_init()`
   系统上电后调用一次。

2. `stm_controller_step(const stm_input_t *in, stm_output_t *out)`
   每个控制周期调用一次。

3. `stm_controller_reset()`
   需要清空内部轨迹状态时调用，比如急停后重新开始。

如果你想统计单次计算耗时，再额外调用这个函数：

4. `stm_controller_set_platform_hooks(const stm_platform_hooks_t *hooks)`
   给控制器传入一个毫秒计时函数。

## 最小使用方式

### 1. 初始化

```c
stm_controller_init();
```

### 2. 每个周期准备输入

```c
stm_input_t in;
stm_output_t out;

for (int i = 0; i < 7; ++i) {
    in.q[i] = joint_pos[i];     // rad
    in.qd[i] = joint_vel[i];    // rad/s
}

in.target_pos[0] = target_x;    // m
in.target_pos[1] = target_y;
in.target_pos[2] = target_z;

in.target_quat[0] = target_w;   // (w, x, y, z)
in.target_quat[1] = target_xq;
in.target_quat[2] = target_yq;
in.target_quat[3] = target_zq;
```

### 3. 每个周期执行一步控制

```c
stm_controller_step(&in, &out);
```

### 4. 取结果

```c
for (int i = 0; i < 7; ++i) {
    motor_tau_cmd[i] = out.tau[i];
}
```

## 输入输出说明

`stm_input_t`

- `q[7]`：当前关节角，单位 `rad`
- `qd[7]`：当前关节角速度，单位 `rad/s`
- `target_pos[3]`：目标末端位置，单位 `m`
- `target_quat[4]`：目标末端姿态四元数，顺序是 `(w, x, y, z)`

`stm_output_t`

- `tau[7]`：输出关节力矩
- `status`：`0` 表示本次控制正常，负数表示安全检查失败
- `ee_pos[3]`：当前末端位置
- `ee_quat[4]`：当前末端姿态
- `traj_t`：内部轨迹时间
- `step_count`：累计步数
- `calc_time_ms`：单步计算耗时，需要你设置平台计时钩子后才有效

## 如果你还要走串口协议

如果 STM32 通过串口接收上位机目标，使用：

- `parse_control_state_packet()`
- `pack_control_force_packet()`

否则可以完全不调用 `uart_protocol.*`，直接自己填 `stm_input_t`。

## 注意事项

- `CONTROL_DT` 在 `core/config.h` 中，必须和你的定时器中断周期一致。
- 默认日志已经关闭，不依赖 `printf`/`stderr`。
- 如果你想开启错误日志，可以在工程里自行重定义 `STM_LOG_ERROR(...)`。
