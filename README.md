# AM-DPBSURDF0422 Sim/Real/MC 使用说明

这个仓库现在保留 AM-DPBSURDF0422 左右双臂 14 轴的仿真和 SocketCAN 真机链路：

- `sim`：Python 侧启动 MuJoCo 仿真服务器，C 侧运行实时控制回路。
- `real`：Python 侧通过 SocketCAN USB2FDCAN 连接真实左右臂，C 侧运行同一套控制核心。
- `mc`：复用同一个 MuJoCo 模型，对左右双臂关节空间做蒙特卡罗采样并输出两侧末端位姿范围。
- `sim-report`：生成 sim-only 报告数据包，包含蒙特卡罗工作空间和确定性闭环多目标阶跃实验。

旧串口真机链路和旧 AM-D02-AemLURDF0413 模型入口不再恢复；真机只保留当前双臂 SocketCAN MIT torque 链路。

## 快速开始

安装依赖：

```bash
./scripts/setup_venv.sh
```

启动交互式菜单：

```bash
./run.sh
```

直接启动联合仿真：

```bash
./run.sh sim
```

运行蒙特卡罗范围检查：

```bash
./run.sh mc
```

启动真机模式后选择 left/right/both：

```bash
./run.sh real left
./run.sh real right
./run.sh real both
```

无窗口运行蒙特卡罗：

```bash
./run.sh mc -n 10000 --seed 42 --no-viewer
```

生成报告数据包：

```bash
./run.sh sim-report
./run.sh sim-report --output-dir results/sim_report_demo -n 500000 --seed 42 --max-hull-points 20000 --control-duration-s 10
```

`sim-report` 默认关闭 viewer/Rerun，使用随机种子 `42`、`500000` 个蒙特卡罗样本和 `10s`
闭环多目标阶跃实验。输出目录默认是 `results/sim_report_<timestamp>/`；输出文件包括：

- `workspace_points.csv`：左右臂末端位置和四元数采样点，不保存采样关节角。
- `workspace_summary.json`：关节采样边界、末端位姿范围、安全内部长方体和模型/控制配置元数据。
- `control_loop.csv`：闭环实验逐步数据，包含目标/实际 TCP 位置、误差、速度、状态、力矩、`q` 和 `qd`。
- `control_summary.json`：闭环误差、力矩、状态计数和目标段信息。
- `report.md` 与若干 `.svg`：中文报告摘要和可直接引用的工作空间/闭环曲线图。

注意：`sim-report` 输出只代表 MuJoCo 仿真和 C 控制核心证据，不能替代真机验证。

## 仓库结构

- `AM-DPBSURDF0422/`：当前 MuJoCo/URDF 模型、mesh 和 ROS 描述文件。
- `python/sim/`：MuJoCo 仿真、UDP 服务和 Monte Carlo 工具。
- `python/real/`：SocketCAN USB2FDCAN 真机控制入口、CAN transport 和 C 控制桥。
- `python/` 根目录：保留少量兼容 wrapper 和共享配置/可视化工具。
- `c_interface/`：宿主机 C 控制入口；`c_main` 服务 sim，`real/real_controller` 服务 real。
- `stm32_code/`：左右两个独立 7 轴控制器上下文、运动学、动力学和末端速度闭环。
- `tests/`：启动脚本、Monte Carlo、viewer 和 Rerun 相关测试。

## 当前模型约定

仿真加载 `AM-DPBSURDF0422/urdf/AM-DPBSURDF0422.urdf`，同时控制左臂 7 轴和右臂 7 轴：

```text
# Left
ArmL02_Joint
AM-D02-J14_Joint
ArmL04_Joint
ArmL05_Joint
ArmL06_Joint
ArmL07_Joint
ArmL07Output_Joint

# Right
ArmR01_Joint_duplicate_2
AM-D02R-J03_Joint
ArmR04_Joint
ArmR05_Link
ArmR06_Link
ArmR07_Link
ArmR07Output_Link
```

默认情况下，仿真保留 `Waist01_Joint`、`Waist02_Joint`、`Body0422_Joint` 三个躯干自由度，
并在启动 `sim` 时打开一个小 GUI 滑条窗口。滑条会把这 3 个关节锁定到外部命令角度；
C 控制器仍只输出左右双臂 14 轴力矩。Python 只把三躯干角度发送给 C，重力方向解算在 C 端完成。
如需关闭滑条并恢复 14 DOF 固定躯干模型，可设置 `AM_D02_ENABLE_BODY_GUI=0`。

仿真 UDP 协议使用 45 个 `double` 输入和 14 个 `double` 输出：

```text
q[14] + qd[14]
+ body_q[3]
+ left_target_pos[3] + left_target_quat[4]
+ right_target_pos[3] + right_target_quat[4]
```

其中 `left_target_pos` / `right_target_pos` 使用 Body0422 动态目标坐标系：
坐标原点跟随当前 `Body0422_Link`，坐标轴方向使用 `Body0422_Link` 相对零位的旋转。
MuJoCo 里的目标方块会按该相对位姿随 Body0422 一起平移和旋转；四元数也使用同一个动态目标坐标系。
C 端内部从当前 TCP 到最终目标生成五次 S 曲线参考点；host wrapper 传入的 elapsed time
用于推进该曲线时间，同时累计 `traj_t` 和安全恢复计时。sim UDP 保持一收一发：每收到一帧
MuJoCo 状态，C 端计算一次力矩并立即发送一次 `tau[14]`。TCP 平移速度由电机回传 `qd`
经 Jacobian 映射得到的实际末端速度闭环控制，默认 S 曲线峰值线速度为 `0.05 m/s`。
C 侧不再保留默认左臂 7 轴闭环入口；公开控制链路统一使用
`stm_controller_step_elapsed()`，并可通过 `stm_input_t.active_arm_mask`
选择左臂、右臂或双臂。本仓库的 UDP 仿真默认填 `STM_ARM_MASK_BOTH`。

## C 语言控制闭环核心

当前控制核心集中在 `stm32_code/`，宿主机上的 `c_interface/c_main` 和
`c_interface/real/real_controller` 都只是包装层。真正的闭环入口是：

```c
stm_controller_step_elapsed(const stm_input_t *in,
                            stm_output_t *out,
                            double elapsed_s)
```

输入由外部平台提供：

- `active_arm_mask`：选择 left、right 或 both。未激活臂不会参与闭环输出。
- `q[14] / qd[14]`：左右臂关节角和关节速度，左臂在 `0..6`，右臂在 `7..13`。
- `body_q[3]`：Body0422 三个躯干关节角，用于更新重力方向。
- `target_pos[2][3] / target_quat[2][4]`：左右最终 TCP 目标，使用 Body0422 动态目标坐标系。
- `elapsed_s`：外部平台本地时钟的真实周期，用来推进 C 端内部轨迹时间、`traj_t` 和安全恢复计时。

每个控制周期内部流程如下：

```text
输入 q/qd/body_q/target
  -> 更新躯干重力方向
  -> 左右臂独立上下文做 FK、TCP offset、TCP frame 和 Jacobian
  -> 根据最终 TCP 目标生成五次 S 曲线参考 pos/quat/twist
  -> 用 J(q) * qd 得到实际 TCP twist，做笛卡尔阻抗/速度闭环
  -> tau_task = J^T * F
  -> 加 RBDL g+c 补偿
  -> 关节位置/速度/力矩安全检查与限幅
  -> 输出 tau[14]、ee_pos、ee_quat、ee_twist、status、traj_t、step_count
```

### 笛卡尔 PD + g+c

当前 C 主闭环不是关节空间 PD，也不再调用旧数值 IK。每个激活臂都在 TCP 笛卡尔空间做阻抗控制，再通过 Jacobian 转成关节力矩：

```text
e_pos = ref_pos - tcp_pos(q)
e_rot = axis_angle(ref_quat * inverse(tcp_quat(q)))
v_tcp = J(q) * qd_filtered

F[i] = K_cart[i] * e[i] + D_cart[i] * (ref_twist[i] - v_tcp[i])
tau_task = J(q)^T * F
tau_gc = RBDL_inverse_dynamics(q, qd_filtered, qdd=0)
tau = saturate(tau_task + tau_gc, JOINT_TORQUE_LIMIT)
```

其中 `e[0..2]` 是 TCP 位置误差，`e[3..5]` 是姿态轴角误差；`F[0..2]` 近似是 N，`F[3..5]` 近似是 N·m。
`tau_gc` 是 gravity + Coriolis/centrifugal，也就是常说的 `g+c` 补偿。它让控制器在重力和当前速度相关动力学上有前馈支撑，PD 只负责把 TCP 拉向参考轨迹并做阻尼。

相关参数在 `stm32_code/config.h`：

- `KP_CART_X/Y/Z`、`KD_CART_X/Y/Z`：平移方向刚度和阻尼；当前调参倾向是强终点跟踪。
- `KP_CART_ROLL/PITCH/YAW`、`KD_CART_ROLL/PITCH/YAW`：姿态轴角误差的刚度和阻尼。
- `KALMAN_Q_VEL`、`KALMAN_R_VEL`：先滤 `qd`，再用于 `v_tcp = J(q) * qd_filtered`、阻尼项和速度安全检查。
- `JOINT_TORQUE_LIMIT_1..7`：最终输出力矩饱和；调大前要确认驱动器、减速器和机械结构余量。

注意：C 输出力矩目前是 `tau_task + tau_gc` 后饱和，不在 C 内额外叠加 follower friction。仿真里的 follower friction 是 Python MuJoCo plant 侧的被动摩擦模型，用来让被控对象更接近真机负载；它不是 C 控制器输出项。

### C 端路径规划与速度规划

外部只给最终 TCP 目标，C 端在 `stm_controller.c` 内为每个激活臂维护一条 `CartesianPathState`。控制器不会把最终目标直接丢给 PD，而是先把“当前 TCP 位姿 -> 最终目标位姿”变成一个连续参考：

```text
current tcp pose
  -> plan CartesianPathState
  -> 每周期用 elapsed_s 推进 path_time_s
  -> 输出 ref_pos / ref_quat / ref_twist
  -> 笛卡尔 PD + g+c
```

#### 什么时候重新规划

每个激活臂独立判断目标是否变化。满足任一条件就从当前 TCP 位姿重新规划，而不是从上一条轨迹的旧参考点续接：

- 该臂路径还没有初始化；
- 最终位置变化大于 `TRAJ_TARGET_CHANGE_POS_EPS_M`；
- 最终姿态变化大于 `TRAJ_TARGET_CHANGE_ORI_EPS_RAD`。

重新规划时记录：

```text
start_pos/start_quat = 当前 TCP 位姿
target_pos/target_quat = 外部给定最终目标
delta = target_pos - start_pos
path_length = norm(delta)
direction = delta / path_length
path_time_s = 0
```

#### 位置路径: 直线 + 五次 S 曲线进度

位置只规划一条 TCP 空间直线，曲线本身不绕路；平滑性来自标量进度 `s(u)`：

```text
u = clamp(path_time_s / duration_s, 0, 1)
s(u) = 10u^3 - 15u^4 + 6u^5
ref_pos = start_pos + direction * path_length * s(u)
progress = path_length * s(u)
```

这个五次多项式满足：

- `s(0)=0`，`s(1)=1`
- 起点和终点速度为 0
- 起点和终点加速度也为 0

所以参考点从当前 TCP 平滑起步，到目标附近平滑停下，避免一步跳到远处目标导致 PD 瞬间给出过大的任务力。

#### 位置速度: 从 S 曲线导数生成 ref_twist

位置速度参考来自同一个进度函数的导数：

```text
ds/du = 30u^2(1-u)^2
ds/dt = (ds/du) / duration_s
speed_ref = path_length * ds/dt
ref_twist[0..2] = direction * speed_ref
```

`ds/du` 的最大值是 `1.875`。代码设置：

```text
duration_s = 1.875 * path_length / END_EFFECTOR_LINEAR_SPEED_MPS
```

因此理论峰值线速度就是 `END_EFFECTOR_LINEAR_SPEED_MPS`。这意味着：

- 改 `END_EFFECTOR_LINEAR_SPEED_MPS` 是在改“参考轨迹走多快”；
- 远目标会得到更长 `duration_s`，近目标更短；
- 如果 `path_length <= END_EFFECTOR_TARGET_POS_TOL_M`，位置参考直接设为目标，线速度参考为 0。

#### 姿态路径: 同步进度的 quaternion slerp

姿态不走欧拉角插值，而是用位置路径同一个 `s(u)` 做四元数球面插值：

```text
ratio = s(u)
ref_quat = slerp(start_quat, target_quat, ratio)
```

这样位置和姿态使用同一个进度比例：路径刚开始时姿态也刚开始转，路径到达终点时姿态参考也到最终姿态。若位置距离很短、直接视为到达，则 `ratio=1`，姿态参考直接为最终姿态。

当前实现没有显式计算姿态角速度参考，也就是说：

```text
ref_twist[3..5] = 0
```

姿态控制仍然跟踪 `ref_quat`，但阻尼项使用的是 `0 - v_tcp_angular`。因此姿态逻辑可以理解为“slerp 生成平滑参考姿态 + 实际角速度阻尼”，而不是“规划一条带角速度前馈的姿态轨迹”。如果后续需要更快且更可控的姿态跟踪，可以在同一套 `s(u)` 上推导角速度前馈并填入 `ref_twist[3..5]`。

#### 到达和保持

每个周期都会用实际 TCP 位姿和实际 TCP 速度判断是否到达：

- 位置误差 `<= END_EFFECTOR_TARGET_POS_TOL_M`
- 姿态误差 `<= END_EFFECTOR_TARGET_ORI_TOL_RAD`
- 沿路径方向的实际 TCP 速度 `<= END_EFFECTOR_ARRIVAL_SPEED_TOL_MPS`

到达后该臂参考固定在最终目标，`ref_twist` 清零，PD 变成围绕目标位姿的保持控制。

### TCP 速度闭环方式

速度闭环发生在 TCP 笛卡尔空间，不是直接对电机速度下命令，也不是额外套一层独立的关节速度 PID。每个周期的实际速度链路是：

```text
电机反馈 qd_raw
  -> control_filter_velocities_arm() 滤波
  -> qd_filtered
  -> FK/Jacobian 得到 J(q)
  -> v_tcp_actual = J(q) * qd_filtered
  -> 写入 out->ee_twist，同时进入笛卡尔阻尼项
```

参考速度链路来自上一节的路径规划：

```text
S 曲线进度 s(u)
  -> ds/dt
  -> ref_twist[0..2] = path_direction * path_length * ds/dt
  -> ref_twist[3..5] = 0
```

真正闭环的地方在 `control_step_v2_arm_with_reference()`。它先用当前 `q` 计算 TCP 位置/姿态误差，再用 Jacobian 和滤波后的 `qd` 计算实际 TCP twist，然后按 6 个笛卡尔轴生成任务力：

```text
F[j] = CARTESIAN_K[j] * pose_error[j]
     + CARTESIAN_D[j] * (ref_twist[j] - v_tcp_actual[j])
```

所以速度闭环可以理解为“PD 里的 D 项跟踪 TCP 参考速度”：

- 位置方向：`ref_twist[0..2]` 是五次 S 曲线给出的前馈线速度，实际 TCP 线速度偏慢时阻尼项会沿路径方向补力，偏快时会反向压住。
- 姿态方向：当前没有显式角速度前馈，`ref_twist[3..5] = 0`，姿态由 `ref_quat` 的轴角误差负责跟踪，角速度项主要做阻尼，让末端旋转不要发散或过冲。
- 到达保持：到达后 `ref_twist` 全部清零，速度闭环变成“把实际 TCP 速度压向 0”的保持阻尼。

最后 `F` 通过 `tau_task = J(q)^T * F` 映射成关节力矩，再叠加 `g+c` 补偿。也就是说速度闭环影响的是笛卡尔任务力和最终力矩，而不是把某个电机速度硬限制到目标值。

调参时重点看两组参数：

- `KD_CART_X/Y/Z` 和 `KD_CART_ROLL/PITCH/YAW`：越大，参考速度跟踪和刹车越强；过大时会放大速度噪声，表现为抖动、发黏或力矩更容易饱和。
- `KALMAN_Q_VEL` / `KALMAN_R_VEL`：决定 `qd_filtered` 的平滑和滞后。滤得太轻，速度闭环更敏感但更容易吃噪声；滤得太重，界面和控制看起来更稳，但实际速度反馈会变慢。

`END_EFFECTOR_REAL_SPEED_LIMIT_MPS` 是速度闭环之外的安全保护。它不改变 `ref_twist`，而是在实际 TCP 线速度已经超过阈值时修改线性任务力：去掉继续加速的分量，并追加反向制动。

### 速度限制与安全限位

控制里有三层不同含义的“速度限制”：

1. `END_EFFECTOR_LINEAR_SPEED_MPS`

   这是路径规划的参考峰值线速度，决定 S 曲线生成出来的 `ref_twist_linear` 有多快。调大它会让目标参考走得更快，但也会让 PD 需要更大的力矩去追踪，容易触发力矩饱和或速度安全。

2. `END_EFFECTOR_REAL_SPEED_LIMIT_MPS`

   这是实际 TCP 线速度保护，作用在笛卡尔任务力 `F` 上，而不是直接把速度数值硬夹住。若 `|v_tcp_linear|` 超过该阈值，`control_limit_linear_wrench_by_tcp_speed()` 会：

   - 去掉 `F` 中继续沿当前速度方向加速的分量；
   - 按平均平移阻尼追加反向制动力，大小约为 `mean(KD_CART_X/Y/Z) * (speed - limit)`。

   因此它的效果是“停止继续加速并主动刹车”，但真机瞬时速度仍由机械惯量、当前力矩、驱动响应和反馈周期共同决定。

3. `JOINT_VEL_LIMIT`

   这是关节速度安全阈值，当前左右臂每轴统一为 `5.0 rad/s`。滤波后的 `qd_filtered` 任一激活轴超过阈值，C 核心进入 safety latch 并输出当前激活臂零力矩。恢复需要所有激活关节速度低于 `SAFETY_RECOVERY_QD_ZERO_TOL` 并持续 `SAFETY_RECOVERY_HOLD_S`。

另外还有两类硬安全：

- 位置安全：`control_check_safety_arm()` 会把每个关节 `[min,max]` 按 `CONTROL_JOINT_LIMIT_INSET_RATIO` 向内收缩后再检查。
- 力矩安全：输出先在 `control_logic.c` 内饱和，`stm_controller.c` 还会检查非有限值和超限；失败则进入 safety latch。

安全状态由 C 核心给出。`STM_STATUS_OK` 表示正常输出；进入 safety latch 后输出零力矩，并等待关节速度满足恢复条件。
在 sim 中，C wrapper 会继续向 MuJoCo 发送零力矩并等待恢复；在 real 中，Python 看到 `status < 0` 会立即进入真机安全停机，发送 zero torque 并 disable 当前激活电机。

`c_interface/real/real_controller.c` 是 PC real 模式的二进制桥：Python 通过 stdin 写入 packed 输入包，C 调用同一个
`stm_controller_step_elapsed()`，再通过 stdout 返回 packed 输出包。这个桥不是 STM32H7 必须移植的协议；移植到板端时只需要保留
`stm32_code/` 控制核心和你自己的采样、通信、定时与电机驱动层。

## Sim/Real 调度模型

本仓库现在有两套调度模型，容易混淆，但边界很清楚：C 控制核心只算闭环，Python 负责仿真/真机 I/O、viewer 和 Rerun。

### Sim: Python MuJoCo 后台进程 + 前台 C 控制进程

`./run.sh sim` 的进程结构：

```text
run.sh
  -> 编译 c_interface/c_main
  -> 后台启动 python/sim/main_server.py
  -> 等 ready file
  -> 前台启动 ./c_interface/c_main
```

Python 仿真进程负责 MuJoCo 状态、目标块、viewer、Rerun 和 UDP socket。C 进程是前台实时控制回路：

```text
C c_main
  INIT/recv state[45 doubles]
  -> stm_controller_step_elapsed()
  -> send tau[14 doubles]
  -> recv next state
```

Python UDP hot path 是一收一发：收到 `tau[14]` 后，在 `env_lock` 内裁剪力矩、推进 MuJoCo 一步、写下一帧 state packet，然后立即回给 C。
viewer 刷新放在 `SimViewerSyncWorker` 线程，Rerun 放在 `SimRerunLogger` 线程，Body0422 滑条 GUI 也在独立线程。这样 viewer 卡顿或 Rerun backpressure 不会直接阻塞 UDP 控制热路径。

Sim 调度注意点：

- C 和 Python 是两个进程，UDP 是同步闭环边界；一帧 state 对应一次 C 控制和一次 MuJoCo step。
- Python 内部所有 MuJoCo 读写通过 `env_lock` 串行化，避免 viewer、GUI 和 UDP 热路径同时改 `mjData`。
- Rerun 日志队列会合并积压，只保留最新 payload；吞吐不足时优先保证控制/仿真继续走。

### Real: Python 主进程 + C bridge 子进程 + CAN 多线程流水线

`./run.sh real left|right|both` 的进程结构：

```text
run.sh
  -> 编译 c_interface/real/real_controller
  -> 前台启动 python/real/main.py
       -> RealControllerBridge 子进程: c_interface/real/real_controller
       -> Python 主线程: MuJoCo mirror viewer 和 target 输入
       -> Python CAN 控制线程: 等反馈、调用 C bridge、提交 TX
       -> Python feedback worker: 每个 CAN runtime 一个
       -> Python TX worker: 每个 CAN 接口一个
       -> Python Rerun logger: 可选异步日志线程
```

real CAN 控制线程不会直接串行读写所有 CAN。启动阶段仍同步 clear/enable/zero；进入闭环后使用流水线：

```text
FeedbackWorker(can0/can1)
  read_available()
  -> pop feedback frame
  -> RealFeedbackHub 记录 q/qd/tau_actual、完整关节 bitmask、seq

ControlThread
  等待 active-arm 的下一组完整反馈 snapshot
  -> 更新 RealSharedState
  -> 如 target 未初始化或 feedback-only，提交 zero/last keepalive
  -> 调用 RealControllerBridge.compute()
  -> 未激活臂 tau 清零
  -> 提交最新 tau 到对应 RealTxWorker

RealTxWorker(can0/can1)
  single-slot latest-only pending tau
  -> 同一 CAN 接口内按 J1..J7 顺序发送
  -> 左右 CAN 接口之间并行发送
```

real 调度注意点：

- both 模式必须等左右两个 active arm 都收到新的 7 轴完整反馈，才会调用 C bridge，避免用一侧新反馈和一侧旧反馈混算。
- TX worker 积压时覆盖旧 torque，只发最新 pending 命令；目标是低延迟真机控制，而不是补发历史 torque。
- feedback-only、target 未初始化和等待反馈期间的 keepalive 都提交给 TX worker，不再由控制线程直接发 CAN。
- 反馈超时、CAN 读异常、viewer 关闭、Ctrl+C 或 C bridge `status < 0` 都会置 `shutdown_event`。
- 停机时先停 feedback worker，再让 TX worker 丢弃普通 pending、发送全 0 MIT torque、disable 当前 runtime 的所有 motor，最后关闭 transport。

## 真机 SocketCAN 模式

真机模式按左右臂分离部署：

```text
left  -> can0, motor id 1..7 -> q[0..6] / tau[0..6]
right -> can1, motor id 1..7 -> q[7..13] / tau[7..13]
both  -> can0 + can1，同时控制 14 轴
```

启动命令：

```bash
./run.sh real left
./run.sh real right
./run.sh real both
./run.sh real both --no-send
./run.sh real both --gravity-only
./run.sh real left --send
```

直接运行 `./run.sh` 进入交互菜单时，选择 real 和 left/right/both 后，还会继续选择控制量下发方式：

- `send control`：默认真机闭环模式。C 控制器计算 `tau` 后，TX worker 按 CAN 接口下发 MIT torque。
- `no-send/observe`：仍读取 CAN 反馈、调用 C 控制器并把计算力矩显示到 Rerun，但实际 CAN 只持续下发全 0 MIT torque。Rerun 中 `tau_raw/tau_total` 用于观察计算结果，`tx_label/tx_str` 标明实际下发为 0。
- `gravity-only`：仍计算完整控制量并显示到 Rerun，但实际 CAN 只下发纯 `G(q)` 重力补偿，不包含 Coriolis/centrifugal 或笛卡尔 PD 任务力矩。
- `AM_D02_REAL_FEEDBACK_ONLY=1`：诊断模式。不调用 C 控制器，只镜像反馈并发送 0 keepalive；它和 `--no-send` 不同，不能用于观察控制器计算力矩。

`gravity-only` 不是 C 端单独闭环模式，而是 Python 真机发送层的力矩选择模式：`RealControllerBridge`
每周期仍调用 C 控制核心，返回完整 `tau` 和单独的 `tau_gravity`；Rerun 继续记录完整 `tau`，
TX worker 只把当前激活臂的 `tau_gravity = G(q)` 复制到 CAN 下发缓冲，未激活臂保持 0。该模式适合真机上单独验证重力补偿方向、量级和关节符号，不会加入 `G+C` 中的速度相关项，也不会加入 TCP 笛卡尔 PD 任务力矩。

默认接口可用环境变量覆盖：

```bash
AM_D02_LEFT_CAN_INTERFACE=can0
AM_D02_RIGHT_CAN_INTERFACE=can1
AM_D02_CAN_NOMINAL_BITRATE=1000000
AM_D02_CAN_DATA_BITRATE=5000000
AM_D02_CAN_FORCE_FD=1
AM_D02_CAN_CONFIGURE_INTERFACE=0
AM_D02_CAN_FEEDBACK_TIMEOUT_S=0.10
AM_D02_REAL_CONTROL_TARGET_HZ=1000
AM_D02_REAL_C_BRIDGE_TIMEOUT_S=0.005
AM_D02_REAL_STATS_INTERVAL_S=1.0
AM_D02_REAL_THREAD_JOIN_TIMEOUT_S=2.0
```

真机模式启动后会拉起一个 MuJoCo viewer 做镜像对照和目标输入界面：

- CAN 链路内部使用多线程流水线：feedback worker 持续收真实电机反馈，控制线程只消费最新完整 active-arm 快照并调用 `c_interface/real/real_controller`，每个 CAN 接口各有 TX worker 下发 MIT torque。
- 控制链以 1 kHz 为目标预算：feedback worker 有帧时会连续 drain，TX worker 忙时覆盖旧 pending 命令只保留最新力矩，Rerun 和 MuJoCo viewer 不进入 CAN/C 热路径。
- 主线程负责 MuJoCo viewer，把真实 `q[14] / qd[14]` 镜像到模型中，并读取左右目标块拖拽后的目标位姿。
- MuJoCo 在 real 模式下不跑真实动力学，也不把 `tau` 施加到物理仿真；它只显示真机反馈姿态、当前 TCP 和目标块。
- 第一轮完整反馈后，viewer 会用真实关节角初始化，并把目标块放到当前 TCP，避免启动瞬间追旧目标。

real 模式默认启用 Rerun，会记录完整闭环数据：`q[14]`、`qd[14]`、C 输出
`tau_total[14]`、CAN 回传 `tau_actual[14]`、`torque_gap`、TCP 实际/目标位姿、位置/姿态误差、C bridge 耗时、反馈等待、TX 覆盖计数和
控制链路周期。单臂模式下只把当前激活臂作为有效控制对象；未激活臂不会参与反馈等待、控制计算或安全恢复。
如需关闭，可用 `AM_D02_ENABLE_RERUN=0 ./run.sh real right`。

如果反馈超时、CAN 异常、viewer 关闭、Ctrl+C 或 C 控制器返回 `status < 0`，程序会对当前激活臂发送零力矩并 disable，然后退出。

## 常用环境变量

```bash
AM_D02_PYTHON=/path/to/python
AM_D02_VENV_DIR=.venv
AM_D02_ENABLE_VIEWER=1
AM_D02_ENABLE_RERUN=1
AM_D02_VISUAL_PROFILE=balanced   # balanced | full | fast
AM_D02_RERUN_LOG_STRIDE=20
AM_D02_RERUN_MAX_HZ=15
AM_D02_SIM_TARGET_HZ=1000
AM_D02_SIM_STATS_INTERVAL_S=1.0
AM_D02_SIM_VIEWER_FPS=30
AM_D02_SIM_VIEWER_LOCK_TIMEOUT_S=0.0002
AM_D02_SIM_VIEWER_SYNC_BUDGET_MS=1.0
AM_D02_SIM_VIEWER_BACKOFF_FRAMES=2
AM_D02_RERUN_DETAILED_PERF=0
AM_D02_SIM_RERUN_INCLUDE_TWIST=0
AM_D02_SIM_RERUN_QUEUE_SIZE=512
AM_D02_SIM_UDP_TIMEOUT_S=0.002
AM_D02_FIX_UNCONTROLLED_JOINTS=1
AM_D02_ENABLE_BODY_GUI=1
AM_D02_MC_SAMPLES=50000
AM_D02_MC_SEED=42
AM_D02_MC_PROGRESS_INTERVAL=200
AM_D02_MC_MAX_VIS_POINTS=3000
AM_D02_MC_MAX_HULL_POINTS=12000
```

`AM_D02_VISUAL_PROFILE=balanced` 是默认档位：保留 MuJoCo viewer 完整交互，但把 Rerun 默认降到 15 Hz / stride 20，并只记录关键性能曲线。需要完整诊断时用 `AM_D02_VISUAL_PROFILE=full`，会恢复 60 FPS viewer、30 Hz Rerun、stride 10、详细 performance 曲线和 sim TCP twist 采样；只追速度时可用 `AM_D02_VISUAL_PROFILE=fast`。
`AM_D02_RERUN_MAX_HZ` 是 Rerun 最大发送频率上限；控制频率很高时，日志线程会合并积压旧帧并只发送最新状态，避免 Rerun gRPC 接收端 backpressure。
`sim` UDP 链路以 `AM_D02_SIM_TARGET_HZ` 作为监控目标，不主动 sleep 限频；MuJoCo viewer 保留完整交互能力，但 sync 线程只在 `AM_D02_SIM_VIEWER_LOCK_TIMEOUT_S` 内拿到仿真锁时刷新，且 `viewer.sync()` 超过 `AM_D02_SIM_VIEWER_SYNC_BUDGET_MS` 后会临时退避 `AM_D02_SIM_VIEWER_BACKOFF_FRAMES` 帧，把仿真锁让回 UDP 热路径。

## 开发与验证

编译 C 控制器：

```bash
make -C c_interface clean && make -C c_interface c_main
make -C c_interface real_controller
```

运行测试：

```bash
pytest
```

运行 MC 验收：

```bash
AM_D02_ENABLE_VIEWER=0 AM_D02_ENABLE_RERUN=0 ./run.sh mc -n 100 --no-viewer
```

运行 sim 验收：

```bash
AM_D02_ENABLE_VIEWER=0 AM_D02_ENABLE_RERUN=0 ./run.sh sim
```

`sim` 会持续运行，确认 C 控制器连接成功并进入控制循环后，用 `Ctrl+C` 结束。
如需无 GUI 固定躯干运行：

```bash
AM_D02_ENABLE_BODY_GUI=0 AM_D02_ENABLE_VIEWER=0 AM_D02_ENABLE_RERUN=0 ./run.sh sim
```
