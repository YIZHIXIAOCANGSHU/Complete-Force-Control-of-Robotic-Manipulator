# AM-D02 真机控制与全参辨识说明

这个仓库用于驱动 AM-D02 七轴机械臂控制链路，只保留四个入口模式：

- `sim`：真机控制仿真，启动 MuJoCo 仿真服务器和 Pinocchio 控制器，用于不接硬件时联调控制链路。
- `real`：真实硬件控制，保留串口 serial 和 SocketCAN USB2FDCAN 后端。
- `param-id-sim`：全参辨识 PD 闭环仿真，基于 MuJoCo 采集辨识数据并求解质量、质心、惯量参数。
- `param-id-real`：全参辨识实机采集，通过真实硬件反馈采集数据并执行参数辨识。

最重要的入口是：

```bash
./run.sh
```

不带参数时会进入数字菜单：

```text
1) sim           - 真机控制仿真
2) real          - 真实硬件控制
3) param-id-sim  - 全参辨识 PD 闭环仿真
4) param-id-real - 全参辨识实机采集
```

也可以直接指定模式：

```bash
./run.sh sim
./run.sh real
./run.sh param-id-sim
./run.sh param-id-real
```

## 这个仓库里有什么

- `run.sh`
  四模式统一启动脚本。
- `python/modes/`
  四个用户入口模式：`control_sim`、`control_real`、`param_id_sim`、`param_id_real`。
- `python/control/`
  真机控制公共流水线和 control-sim 后端。
- `python/param_id/`
  全参辨识算法层，包含激励轨迹、回归器、求解器和报告生成。
- `python/sim/`
  MuJoCo 场景、环境和 Pinocchio 仿真控制器。
- `python/common/`
  MuJoCo、Rerun、运行时状态、坐标转换等公共工具。
- `python/core/`
  Pinocchio / 动力学后端。
- `usb2fdcan_send/`
  达妙 USB2FDCAN 协议、反馈解码和发送工具。
- `control/`
  控制相关公共资源。
- `c_interface/`
  宿主机 C 接口程序。
- `stm32_code/`
  可移植 C 控制算法。
- `AM-D02-AemLURDF_real/`
  机械臂 URDF 和 mesh 资源。
- `results/`
  参数辨识、报告和运行输出目录。

## 快速开始

建议在 Linux 环境下运行，至少准备：

- `python3`
- `pip`
- `gcc`
- `make`

安装 Python 依赖：

```bash
python3 -m pip install -r python/requirements.txt
```

常用依赖包括：

- `mujoco`：仿真和可视化核心依赖。
- `numpy` / `scipy`：数值计算和参数辨识。
- `rerun-sdk`：实时可视化。
- `pyserial`：真机串口模式。

## 四个模式

### `sim` 真机控制仿真

```bash
./run.sh sim
```

运行链路：

1. `run.sh` 启动 `python -m modes.control_sim.main`。
2. Python 侧创建 MuJoCo 仿真环境并写入 ready file。
3. 前台启动 `python/sim/pinocchio_sim_controller.py`。
4. 控制器从仿真获取状态，调用 Pinocchio/控制流水线计算力矩。
5. 力矩回写 MuJoCo，形成闭环仿真。

适合：

- 不接真机时验证控制回路。
- 调试坐标系、关节方向、力矩限制和可视化。
- 在进入真实硬件前做安全检查。

### `real` 真实硬件控制

```bash
./run.sh real
```

可以继续通过参数选择后端：

```bash
./run.sh real serial
./run.sh real usbfdcan
```

串口后端保留旧链路，适合仍使用串口下位机的设备。USB2FDCAN 后端默认使用 SocketCAN 网卡 `can0`，适合达妙电机 MIT torque 控制链路。

常用环境变量：

```bash
AM_D02_CAN_INTERFACE=can1 ./run.sh real usbfdcan
AM_D02_SERIAL_FORWARD_TARGET=1 ./run.sh real serial
```

真实硬件运行前请确认急停、机械限位、CAN/串口权限和电机使能状态。

### `param-id-sim` 全参辨识 PD 闭环仿真

```bash
./run.sh param-id-sim
```

入口：

```bash
python -m modes.param_id_sim.main
```

该模式使用 MuJoCo + PD/前馈控制执行激励轨迹，采集仿真中的 `q`、`qd`、`tau`，再构建回归器求解质量、质心和惯量参数。默认可打开 MuJoCo viewer，Rerun 默认关闭。

常用环境变量：

```bash
AM_D02_ENABLE_VIEWER=0 ./run.sh param-id-sim
AM_D02_ENABLE_RERUN=1 ./run.sh param-id-sim
```

### `param-id-real` 全参辨识实机采集

```bash
./run.sh param-id-real
```

入口：

```bash
python -m modes.param_id_real.main
```

该模式在真实硬件上执行或采集激励轨迹数据，使用反馈状态与力矩数据进行全参辨识。默认启用 Rerun，默认 CAN 接口为 `can0`。

示例：

```bash
AM_D02_CAN_INTERFACE=can1 ./run.sh param-id-real
```

实机辨识会驱动真实机械臂，运行前必须确认工作空间安全、急停可用、设备通信正常。

## 常用环境变量

```bash
AM_D02_ENABLE_VIEWER=1
AM_D02_ENABLE_RERUN=1
AM_D02_RERUN_LOG_STRIDE=25
AM_D02_REAL_VIEWER_FPS=30
AM_D02_RERUN_QUEUE_SIZE=512
AM_D02_SIM_REALTIME=1
AM_D02_SERIAL_FORWARD_TARGET=0
AM_D02_CAN_INTERFACE=can0
AM_D02_CAN_FEEDBACK_TIMEOUT_S=0.10
AM_D02_CAN_STARTUP_ENABLE=1
```

含义简述：

- `AM_D02_ENABLE_VIEWER`：是否打开 MuJoCo 可视化窗口。
- `AM_D02_ENABLE_RERUN`：是否启用 Rerun 实时可视化。
- `AM_D02_RERUN_LOG_STRIDE`：Rerun 每隔多少步记录一次。
- `AM_D02_REAL_VIEWER_FPS`：真机模式下 MuJoCo viewer 刷新率。
- `AM_D02_RERUN_QUEUE_SIZE`：Rerun 异步队列大小。
- `AM_D02_SIM_REALTIME`：`sim` 模式是否按 MuJoCo 步长实时节拍运行。
- `AM_D02_SERIAL_FORWARD_TARGET`：真机串口模式是否把目标位姿继续下发到串口设备。
- `AM_D02_CAN_INTERFACE`：USB2FDCAN 使用的 SocketCAN 网卡。
- `AM_D02_CAN_FEEDBACK_TIMEOUT_S`：USB2FDCAN 反馈超时时间。
- `AM_D02_CAN_STARTUP_ENABLE`：USB2FDCAN 链路启动时是否 clear error、enable 并发送 MIT 零力矩。

## 串口注意事项

串口模式默认设备通常是：

```text
/dev/ttyUSB0
```

如果真机跑不起来，优先检查：

- 设备是否真的挂在 `/dev/ttyUSB0`。
- 当前用户是否有串口访问权限。
- 下位机波特率是否匹配。
- 是否有其他程序占用了串口。

## SocketCAN USB2FDCAN 注意事项

默认 CAN 网卡：

```text
can0
```

常见配置命令：

```bash
sudo ip link set can0 down
sudo ip link set can0 type can bitrate 1000000 dbitrate 5000000 fd on
sudo ip link set can0 up
```

如果设备不是 `can0`：

```bash
AM_D02_CAN_INTERFACE=can1 ./run.sh real usbfdcan
AM_D02_CAN_INTERFACE=can1 ./run.sh param-id-real
```

## 推荐阅读入口

- `run.sh`：四模式统一启动逻辑。
- `python/modes/control_sim/main.py`：真机控制仿真入口。
- `python/modes/control_real/main.py`：真实硬件控制入口。
- `python/modes/param_id_sim/main.py`：全参辨识 PD 闭环仿真入口。
- `python/modes/param_id_real/main.py`：全参辨识实机入口。
- `python/control/`：控制公共流水线和仿真后端。
- `python/param_id/identification.py`：参数辨识求解。
- `python/param_id/regressor.py`：回归矩阵构建。
- `python/common/mujoco/`：MuJoCo 公共仿真模块。
- `python/core/pinocchio_backend.py`：Pinocchio 动力学后端。
- `usb2fdcan_send/`：USB2FDCAN 协议实现。
- `stm32_code/`：可移植控制算法本体。

## 第一次使用建议

1. 安装依赖：`python3 -m pip install -r python/requirements.txt`。
2. 先运行 `./run.sh sim`，确认仿真链路可用。
3. 再运行 `./run.sh param-id-sim`，确认辨识仿真可用。
4. 确认硬件、安全和通信后，再运行 `./run.sh real` 或 `./run.sh param-id-real`。

这样可以把问题分清楚：依赖、仿真、辨识算法、通信链路、真实硬件分别定位。
