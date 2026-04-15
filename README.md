# AM-D02 真机启动与联合仿真说明

这个仓库用于驱动一套 AM-D02 七轴机械臂控制链路，包含两种使用方式：

- `sim` 模式：Python 侧启动 MuJoCo 仿真，C 侧运行实时控制回路，适合联调控制算法。
- `real` 模式：Python 侧连接真实串口设备，并调用本仓库内的 C 算法后端，适合真机测试和可视化观察。

如果你是第一次接触这个项目，最重要的入口只有一个：

```bash
./run.sh sim
```

或：

```bash
./run.sh real
```

## 这个仓库里有什么

根目录主要包含这些部分：

- `run.sh`
  统一启动脚本。陌生人优先从这里开始，不需要先手动拼装命令。
- `python/`
  Python 侧逻辑，包括 MuJoCo 仿真、串口收发、Rerun 可视化、C 后端桥接。
- `c_interface/`
  宿主机上的 C 程序。`sim` 模式下负责实时控制回路，`real` 模式下负责重力补偿/控制计算。
- `stm32_code/`
  控制算法的可移植 C 实现，宿主机和 STM32 共用这一套核心逻辑。
- `stm32_port_package/`
  更适合直接移植进 STM32 工程的精简版本，目录内自带独立说明。
- `AM-D02-AemLURDF0413/`
  机械臂模型、URDF、mesh、launch 等资源。
- `results/`
  结果图和输出文件目录。

## 快速开始

### 1. 环境准备

建议在 Linux 环境下运行，至少准备：

- `python3`
- `pip`
- `gcc`
- `make`

安装 Python 依赖：

```bash
python3 -m pip install -r python/requirements.txt
```

`python/requirements.txt` 当前包含：

- `mujoco`
- `numpy`
- `rerun-sdk`
- `pyserial`

说明：

- `mujoco` 是仿真和部分真机可视化都会用到的核心依赖。
- `rerun-sdk` 用于实时可视化；没装也能跑部分流程，但会失去 Rerun 相关界面。
- `pyserial` 只在真机串口模式中需要。

### 2. 最短启动方式

仿真模式：

```bash
./run.sh sim
```

真机模式：

```bash
./run.sh real
```

`run.sh` 会自动做这些事情：

- 按模式编译 `c_interface/` 里的 C 程序。
- `sim` 模式下启动 Python UDP 仿真服务器，再启动前台 C 控制回路。
- `real` 模式下检查 `/dev/ttyUSB0`，然后启动 Python 串口控制程序。

## 两种模式分别在做什么

### `sim` 模式

命令：

```bash
./run.sh sim
```

运行链路：

1. 启动 `python/main_server.py`
2. Python 侧创建 MuJoCo 仿真环境，并在 `9876/UDP` 上等待 C 客户端
3. 启动 `c_interface/c_main`
4. C 程序从仿真获取状态，调用 `stm32_code/` 中的控制算法
5. 将关节力矩再发回 MuJoCo 执行

适合场景：

- 调控制算法
- 看坐标系转换是否正常
- 在不接真机的情况下验证控制回路

默认行为：

- 会打开 MuJoCo viewer（如果环境支持）
- 会启用 Rerun 记录（如果安装了 `rerun-sdk`）

### `real` 模式

命令：

```bash
./run.sh real
```

运行链路：

1. 编译并启动 `c_interface/serial_gravity_comp`
2. Python 侧运行 `python/serial_control.py`
3. 打开串口 `/dev/ttyUSB0`
4. 接收下位机反馈的关节状态
5. 调用 C 后端计算力矩、末端位姿和相关数据
6. 可选地把目标位姿继续发给下位机
7. 用 MuJoCo/Rerun 做在线观察

适合场景：

- 真实机械臂串口联调
- 本地算法与下位机反馈联合运行
- 观察目标位姿、反馈位姿、力矩和时延

## 第一次使用时建议这样做

如果你完全不熟悉这个项目，建议按下面顺序：

1. 先执行 `python3 -m pip install -r python/requirements.txt`
2. 再运行 `./run.sh sim`
3. 确认仿真模式能进入控制循环
4. 最后再接入真机，运行 `./run.sh real`

这样更容易把问题分清楚：是依赖问题、仿真问题，还是串口/真机问题。

## 常用可调参数

这个项目通过环境变量控制一些运行行为，常见的有：

```bash
AM_D02_ENABLE_VIEWER=1
AM_D02_ENABLE_RERUN=1
AM_D02_RERUN_LOG_STRIDE=25
AM_D02_REAL_VIEWER_FPS=30
AM_D02_RERUN_QUEUE_SIZE=512
AM_D02_SERIAL_FORWARD_TARGET=0
```

例如：

```bash
AM_D02_ENABLE_RERUN=0 ./run.sh sim
```

或：

```bash
AM_D02_SERIAL_FORWARD_TARGET=1 ./run.sh real
```

含义简述：

- `AM_D02_ENABLE_VIEWER`
  是否打开 MuJoCo 可视化窗口。
- `AM_D02_ENABLE_RERUN`
  是否启用 Rerun 实时可视化。
- `AM_D02_RERUN_LOG_STRIDE`
  Rerun 每隔多少步记录一次，值越大越省资源。
- `AM_D02_REAL_VIEWER_FPS`
  真机模式下 MuJoCo viewer 的刷新率。
- `AM_D02_RERUN_QUEUE_SIZE`
  Rerun 异步队列大小。
- `AM_D02_SERIAL_FORWARD_TARGET`
  真机模式下是否把目标位姿继续下发到串口设备。

## 串口和真机注意事项

当前真机模式默认串口是：

```text
/dev/ttyUSB0
```

启动 `./run.sh real` 时，脚本会尝试检查这个设备，并在有需要时执行：

```bash
sudo chmod o+rw /dev/ttyUSB0
```

如果真机跑不起来，优先检查：

- 设备是否真的挂在 `/dev/ttyUSB0`
- 当前用户是否有串口访问权限
- 下位机波特率是否匹配（当前代码里是 `115200`）
- 是否存在别的程序占用了串口

如果你的设备不是 `/dev/ttyUSB0`，需要修改 `python/serial_app.py` 里的 `SERIAL_PORT`。

## 目录说明：应该先看哪些文件

如果你想快速理解代码，推荐从这些入口看起：

- `run.sh`
  看项目怎么被整体拉起来。
- `python/main_server.py`
  看仿真模式的 Python 入口。
- `python/udp_server.py`
  看 Python MuJoCo 服务器如何和 C 程序通信。
- `python/serial_control.py`
  看真机模式的 Python 入口。
- `python/serial_app.py`
  看串口收发、可视化、C 后端调用主流程。
- `c_interface/main.c`
  看仿真模式下的 C 控制主循环。
- `c_interface/serial_gravity_comp.c`
  看真机模式下 Python 如何调用 C 算法后端。
- `stm32_code/`
  看可移植控制算法本体。

## 如果你只想移植到 STM32

优先看：

- `stm32_port_package/README.md`
- `stm32_port_package/core/`

这里已经整理成更适合单片机工程集成的版本。日常控制最关键的入口函数是：

- `stm_controller_init()`
- `stm_controller_step(...)`
- `stm_controller_reset()`

也就是说，如果你不是来跑宿主机仿真，而是只关心 STM32 落地，这个目录更适合作为起点。

## 常见问题

### 1. `./run.sh sim` 启动后没有画面

先检查：

- 是否安装了 `mujoco`
- 当前环境是否支持图形界面
- 是否人为设置了 `AM_D02_ENABLE_VIEWER=0`

如果只是想先验证链路是否通，可以临时关闭 viewer：

```bash
AM_D02_ENABLE_VIEWER=0 ./run.sh sim
```

### 2. Rerun 没起来

先检查是否安装了：

```bash
python3 -m pip install rerun-sdk
```

如果你不需要它，也可以直接关闭：

```bash
AM_D02_ENABLE_RERUN=0 ./run.sh sim
```

### 3. 真机模式报串口打开失败

优先检查：

- 线是否接好
- 设备名是否正确
- 是否有权限访问该串口
- 是否有其他串口助手占用

### 4. 想知道控制参数在哪里改

优先看：

- `python/config.py`
- `stm32_code/config.h`

前者主要影响宿主机侧运行行为，后者主要影响控制器本体参数和周期。

## 一个给陌生人的建议

第一次接手这个仓库，不要一上来就同时排查 MuJoCo、串口、STM32 和控制算法。最省时间的方式通常是：

1. 先跑通 `sim`
2. 再接真机
3. 最后再改控制参数

这样出问题时更容易定位是哪一层出了问题。
