# AM-D02 参数辨识与载荷辨识

当前分支聚焦辨识入口：

- `param-id-sim`：仿真参数辨识。
- `param-id-real`：实机参数辨识采集。
- `payload-id-sim`：末端载荷辨识仿真。

## 快速开始

```bash
python3 -m venv .venv
source .venv/bin/activate
python -m pip install -U pip
python -m pip install -r python/requirements.txt
./run.sh
```

也可以直接启动：

```bash
./run.sh param-id-sim
./run.sh param-id-real
./run.sh payload-id-sim
```

## 目录

- `src/robot_control/`：全部 Python 源码。
- `src/robot_control/modes/`：辨识运行模式入口，只负责编排启动流程。
  - `param_id_real/`：实机参数辨识采集。
  - `param_id_sim/`：仿真参数辨识采集与候选验证。
  - `payload_id_sim/`：仿真末端载荷辨识。
- `src/robot_control/param_id/`：参数辨识算法库，包含激励轨迹、预处理、回归矩阵、求解、诊断和报告。
- `src/robot_control/dynamics/`：Pinocchio 动力学和重力补偿后端。
- `src/robot_control/hardware/usb2fdcan/`：USB2FDCAN 配置、协议、反馈解码、SocketCAN 传输和高层电机传输。
- `src/robot_control/shared/`：跨运行模式复用的 runtime、MuJoCo、Rerun、状态和坐标变换工具。
- `assets/robot_model/`：URDF、mesh、ROS launch/config 资源。
- `outputs/`：运行输出。
- `tests/`：测试。
- `run.sh`：四模式启动脚本。

## SocketCAN

```bash
sudo ip link set can0 down
sudo ip link set can0 type can bitrate 1000000 dbitrate 5000000 fd on
sudo ip link set can0 up
```

```bash
AM_D02_CAN_INTERFACE=can1 ./run.sh real
AM_D02_CAN_INTERFACE=can1 ./run.sh param-id-real
```

实机辨识会驱动真实机械臂，启动前确认急停、机械限位、工作空间和 CAN 通信状态。
