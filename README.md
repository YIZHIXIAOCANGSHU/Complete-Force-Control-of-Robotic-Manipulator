# AM-DPBSURDF0422 Sim/MC 使用说明

这个仓库现在保留 AM-DPBSURDF0422 左右双臂 14 轴的仿真链路：

- `sim`：Python 侧启动 MuJoCo 仿真服务器，C 侧运行实时控制回路。
- `mc`：复用同一个 MuJoCo 模型，对左右双臂关节空间做蒙特卡罗采样并输出两侧末端位姿范围。

真机串口、SocketCAN、旧 AM-D02-AemLURDF0413 模型和相关入口已经移除。

## 快速开始

安装依赖：

```bash
python3 -m pip install -r python/requirements.txt
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

无窗口运行蒙特卡罗：

```bash
./run.sh mc -n 10000 --seed 42 --no-viewer
```

## 仓库结构

- `AM-DPBSURDF0422/`：当前 MuJoCo/URDF 模型、mesh 和 ROS 描述文件。
- `python/`：MuJoCo 仿真、UDP 服务、Monte Carlo 工具和 Rerun 可视化。
- `c_interface/`：宿主机 C 控制入口，`sim` 模式下连接 Python UDP 仿真服务器。
- `stm32_code/`：左右两个独立 7 轴控制器上下文、运动学、动力学和轨迹逻辑。
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

默认情况下，腰部/躯干等非双臂受控关节会在临时 MuJoCo 模型中编译为固定关节，
让仿真自由度和 C 控制器的 14 轴输出一致，避免未接入控制器的 3 个自由度在动力学中耦合。
如需恢复完整 17 DOF 模型，可设置 `AM_D02_FIX_UNCONTROLLED_JOINTS=0`，此时腰/躯干 3 DOF
仍不会参与 C 控制输出。

仿真 UDP 协议使用 43 个 `double` 输入和 14 个 `double` 输出：

```text
q[14] + qd[14]
+ left_target_pos[3] + left_target_quat[4]
+ right_target_pos[3] + right_target_quat[4]
+ dt[1]
```

## 常用环境变量

```bash
AM_D02_PYTHON=/path/to/python
AM_D02_ENABLE_VIEWER=1
AM_D02_ENABLE_RERUN=1
AM_D02_RERUN_LOG_STRIDE=10
AM_D02_FIX_UNCONTROLLED_JOINTS=1
AM_D02_MC_SAMPLES=50000
AM_D02_MC_SEED=42
AM_D02_MC_PROGRESS_INTERVAL=200
AM_D02_MC_MAX_VIS_POINTS=3000
AM_D02_MC_MAX_HULL_POINTS=12000
```

## 开发与验证

编译 C 控制器：

```bash
make -C c_interface clean && make -C c_interface c_main
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
如需对比旧的完整双臂自由度行为：

```bash
AM_D02_FIX_UNCONTROLLED_JOINTS=0 AM_D02_ENABLE_VIEWER=0 AM_D02_ENABLE_RERUN=0 ./run.sh sim
```
