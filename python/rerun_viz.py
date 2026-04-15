"""
Rerun 可视化模块
输出图表:
- 末端位置误差: X/Y/Z 轴各一个图表 (MuJoCo vs RBDL-Lite)
- 末端姿态误差: Roll/Pitch/Yaw 各一个图表 (MuJoCo vs RBDL-Lite)
- 关节力矩误差: 每个关节一个图表 (Δτ = RBDL-Lite - MuJoCo)
- 静态自稳: 各姿态的角速度和漂移
"""
import numpy as np

try:
    import rerun as rr
    import rerun.blueprint as rrb
    RERUN_AVAILABLE = True
except ImportError:
    RERUN_AVAILABLE = False
    print("警告: rerun-sdk 未安装，将跳过 Rerun 可视化")

from config import Config

_AXIS_COLORS = {
    'X':     [230, 80, 80],
    'Y':     [80, 190, 80],
    'Z':     [80, 120, 230],
    'Roll':  [230, 80, 80],
    'Pitch': [80, 190, 80],
    'Yaw':   [80, 120, 230],
}

_MODE_COLORS = {
    'mujoco': [50, 150, 230],
    'c_engine': [230, 100, 50],
}

_JOINT_COLORS = [
    [230, 50, 50],
    [230, 140, 30],
    [210, 200, 30],
    [50, 200, 50],
    [50, 200, 200],
    [50, 80, 230],
    [150, 50, 230],
]


_POSE_NAME_MAP = {
    "零位": "zero",
    "伸展位": "extend",
    "随机位": "random",
}

def _safe_pose_name(name: str) -> str:
    """将中文姿态名转为 ASCII 安全名称"""
    return _POSE_NAME_MAP.get(name, f"pose_{hash(name) % 10000}")


def quaternion_to_euler(w, x, y, z):
    """
    四元数转欧拉角 (Roll, Pitch, Yaw) - ZYX 顺序
    """
    import math
    # roll (x-axis rotation)
    sinr_cosp = 2 * (w * x + y * z)
    cosr_cosp = 1 - 2 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    # pitch (y-axis rotation)
    sinp = 2 * (w * y - z * x)
    if abs(sinp) >= 1:
        pitch = math.copysign(math.pi / 2, sinp) # use 90 degrees if out of range
    else:
        pitch = math.asin(sinp)

    # yaw (z-axis rotation)
    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)

    return np.array([roll, pitch, yaw])

def quat_to_euler(quat):
    """适配 [w, x, y, z] 或 [x, y, z, w] 的包装"""
    # 假设输入是 [w, x, y, z]
    return quaternion_to_euler(quat[0], quat[1], quat[2], quat[3])

def quaternion_multiply(q1: np.ndarray, q2: np.ndarray) -> np.ndarray:
    """四元数乘法 [w, x, y, z]"""
    w1, x1, y1, z1 = q1
    w2, x2, y2, z2 = q2
    return np.array([
        w1*w2 - x1*x2 - y1*y2 - z1*z2,
        w1*x2 + x1*w2 + y1*z2 - z1*y2,
        w1*y2 - x1*z2 + y1*w2 + z1*x2,
        w1*z2 + x1*y2 - y1*x2 + z1*w2
    ])

def compute_rotation_error(quat_actual: np.ndarray, quat_desired: np.ndarray) -> np.ndarray:
    """
    计算旋转误差 (欧拉角表示, 单位 deg)
    quat_actual: (N, 4), quat_desired: (N, 4), 格式 [w,x,y,z]
    返回: (N, 3) [roll_err, pitch_err, yaw_err] in degrees
    """
    N = len(quat_actual)
    errors = np.zeros((N, 3))
    for i in range(N):
        q_act = quat_actual[i]
        q_des = quat_desired[i]
        q_des_inv = np.array([q_des[0], -q_des[1], -q_des[2], -q_des[3]])
        q_err = quaternion_multiply(q_act, q_des_inv)
        if q_err[0] < 0:
            q_err = -q_err
        errors[i] = quat_to_euler(q_err)
    return np.degrees(errors)


def compute_rotation_error_single(quat_actual: np.ndarray, quat_desired: np.ndarray) -> np.ndarray:
    """单步姿态误差，避免为单个样本构造额外批量数组。"""
    q_des_inv = np.array([quat_desired[0], -quat_desired[1], -quat_desired[2], -quat_desired[3]])
    q_err = quaternion_multiply(quat_actual, q_des_inv)
    if q_err[0] < 0:
        q_err = -q_err
    return np.rad2deg(quat_to_euler(q_err))

def init_rerun(app_name: str = "AM-D02 Simulation"):
    """初始化 Rerun (不发送 Blueprint，等数据写入后再发)"""
    if not RERUN_AVAILABLE:
        return False
    rr.init(app_name, spawn=True)
    # 强制在某些环境下也尝试打开浏览器界面
    # rr.spawn() 
    return True


def _setup_trajectory_styles():
    """设置轨迹跟踪相关的曲线样式"""
    for axis in ('X', 'Y', 'Z'):
        rr.log(f"position_error/{axis}/mujoco",
               rr.SeriesLines(colors=[_MODE_COLORS['mujoco']],
                              names=["MuJoCo"],
                              widths=[2]),
               static=True)
        rr.log(f"position_error/{axis}/c_engine",
               rr.SeriesLines(colors=[_MODE_COLORS['c_engine']],
                              names=["RBDL-Lite"],
                              widths=[2]),
               static=True)
    
    for axis in ('Roll', 'Pitch', 'Yaw'):
        rr.log(f"rotation_error/{axis}/mujoco",
               rr.SeriesLines(colors=[_MODE_COLORS['mujoco']],
                              names=["MuJoCo"],
                              widths=[2]),
               static=True)
        rr.log(f"rotation_error/{axis}/c_engine",
               rr.SeriesLines(colors=[_MODE_COLORS['c_engine']],
                              names=["RBDL-Lite"],
                              widths=[2]),
               static=True)

    for i in range(Config.NUM_JOINTS):
        rr.log(f"torque_error/J{i+1}/delta",
               rr.SeriesLines(colors=[_JOINT_COLORS[i]],
                              names=["RBDL-Lite - MuJoCo"],
                              widths=[2]),
               static=True)
        # 为“所有轴的误差合在一个图表”准备样式
        rr.log(f"torque_error_all/J{i+1}",
               rr.SeriesLines(colors=[_JOINT_COLORS[i]],
                              names=[f"J{i+1} Error"],
                              widths=[2]),
               static=True)

        rr.log(f"feedforward/J{i+1}/diff",
               rr.SeriesLines(colors=[_JOINT_COLORS[i]],
                              names=[f"J{i+1} C+G Diff (RBDL-Lite - MuJoCo)"],
                              widths=[2]),
               static=True)

        rr.log(f"total_torque/J{i+1}/c_engine",
               rr.SeriesLines(colors=[_MODE_COLORS['c_engine']],
                              names=["RBDL-Lite (Total)"],
                              widths=[2]),
               static=True)

        rr.log(f"total_torque/J{i+1}/actual",
               rr.SeriesLines(colors=[_MODE_COLORS['mujoco']],
                              names=["Actual Torque Sensor"],
                              widths=[1.5]),
               static=True)

        rr.log(f"torque_gap/J{i+1}/delta",
               rr.SeriesLines(colors=[_JOINT_COLORS[i]],
                              names=["STM32 Total - Actual Sensor"],
                              widths=[2]),
               static=True)

    rr.log("performance/c_engine_time",
           rr.SeriesLines(colors=[_MODE_COLORS['c_engine']],
                          names=["Python Control Step Time (ms)"],
                          widths=[2]),
           static=True)

    # 世界坐标轴（原点）- 静态参考
    rr.log(
        "trajectory_3d/origin",
        rr.Arrows3D(
            vectors=[[0.1, 0, 0], [0, 0.1, 0], [0, 0, 0.1]],
            colors=[[220, 50, 50], [50, 220, 50], [50, 50, 220]],
        ),
        static=True,
    )

    # 关节状态曲线样式
    for i in range(Config.NUM_JOINTS):
        rr.log(f"joint_state/q/J{i+1}", rr.SeriesLines(colors=[_JOINT_COLORS[i]], names=[f"J{i+1} pos"], widths=[2]), static=True)
        rr.log(f"joint_state/qd/J{i+1}", rr.SeriesLines(colors=[_JOINT_COLORS[i]], names=[f"J{i+1} vel"], widths=[2]), static=True)

    # 延时曲线样式
    rr.log("performance/uart_latency",
           rr.SeriesLines(colors=[[230, 150, 50]],
                          names=["UART Loop Period (ms)"],
                          widths=[2]),
           static=True)

    rr.log("performance/uart_cycle_hz",
           rr.SeriesLines(colors=[[230, 120, 40]],
                          names=["UART Loop Rate (Hz)"],
                          widths=[2]),
           static=True)

    rr.log("performance/uart_transfer_kbps",
           rr.SeriesLines(colors=[[230, 180, 80]],
                          names=["UART Effective Throughput (kbps)"],
                          widths=[2]),
           static=True)
    
    # STM32 计算耗时样式
    rr.log("performance/stm32_calc_time",
           rr.SeriesLines(colors=[[100, 200, 100]],
                          names=["STM32 Algorithm Calc Time (ms)"],
                          widths=[2]),
           static=True)

    rr.log("performance/stm32_calc_hz",
           rr.SeriesLines(colors=[[80, 180, 220]],
                          names=["STM32 Calc Rate (Hz)"],
                          widths=[2]),
           static=True)

def setup_realtime_styles():
    """设置交互式 Rerun 的曲线样式和试图蓝图，在仿真启动前调用"""
    if not RERUN_AVAILABLE: return
    _setup_trajectory_styles()
    
    # Position tracking
    for axis in ('X', 'Y', 'Z'):
        rr.log(f"tracking/pos/{axis}",
               rr.SeriesLines(colors=[[230, 100, 50], [80, 220, 80]], 
                              names=["Actual", "Desired"], 
                              widths=[2.0, 1.0]),
               static=True)

    # Rotation tracking
    for axis in ('Roll', 'Pitch', 'Yaw'):
        rr.log(f"tracking/rot/{axis}",
               rr.SeriesLines(colors=[[50, 150, 230], [80, 220, 80]], 
                              names=["Actual", "Desired"],
                              widths=[2.0, 1.0]),
               static=True)
    
    # Error tracking styles
    for axis in ('X', 'Y', 'Z', 'Roll', 'Pitch', 'Yaw'):
        color = _AXIS_COLORS.get(axis, [200, 200, 200])
        rr.log(f"error/{axis}", 
               rr.SeriesLines(colors=[color], names=[f"{axis} Error"], widths=[2.0]),
               static=True)
    
    # 增加初始数据点，确保 Viewer 启动时图表可见
    rr.set_time_seconds("time", 0.0)
    for axis in ('X', 'Y', 'Z'):
        rr.log(f"tracking/pos/{axis}/actual", rr.Scalars(0.0))
        rr.log(f"tracking/pos/{axis}/desired", rr.Scalars(0.0))
        rr.log(f"error/{axis}", rr.Scalars(0.0))
    for axis in ('Roll', 'Pitch', 'Yaw'):
        rr.log(f"tracking/rot/{axis}/actual", rr.Scalars(0.0))
        rr.log(f"tracking/rot/{axis}/desired", rr.Scalars(0.0))
        rr.log(f"error/{axis}", rr.Scalars(0.0))
    for i in range(Config.NUM_JOINTS):
        rr.log(f"joint_state/q/J{i+1}", rr.Scalars(0.0))
        rr.log(f"joint_state/qd/J{i+1}", rr.Scalars(0.0))
        rr.log(f"total_torque/J{i+1}/c_engine", rr.Scalars(0.0))
        rr.log(f"total_torque/J{i+1}/actual", rr.Scalars(0.0))
        rr.log(f"torque_gap/J{i+1}/delta", rr.Scalars(0.0))
    rr.log("performance/c_engine_time", rr.Scalars(0.0))
    rr.log("performance/uart_latency", rr.Scalars(0.0))
    rr.log("performance/uart_cycle_hz", rr.Scalars(0.0))
    rr.log("performance/uart_transfer_kbps", rr.Scalars(0.0))
    rr.log("performance/stm32_calc_time", rr.Scalars(0.0))
    rr.log("performance/stm32_calc_hz", rr.Scalars(0.0))

    # Note: TimeseriesView origin should be the parent entity to show multiple children as series

    pos_views = []
    for axis in ('X', 'Y', 'Z'):
        pos_views.append(rrb.TimeSeriesView(
            name=f"EE Position {axis} (m)", origin=f"/tracking/pos/{axis}",
        ))
    
    rot_views = []
    for axis in ('Roll', 'Pitch', 'Yaw'):
        rot_views.append(rrb.TimeSeriesView(
            name=f"EE Rotation {axis} (deg)", origin=f"/tracking/rot/{axis}",
        ))

    pos_err_views = []
    for axis in ('X', 'Y', 'Z'):
        pos_err_views.append(rrb.TimeSeriesView(
            name=f"Position Error {axis} (mm)", origin=f"/error/{axis}",
        ))
    
    rot_err_views = []
    for axis in ('Roll', 'Pitch', 'Yaw'):
        rot_err_views.append(rrb.TimeSeriesView(
            name=f"Rotation Error {axis} (deg)", origin=f"/error/{axis}",
        ))

    total_torque_views = []
    for i in range(Config.NUM_JOINTS):
        short = Config.JOINT_NAMES[i].replace('ArmL', '').replace('_Joint', '')
        total_torque_views.append(rrb.TimeSeriesView(
            name=f"J{i+1} ({short}) Total Torque (N*m)", origin=f"/total_torque/J{i+1}",
        ))

    torque_gap_views = []
    for i in range(Config.NUM_JOINTS):
        short = Config.JOINT_NAMES[i].replace('ArmL', '').replace('_Joint', '')
        torque_gap_views.append(rrb.TimeSeriesView(
            name=f"J{i+1} ({short}) Torque Gap (N*m)", origin=f"/torque_gap/J{i+1}",
        ))

    joint_q_view = rrb.TimeSeriesView(name="Joint Positions (rad)", origin="/joint_state/q")
    joint_qd_view = rrb.TimeSeriesView(name="Joint Velocities (rad/s)", origin="/joint_state/qd")
    
    uart_log_view = rrb.TextLogView(name="UART Protocol Log", origin="/uart_log")
    
    latency_view = rrb.TimeSeriesView(
        name="UART Communication Latency (ms)", origin="/performance/uart_latency",
    )

    uart_cycle_rate_view = rrb.TimeSeriesView(
        name="UART Loop Rate (Hz)", origin="/performance/uart_cycle_hz",
    )

    uart_transfer_rate_view = rrb.TimeSeriesView(
        name="UART Effective Throughput (kbps)", origin="/performance/uart_transfer_kbps",
    )
    
    stm32_time_view = rrb.TimeSeriesView(
        name="STM32 Calculation Time (ms)", origin="/performance/stm32_calc_time",
    )

    stm32_rate_view = rrb.TimeSeriesView(
        name="STM32 Calculation Rate (Hz)", origin="/performance/stm32_calc_hz",
    )

    python_time_view = rrb.TimeSeriesView(
        name="Python Control Step Time (ms)", origin="/performance/c_engine_time",
    )

    blueprint = rrb.Blueprint(
        rrb.Vertical(
            rrb.Tabs(
                rrb.Spatial3DView(name="3D Interactive", origin="/trajectory_3d"),
                rrb.Vertical(
                    rrb.Horizontal(*pos_views),
                    rrb.Horizontal(*rot_views),
                    name="EE Tracking (Actual vs Desired)"
                ),
                rrb.Vertical(
                    rrb.Horizontal(*pos_err_views),
                    rrb.Horizontal(*rot_err_views),
                    name="EE Tracking Error"
                ),
                rrb.Vertical(joint_q_view, joint_qd_view, name="Joint States"),
                rrb.Vertical(*total_torque_views, name="Total Torque (from STM32)"),
                rrb.Vertical(*torque_gap_views, name="Total Torque Gap"),
                rrb.Vertical(uart_log_view, latency_view, name="UART Protocol"),
                rrb.Vertical(
                    python_time_view,
                    uart_cycle_rate_view,
                    uart_transfer_rate_view,
                    stm32_time_view,
                    stm32_rate_view,
                    name="Performance Rates",
                ),
            )
        ),
        collapse_panels=True,
    )
    rr.send_blueprint(blueprint)

def log_realtime_step(
    t: float,
    pos_actual: np.ndarray,
    pos_desired: np.ndarray,
    quat_actual: np.ndarray,
    quat_desired: np.ndarray,
    tau_total: np.ndarray,
    cycle_time: float,
    q: np.ndarray = None,
    qd: np.ndarray = None,
    tau_actual: np.ndarray = None,
    rx_str: str = None,
    tx_str: str = None,
    tx_label: str = "Torques",
    step_count: int = 0,
    uart_latency_ms: float = None,
    uart_cycle_hz: float = None,
    uart_transfer_kbps: float = None,
    stm32_calc_time_ms: float = None,
    stm32_calc_hz: float = None,
):
    """单步记录交互式仿真数据"""
    if not RERUN_AVAILABLE: return
    if Config.RERUN_LOG_STRIDE > 1 and step_count % Config.RERUN_LOG_STRIDE != 0:
        return
    rr.set_time_seconds("time", t)
    
    # Joint States
    if q is not None:
        for i in range(len(q)):
            rr.log(f"joint_state/q/J{i+1}", rr.Scalars(float(q[i])))
    if qd is not None:
        for i in range(len(qd)):
            rr.log(f"joint_state/qd/J{i+1}", rr.Scalars(float(qd[i])))
    
    # Position Tracking & Error
    for i, axis in enumerate(('X', 'Y', 'Z')):
        rr.log(f"tracking/pos/{axis}/actual", rr.Scalars(float(pos_actual[i])))
        rr.log(f"tracking/pos/{axis}/desired", rr.Scalars(float(pos_desired[i])))
        # Error in mm
        rr.log(f"error/{axis}", rr.Scalars(float((pos_actual[i] - pos_desired[i]) * 1000.0)))
        
    # Rotation Tracking & Error
    rot_actual = quat_to_euler(quat_actual)
    rot_desired = quat_to_euler(quat_desired)
    rot_err = compute_rotation_error_single(quat_actual, quat_desired)
    rot_actual_deg = np.rad2deg(rot_actual)
    rot_desired_deg = np.rad2deg(rot_desired)

    for i, axis in enumerate(('Roll', 'Pitch', 'Yaw')):
        rr.log(f"tracking/rot/{axis}/actual", rr.Scalars(float(rot_actual_deg[i])))
        rr.log(f"tracking/rot/{axis}/desired", rr.Scalars(float(rot_desired_deg[i])))
        # Error in deg
        rr.log(f"error/{axis}", rr.Scalars(float(rot_err[i])))
        
    # Torques
    for j in range(len(tau_total)):
        rr.log(f"total_torque/J{j+1}/c_engine", rr.Scalars(float(tau_total[j])))
        if tau_actual is not None:
            rr.log(f"total_torque/J{j+1}/actual", rr.Scalars(float(tau_actual[j])))
            rr.log(f"torque_gap/J{j+1}/delta", rr.Scalars(float(tau_total[j] - tau_actual[j])))
            
    # Text Log for Sent/Received Data (Throttled to 10Hz to prevent lag at 1kHz loop)
    if rx_str and tx_str:
        rr.log("uart_log", rr.TextLog(f"[{step_count}] RX (Positions): {rx_str}\n[{step_count}] TX ({tx_label}): {tx_str}"))

    # Performance
    rr.log("performance/c_engine_time", rr.Scalars(float(cycle_time)))
    if uart_latency_ms is not None:
        rr.log("performance/uart_latency", rr.Scalars(float(uart_latency_ms)))
    if uart_cycle_hz is not None:
        rr.log("performance/uart_cycle_hz", rr.Scalars(float(uart_cycle_hz)))
    if uart_transfer_kbps is not None:
        rr.log("performance/uart_transfer_kbps", rr.Scalars(float(uart_transfer_kbps)))
    if stm32_calc_time_ms is not None:
        rr.log("performance/stm32_calc_time", rr.Scalars(float(stm32_calc_time_ms)))
    if stm32_calc_hz is not None:
        rr.log("performance/stm32_calc_hz", rr.Scalars(float(stm32_calc_hz)))
    
    # 3D
    rr.log("trajectory_3d/actual_ee", 
           rr.Points3D([pos_actual], colors=[[230, 100, 50]], radii=0.015, labels=["Actual"]))
    rr.log("trajectory_3d/target_goal", 
           rr.Points3D([pos_desired], colors=[[80, 220, 80]], radii=0.015, labels=["Target"]))
    
    # 还可以画一条连线表示偏差
    rr.log("trajectory_3d/error_line",
           rr.LineStrips3D([[pos_actual, pos_desired]], colors=[[255, 0, 0]], radii=0.002))



