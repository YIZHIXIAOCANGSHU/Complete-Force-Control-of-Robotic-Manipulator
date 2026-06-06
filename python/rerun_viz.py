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
from rerun_runtime.constants import (
    ACTUAL_COLOR as _ACTUAL_COLOR,
    ARM_DISPLAY_NAMES as _ARM_DISPLAY_NAMES,
    ARM_LABELS as _ARM_LABELS,
    AXIS_COLORS as _AXIS_COLORS,
    ERROR_COLOR as _ERROR_COLOR,
    JOINT_COLORS as _JOINT_COLORS,
    LEFT_ARM_COLOR as _LEFT_ARM_COLOR,
    LIMIT_COLOR as _LIMIT_COLOR,
    MODE_COLORS as _MODE_COLORS,
    POSITION_DISPLAY_UNIT as _POSITION_DISPLAY_UNIT,
    REFERENCE_COLOR as _REFERENCE_COLOR,
    RIGHT_ARM_COLOR as _RIGHT_ARM_COLOR,
    SAFETY_LOG_MARGIN_RAD as _SAFETY_LOG_MARGIN_RAD,
    SAFETY_LOG_MARGIN_RAD_S as _SAFETY_LOG_MARGIN_RAD_S,
    SAFETY_TEXT_LOG_INTERVAL_STEPS as _SAFETY_TEXT_LOG_INTERVAL_STEPS,
    TARGET_COLOR as _TARGET_COLOR,
)
from rerun_runtime.math_utils import (
    _as_arm_array,
    _position_to_display_units,
    _safe_pose_name,
    _torque_utilization,
    compute_rotation_error,
    compute_rotation_error_single,
    quat_to_euler,
    quaternion_multiply,
    quaternion_to_euler,
)
from rerun_runtime.performance import (
    DETAILED_PERFORMANCE_PATHS as _DETAILED_PERFORMANCE_PATHS,
    ESSENTIAL_PERFORMANCE_PATHS as _ESSENTIAL_PERFORMANCE_PATHS,
    _rerun_should_log_perf,
)
from rerun_runtime.safety import (
    _format_safety_warnings,
    _joint_safe_limits_rad,
    _joint_safety_margins,
)

_last_safety_warning_signature: str | None = None
_last_safety_warning_step: int | None = None


def _log_perf_scalar(path: str, value) -> None:
    if value is None or not _rerun_should_log_perf(path):
        return
    rr.log(path, rr.Scalars(float(value)))


def _safety_warning_signature(safety_warning: str) -> str:
    parts = []
    for item in safety_warning.split("; "):
        label = item.split("=", 1)[0]
        parts.append(label)
    return "; ".join(parts)


def _should_log_safety_warning(safety_warning: str, step_count: int) -> bool:
    global _last_safety_warning_signature, _last_safety_warning_step

    signature = _safety_warning_signature(safety_warning)
    if signature != _last_safety_warning_signature:
        _last_safety_warning_signature = signature
        _last_safety_warning_step = int(step_count)
        return True

    last_step = _last_safety_warning_step
    if last_step is None or int(step_count) - last_step >= _SAFETY_TEXT_LOG_INTERVAL_STEPS:
        _last_safety_warning_step = int(step_count)
        return True

    return False


def _reset_safety_warning_throttle() -> None:
    global _last_safety_warning_signature, _last_safety_warning_step
    _last_safety_warning_signature = None
    _last_safety_warning_step = None

def init_rerun(app_name: str = "AM-D02 Simulation"):
    """初始化 Rerun (不发送 Blueprint，等数据写入后再发)"""
    if not RERUN_AVAILABLE:
        return False
    rr.init(app_name, spawn=True)
    # 强制在某些环境下也尝试打开浏览器界面
    # rr.spawn() 
    return True


def _setup_trajectory_styles():
    """设置 3D 和性能曲线的基础样式。"""
    rr.log(
        "trajectory_3d/origin",
        rr.Arrows3D(
            vectors=[[0.1, 0, 0], [0, 0.1, 0], [0, 0, 0.1]],
            colors=[[220, 50, 50], [50, 220, 50], [50, 50, 220]],
        ),
        static=True,
    )

    performance_styles = {
        "performance/c_engine_time": (_MODE_COLORS["c_engine"], "Python Control Step Time (ms)", 2),
        "performance/link_latency": ([230, 150, 50], "Control Link Period (ms)", 2),
        "performance/link_cycle_hz": ([230, 120, 40], "Control Link Rate (Hz)", 2),
        "performance/link_transfer_kbps": ([230, 180, 80], "Control Link Throughput (kbps)", 2),
        "performance/stm32_calc_time": ([100, 200, 100], "STM32 Algorithm Calc Time (ms)", 2),
        "performance/stm32_calc_hz": ([80, 180, 220], "STM32 Calc Rate (Hz)", 2),
        "performance/feedback_wait_ms": ([120, 180, 240], "Feedback Wait (ms)", 2),
        "performance/tx_overwrite_count": ([220, 120, 80], "TX Pending Overwrites", 2),
        "performance/can_backpressure_count": ([180, 80, 160], "CAN Backpressure Count", 2),
        "performance/control_target_hz": ([120, 120, 120], "Control Target Hz", 1.5),
        "performance/sim_target_hz": ([120, 120, 120], "Sim Target Hz", 1.5),
        "performance/sim_service_ms": ([50, 150, 230], "Sim UDP Service Time (ms)", 2),
        "performance/sim_mujoco_step_ms": ([80, 190, 80], "Sim MuJoCo Step Time (ms)", 2),
        "performance/sim_state_packet_ms": ([230, 180, 80], "Sim State Packet Time (ms)", 2),
        "performance/sim_rerun_overwrite_count": ([220, 120, 80], "Sim Rerun Overwrites", 2),
        "performance/sim_rerun_drop_count": ([210, 80, 80], "Sim Rerun Drops", 2),
        "performance/sim_socket_timeout_count": ([180, 80, 160], "Sim Socket Timeouts", 2),
        "performance/viewer_sync_count": ([90, 170, 230], "Viewer Sync Count", 2),
        "performance/viewer_skip_count": ([230, 120, 60], "Viewer Skip Count", 2),
        "performance/viewer_sync_ms": ([120, 200, 120], "Viewer Sync Time (ms)", 2),
        "performance/viewer_lock_wait_ms": ([200, 170, 80], "Viewer Lock Wait (ms)", 2),
    }
    for path, (color, name, width) in performance_styles.items():
        if not _rerun_should_log_perf(path):
            continue
        rr.log(
            path,
            rr.SeriesLines(colors=[color], names=[name], widths=[width]),
            static=True,
        )


def _setup_arm_realtime_styles() -> None:
    """为左右臂合并曲线写入 Rerun SeriesLines 样式。"""
    for arm_label, arm_name in zip(_ARM_LABELS, _ARM_DISPLAY_NAMES):
        for axis in ("X", "Y", "Z"):
            rr.log(
                f"arms/{arm_label}/position/{axis}",
                rr.SeriesLines(
                    colors=[_ACTUAL_COLOR, _TARGET_COLOR, _REFERENCE_COLOR],
                    names=[
                        f"{arm_name} {axis} Actual",
                        f"{arm_name} {axis} Target",
                        f"{arm_name} {axis} Reference",
                    ],
                    widths=[2.0, 1.2, 1.5],
                ),
                static=True,
            )
            rr.log(
                f"arms/{arm_label}/position_error/{axis}",
                rr.SeriesLines(
                    colors=[_ERROR_COLOR],
                    names=[f"{arm_name} {axis} Error"],
                    widths=[2.0],
                ),
                static=True,
            )
            rr.log(
                f"arms/{arm_label}/tcp_velocity/{axis}",
                rr.SeriesLines(
                    colors=[_AXIS_COLORS[axis]],
                    names=[f"{arm_name} TCP Velocity {axis}"],
                    widths=[2.0],
                ),
                static=True,
            )
        rr.log(
            f"arms/{arm_label}/tcp_speed/linear",
            rr.SeriesLines(
                colors=[_ACTUAL_COLOR],
                names=[f"{arm_name} TCP Linear Speed"],
                widths=[2.0],
            ),
            static=True,
        )
        for axis in ("Roll", "Pitch", "Yaw"):
            rr.log(
                f"arms/{arm_label}/rotation/{axis}",
                rr.SeriesLines(
                    colors=[_ACTUAL_COLOR, _TARGET_COLOR, _REFERENCE_COLOR],
                    names=[
                        f"{arm_name} {axis} Actual",
                        f"{arm_name} {axis} Target",
                        f"{arm_name} {axis} Reference",
                    ],
                    widths=[2.0, 1.2, 1.5],
                ),
                static=True,
            )
            rr.log(
                f"arms/{arm_label}/rotation_error/{axis}",
                rr.SeriesLines(
                    colors=[_ERROR_COLOR],
                    names=[f"{arm_name} {axis} Error"],
                    widths=[2.0],
                ),
                static=True,
            )
        for i in range(Config.ARM_JOINTS):
            joint = f"J{i + 1}"
            color = _JOINT_COLORS[i]
            rr.log(
                f"arms/{arm_label}/joint_q/{joint}",
                rr.SeriesLines(colors=[color], names=[f"{arm_name} {joint} q"], widths=[2.0]),
                static=True,
            )
            rr.log(
                f"arms/{arm_label}/joint_qd/{joint}",
                rr.SeriesLines(colors=[color], names=[f"{arm_name} {joint} qd"], widths=[2.0]),
                static=True,
            )
            rr.log(
                f"arms/{arm_label}/torque/{joint}",
                rr.SeriesLines(colors=[color], names=[f"{arm_name} {joint} Tau"], widths=[2.0]),
                static=True,
            )
            rr.log(
                f"arms/{arm_label}/torque_raw/{joint}",
                rr.SeriesLines(colors=[_LIMIT_COLOR], names=[f"{arm_name} {joint} Raw Tau"], widths=[1.5]),
                static=True,
            )
            rr.log(
                f"arms/{arm_label}/torque_actual/{joint}",
                rr.SeriesLines(colors=[color], names=[f"{arm_name} {joint} Actual Tau"], widths=[1.5]),
                static=True,
            )
            rr.log(
                f"arms/{arm_label}/torque_gap/{joint}",
                rr.SeriesLines(colors=[_ERROR_COLOR], names=[f"{arm_name} {joint} Tau Gap"], widths=[2.0]),
                static=True,
            )
            rr.log(
                f"arms/{arm_label}/velocity_margin/{joint}",
                rr.SeriesLines(colors=[color], names=[f"{arm_name} {joint} Velocity Margin"], widths=[2.0]),
                static=True,
            )
            rr.log(
                f"arms/{arm_label}/limit_margin_low/{joint}",
                rr.SeriesLines(colors=[color], names=[f"{arm_name} {joint} Low Limit Margin"], widths=[1.5]),
                static=True,
            )
            rr.log(
                f"arms/{arm_label}/limit_margin_high/{joint}",
                rr.SeriesLines(colors=[color], names=[f"{arm_name} {joint} High Limit Margin"], widths=[1.5]),
                static=True,
            )


def _setup_fast_status_styles() -> None:
    """写入第一屏快速状态曲线的 SeriesLines 样式。"""
    for arm_label, arm_name in zip(_ARM_LABELS, _ARM_DISPLAY_NAMES):
        rr.log(
            f"arms/{arm_label}/fast_status/position_error_norm_mm",
            rr.SeriesLines(
                colors=[_ERROR_COLOR],
                names=[f"{arm_name} Position Error Norm"],
                widths=[2.5],
            ),
            static=True,
        )
        rr.log(
            f"arms/{arm_label}/fast_status/reference_error_norm_mm",
            rr.SeriesLines(
                colors=[_REFERENCE_COLOR],
                names=[f"{arm_name} Reference Error Norm"],
                widths=[2.5],
            ),
            static=True,
        )
        rr.log(
            f"arms/{arm_label}/fast_status/rotation_error_norm_deg",
            rr.SeriesLines(
                colors=[_ERROR_COLOR],
                names=[f"{arm_name} Rotation Error Norm"],
                widths=[2.5],
            ),
            static=True,
        )
        rr.log(
            f"arms/{arm_label}/fast_status/tcp_speed_mps",
            rr.SeriesLines(
                colors=[_ACTUAL_COLOR, _LIMIT_COLOR],
                names=[f"{arm_name} TCP Speed", "TCP Speed Limit"],
                widths=[2.5, 1.5],
            ),
            static=True,
        )
        rr.log(
            f"arms/{arm_label}/fast_status/torque_utilization",
            rr.SeriesLines(
                colors=[_ERROR_COLOR, _ACTUAL_COLOR, _LIMIT_COLOR],
                names=[
                    f"{arm_name} Command Utilization",
                    f"{arm_name} Actual Utilization",
                    f"{arm_name} Raw Utilization",
                ],
                widths=[2.5, 1.8, 1.4],
            ),
            static=True,
        )
        rr.log(
            f"arms/{arm_label}/fast_status/min_joint_limit_margin_rad",
            rr.SeriesLines(
                colors=[_LIMIT_COLOR, _ERROR_COLOR],
                names=[f"{arm_name} Min Joint Limit Margin", "Warning Threshold"],
                widths=[2.5, 1.4],
            ),
            static=True,
        )
        rr.log(
            f"arms/{arm_label}/fast_status/min_velocity_margin_rad_s",
            rr.SeriesLines(
                colors=[_LIMIT_COLOR, _ERROR_COLOR],
                names=[f"{arm_name} Min Velocity Margin", "Warning Threshold"],
                widths=[2.5, 1.4],
            ),
            static=True,
        )


def _setup_dashboard_styles() -> None:
    """写入小屏 Status 页使用的左右臂合并曲线样式。"""
    rr.log(
        "dashboard/position_error_norm_mm",
        rr.SeriesLines(
            colors=[_LEFT_ARM_COLOR, _RIGHT_ARM_COLOR],
            names=["Left Position Error Norm", "Right Position Error Norm"],
            widths=[2.5, 2.5],
        ),
        static=True,
    )
    rr.log(
        "dashboard/reference_error_norm_mm",
        rr.SeriesLines(
            colors=[_LEFT_ARM_COLOR, _RIGHT_ARM_COLOR],
            names=["Left Reference Error Norm", "Right Reference Error Norm"],
            widths=[2.5, 2.5],
        ),
        static=True,
    )
    rr.log(
        "dashboard/rotation_error_norm_deg",
        rr.SeriesLines(
            colors=[_LEFT_ARM_COLOR, _RIGHT_ARM_COLOR],
            names=["Left Rotation Error Norm", "Right Rotation Error Norm"],
            widths=[2.5, 2.5],
        ),
        static=True,
    )
    rr.log(
        "dashboard/tcp_speed_mps",
        rr.SeriesLines(
            colors=[_LEFT_ARM_COLOR, _RIGHT_ARM_COLOR, _LIMIT_COLOR],
            names=["Left TCP Speed", "Right TCP Speed", "TCP Speed Limit"],
            widths=[2.5, 2.5, 1.5],
        ),
        static=True,
    )
    rr.log(
        "dashboard/torque_utilization",
        rr.SeriesLines(
            colors=[_LEFT_ARM_COLOR, _RIGHT_ARM_COLOR],
            names=["Left Command Utilization", "Right Command Utilization"],
            widths=[2.5, 2.5],
        ),
        static=True,
    )
    rr.log(
        "dashboard/min_joint_limit_margin_rad",
        rr.SeriesLines(
            colors=[_LEFT_ARM_COLOR, _RIGHT_ARM_COLOR, _ERROR_COLOR],
            names=["Left Min Limit Margin", "Right Min Limit Margin", "Warning Threshold"],
            widths=[2.5, 2.5, 1.5],
        ),
        static=True,
    )
    rr.log(
        "dashboard/min_velocity_margin_rad_s",
        rr.SeriesLines(
            colors=[_LEFT_ARM_COLOR, _RIGHT_ARM_COLOR, _ERROR_COLOR],
            names=["Left Min Velocity Margin", "Right Min Velocity Margin", "Warning Threshold"],
            widths=[2.5, 2.5, 1.5],
        ),
        static=True,
    )


def setup_realtime_styles():
    """设置交互式 Rerun 的曲线样式和试图蓝图，在仿真启动前调用"""
    if not RERUN_AVAILABLE: return
    _setup_trajectory_styles()
    _setup_arm_realtime_styles()
    _setup_fast_status_styles()
    _setup_dashboard_styles()

    rr.set_time_seconds("time", 0.0)
    for arm_label in _ARM_LABELS:
        for axis in ("X", "Y", "Z"):
            rr.log(f"arms/{arm_label}/position/{axis}/actual", rr.Scalars(0.0))
            rr.log(f"arms/{arm_label}/position/{axis}/target", rr.Scalars(0.0))
            rr.log(f"arms/{arm_label}/position/{axis}/reference", rr.Scalars(0.0))
            rr.log(f"arms/{arm_label}/position_error/{axis}/value", rr.Scalars(0.0))
            rr.log(f"arms/{arm_label}/tcp_velocity/{axis}/value", rr.Scalars(0.0))
        rr.log(f"arms/{arm_label}/tcp_speed/linear/value", rr.Scalars(0.0))
        rr.log(f"arms/{arm_label}/fast_status/position_error_norm_mm/value", rr.Scalars(0.0))
        rr.log(f"arms/{arm_label}/fast_status/reference_error_norm_mm/value", rr.Scalars(0.0))
        rr.log(f"arms/{arm_label}/fast_status/rotation_error_norm_deg/value", rr.Scalars(0.0))
        rr.log(f"arms/{arm_label}/fast_status/tcp_speed_mps/value", rr.Scalars(0.0))
        rr.log(
            f"arms/{arm_label}/fast_status/tcp_speed_mps/limit",
            rr.Scalars(float(Config.END_EFFECTOR_REAL_SPEED_LIMIT_MPS)),
        )
        rr.log(f"arms/{arm_label}/fast_status/torque_utilization/command", rr.Scalars(0.0))
        rr.log(f"arms/{arm_label}/fast_status/torque_utilization/actual", rr.Scalars(0.0))
        rr.log(f"arms/{arm_label}/fast_status/torque_utilization/raw", rr.Scalars(0.0))
        rr.log(f"arms/{arm_label}/fast_status/min_joint_limit_margin_rad/value", rr.Scalars(0.0))
        rr.log(
            f"arms/{arm_label}/fast_status/min_joint_limit_margin_rad/warning_threshold",
            rr.Scalars(float(_SAFETY_LOG_MARGIN_RAD)),
        )
        rr.log(f"arms/{arm_label}/fast_status/min_velocity_margin_rad_s/value", rr.Scalars(0.0))
        rr.log(
            f"arms/{arm_label}/fast_status/min_velocity_margin_rad_s/warning_threshold",
            rr.Scalars(float(_SAFETY_LOG_MARGIN_RAD_S)),
        )
        for axis in ("Roll", "Pitch", "Yaw"):
            rr.log(f"arms/{arm_label}/rotation/{axis}/actual", rr.Scalars(0.0))
            rr.log(f"arms/{arm_label}/rotation/{axis}/target", rr.Scalars(0.0))
            rr.log(f"arms/{arm_label}/rotation/{axis}/reference", rr.Scalars(0.0))
            rr.log(f"arms/{arm_label}/rotation_error/{axis}/value", rr.Scalars(0.0))
        for i in range(Config.ARM_JOINTS):
            joint = f"J{i + 1}"
            rr.log(f"arms/{arm_label}/joint_q/{joint}/value", rr.Scalars(0.0))
            rr.log(f"arms/{arm_label}/joint_qd/{joint}/value", rr.Scalars(0.0))
            rr.log(f"arms/{arm_label}/torque/{joint}/command", rr.Scalars(0.0))
            rr.log(f"arms/{arm_label}/torque_raw/{joint}/value", rr.Scalars(0.0))
            rr.log(f"arms/{arm_label}/torque_actual/{joint}/value", rr.Scalars(0.0))
            rr.log(f"arms/{arm_label}/torque_gap/{joint}/value", rr.Scalars(0.0))
            rr.log(f"arms/{arm_label}/velocity_margin/{joint}/value", rr.Scalars(0.0))
            rr.log(f"arms/{arm_label}/limit_margin_low/{joint}/value", rr.Scalars(0.0))
            rr.log(f"arms/{arm_label}/limit_margin_high/{joint}/value", rr.Scalars(0.0))
    for suffix in ("left", "right"):
        rr.log(f"dashboard/position_error_norm_mm/{suffix}", rr.Scalars(0.0))
        rr.log(f"dashboard/reference_error_norm_mm/{suffix}", rr.Scalars(0.0))
        rr.log(f"dashboard/rotation_error_norm_deg/{suffix}", rr.Scalars(0.0))
        rr.log(f"dashboard/tcp_speed_mps/{suffix}", rr.Scalars(0.0))
        rr.log(f"dashboard/torque_utilization/{suffix}", rr.Scalars(0.0))
        rr.log(f"dashboard/min_joint_limit_margin_rad/{suffix}", rr.Scalars(0.0))
        rr.log(f"dashboard/min_velocity_margin_rad_s/{suffix}", rr.Scalars(0.0))
    rr.log("dashboard/tcp_speed_mps/limit", rr.Scalars(float(Config.END_EFFECTOR_REAL_SPEED_LIMIT_MPS)))
    rr.log("dashboard/min_joint_limit_margin_rad/warning_threshold", rr.Scalars(float(_SAFETY_LOG_MARGIN_RAD)))
    rr.log("dashboard/min_velocity_margin_rad_s/warning_threshold", rr.Scalars(float(_SAFETY_LOG_MARGIN_RAD_S)))
    for path in sorted(_ESSENTIAL_PERFORMANCE_PATHS | _DETAILED_PERFORMANCE_PATHS):
        if _rerun_should_log_perf(path):
            rr.log(path, rr.Scalars(0.0))
    rr.log(
        "limits/tcp_speed",
        rr.SeriesLines(colors=[[120, 120, 120]], names=["TCP Speed Limit"], widths=[1.5]),
        static=True,
    )
    rr.log("limits/tcp_speed/value", rr.Scalars(float(Config.END_EFFECTOR_REAL_SPEED_LIMIT_MPS)))

    def link_latency_view():
        return rrb.TimeSeriesView(
            name="Control Link Period (ms)", origin="/performance/link_latency",
        )

    def performance_view(name: str, path: str):
        if not _rerun_should_log_perf(path):
            return None
        return rrb.TimeSeriesView(name=name, origin=f"/{path}")

    performance_overview_views = [
        performance_view("Python Control Step Time (ms)", "performance/c_engine_time"),
        performance_view("Control Link Rate (Hz)", "performance/link_cycle_hz"),
        performance_view("Control Link Period (ms)", "performance/link_latency"),
        performance_view("Sim Target Hz", "performance/sim_target_hz"),
        performance_view("Sim UDP Service Time (ms)", "performance/sim_service_ms"),
        performance_view("Sim MuJoCo Step Time (ms)", "performance/sim_mujoco_step_ms"),
        performance_view("Sim State Packet Time (ms)", "performance/sim_state_packet_ms"),
        performance_view("Sim Rerun Overwrites", "performance/sim_rerun_overwrite_count"),
        performance_view("Sim Rerun Drops", "performance/sim_rerun_drop_count"),
        performance_view("Viewer Sync Time (ms)", "performance/viewer_sync_ms"),
        performance_view("TX Pending Overwrites", "performance/tx_overwrite_count"),
        performance_view("CAN Backpressure Count", "performance/can_backpressure_count"),
    ]
    performance_detailed_views = [
        performance_view("Control Link Throughput (kbps)", "performance/link_transfer_kbps"),
        performance_view("STM32 Calculation Time (ms)", "performance/stm32_calc_time"),
        performance_view("STM32 Calculation Rate (Hz)", "performance/stm32_calc_hz"),
        performance_view("Feedback Wait (ms)", "performance/feedback_wait_ms"),
        performance_view("Control Target Hz", "performance/control_target_hz"),
        performance_view("Sim Socket Timeouts", "performance/sim_socket_timeout_count"),
        performance_view("Viewer Sync Count", "performance/viewer_sync_count"),
        performance_view("Viewer Skip Count", "performance/viewer_skip_count"),
        performance_view("Viewer Lock Wait (ms)", "performance/viewer_lock_wait_ms"),
    ]

    def control_link_view():
        return rrb.Vertical(
            rrb.TextLogView(name="Control Link Log", origin="/control_link_log"),
            link_latency_view(),
            name="Link",
        )

    def arm_axis_view(
        arm_label: str,
        arm_name: str,
        key: str,
        title: str,
        axes: tuple[str, ...],
        unit: str,
    ):
        return rrb.Vertical(
            *[
                rrb.TimeSeriesView(
                    name=f"{arm_name} {title} {axis} ({unit})",
                    origin=f"/arms/{arm_label}/{key}/{axis}",
                )
                for axis in axes
            ],
            name=f"{arm_name} {title}",
        )

    def arm_joint_axis_view(arm_label: str, arm_name: str, key: str, title: str, unit: str):
        return arm_axis_view(
            arm_label,
            arm_name,
            key,
            title,
            tuple(f"J{i + 1}" for i in range(Config.ARM_JOINTS)),
            unit,
        )

    def control_health_view():
        views = [
            performance_view("Python Control Step Time (ms)", "performance/c_engine_time"),
            performance_view("Control Link Period (ms)", "performance/link_latency"),
            performance_view("Control Link Rate (Hz)", "performance/link_cycle_hz"),
            performance_view("TX Pending Overwrites", "performance/tx_overwrite_count"),
            performance_view("CAN Backpressure Count", "performance/can_backpressure_count"),
        ]
        return rrb.Vertical(
            *[view for view in views if view is not None],
            rrb.TextLogView(name="Control Link Log", origin="/control_link_log"),
            name="Link",
        )

    def fast_status_detail_view(arm_label: str, arm_name: str):
        return rrb.Vertical(
            rrb.TimeSeriesView(
                name=f"{arm_name} Position Error Norm (mm)",
                origin=f"/arms/{arm_label}/fast_status/position_error_norm_mm",
            ),
            rrb.TimeSeriesView(
                name=f"{arm_name} Reference Error Norm (mm)",
                origin=f"/arms/{arm_label}/fast_status/reference_error_norm_mm",
            ),
            rrb.TimeSeriesView(
                name=f"{arm_name} Rotation Error Norm (deg)",
                origin=f"/arms/{arm_label}/fast_status/rotation_error_norm_deg",
            ),
            rrb.TimeSeriesView(
                name=f"{arm_name} TCP Speed (m/s)",
                origin=f"/arms/{arm_label}/fast_status/tcp_speed_mps",
            ),
            rrb.TimeSeriesView(
                name=f"{arm_name} Torque Utilization",
                origin=f"/arms/{arm_label}/fast_status/torque_utilization",
            ),
            rrb.TimeSeriesView(
                name=f"{arm_name} Min Joint Limit Margin (rad)",
                origin=f"/arms/{arm_label}/fast_status/min_joint_limit_margin_rad",
            ),
            rrb.TimeSeriesView(
                name=f"{arm_name} Min Velocity Margin (rad/s)",
                origin=f"/arms/{arm_label}/fast_status/min_velocity_margin_rad_s",
            ),
            name=f"{arm_name} Fast Status",
        )

    def tcp_speed_view():
        return rrb.Vertical(
            rrb.TimeSeriesView(name="TCP Speed Limit (m/s)", origin="/limits/tcp_speed"),
            rrb.TimeSeriesView(name="Left TCP Linear Speed (m/s)", origin="/arms/left/tcp_speed/linear"),
            rrb.TimeSeriesView(name="Right TCP Linear Speed (m/s)", origin="/arms/right/tcp_speed/linear"),
            arm_axis_view("left", "Left", "tcp_velocity", "TCP Velocity", ("X", "Y", "Z"), "m/s"),
            arm_axis_view("right", "Right", "tcp_velocity", "TCP Velocity", ("X", "Y", "Z"), "m/s"),
            name="TCP Speed",
        )

    def status_view():
        return rrb.Tabs(
            rrb.Vertical(
                rrb.TimeSeriesView(name="Position Error Norm (mm)", origin="/dashboard/position_error_norm_mm"),
                rrb.TimeSeriesView(name="Reference Error Norm (mm)", origin="/dashboard/reference_error_norm_mm"),
                rrb.TimeSeriesView(name="Rotation Error Norm (deg)", origin="/dashboard/rotation_error_norm_deg"),
                rrb.TimeSeriesView(name="TCP Speed (m/s)", origin="/dashboard/tcp_speed_mps"),
                name="Tracking",
            ),
            rrb.Vertical(
                rrb.TimeSeriesView(name="Command Torque Utilization", origin="/dashboard/torque_utilization"),
                name="Torque",
            ),
            rrb.Vertical(
                rrb.TimeSeriesView(name="Min Joint Limit Margin (rad)", origin="/dashboard/min_joint_limit_margin_rad"),
                rrb.TimeSeriesView(name="Min Velocity Margin (rad/s)", origin="/dashboard/min_velocity_margin_rad_s"),
                name="Safety",
            ),
            control_health_view(),
            name="Status",
        )

    def fast_status_detail_tab():
        return rrb.Horizontal(
            fast_status_detail_view("left", "Left"),
            fast_status_detail_view("right", "Right"),
            name="Fast Status",
        )

    def tracking_detail_view():
        return rrb.Tabs(
            fast_status_detail_tab(),
            rrb.Horizontal(
                arm_axis_view("left", "Left", "position", "Position", ("X", "Y", "Z"), _POSITION_DISPLAY_UNIT),
                arm_axis_view("right", "Right", "position", "Position", ("X", "Y", "Z"), _POSITION_DISPLAY_UNIT),
                name="Position",
            ),
            rrb.Horizontal(
                arm_axis_view("left", "Left", "rotation", "Rotation", ("Roll", "Pitch", "Yaw"), "deg"),
                arm_axis_view("right", "Right", "rotation", "Rotation", ("Roll", "Pitch", "Yaw"), "deg"),
                name="Rotation",
            ),
            rrb.Horizontal(
                arm_axis_view("left", "Left", "position_error", "Position Error", ("X", "Y", "Z"), "mm"),
                arm_axis_view("right", "Right", "position_error", "Position Error", ("X", "Y", "Z"), "mm"),
                name="Position Error",
            ),
            rrb.Horizontal(
                arm_axis_view("left", "Left", "rotation_error", "Rotation Error", ("Roll", "Pitch", "Yaw"), "deg"),
                arm_axis_view("right", "Right", "rotation_error", "Rotation Error", ("Roll", "Pitch", "Yaw"), "deg"),
                name="Rotation Error",
            ),
            tcp_speed_view(),
            name="Tracking Detail",
        )

    def joint_detail_view():
        return rrb.Tabs(
            rrb.Horizontal(
                arm_joint_axis_view("left", "Left", "joint_q", "Joint Q", "rad"),
                arm_joint_axis_view("right", "Right", "joint_q", "Joint Q", "rad"),
                name="Joint Position",
            ),
            rrb.Horizontal(
                arm_joint_axis_view("left", "Left", "joint_qd", "Joint QD", "rad/s"),
                arm_joint_axis_view("right", "Right", "joint_qd", "Joint QD", "rad/s"),
                name="Joint Velocity",
            ),
            name="Joint Detail",
        )

    def torque_detail_view():
        return rrb.Tabs(
            rrb.Horizontal(
                arm_joint_axis_view("left", "Left", "torque", "Torque", "N*m"),
                arm_joint_axis_view("right", "Right", "torque", "Torque", "N*m"),
                name="Command",
            ),
            rrb.Horizontal(
                arm_joint_axis_view("left", "Left", "torque_raw", "Raw Torque", "N*m"),
                arm_joint_axis_view("right", "Right", "torque_raw", "Raw Torque", "N*m"),
                name="Raw",
            ),
            rrb.Horizontal(
                arm_joint_axis_view("left", "Left", "torque_actual", "Actual Torque", "N*m"),
                arm_joint_axis_view("right", "Right", "torque_actual", "Actual Torque", "N*m"),
                name="Actual",
            ),
            rrb.Horizontal(
                arm_joint_axis_view("left", "Left", "torque_gap", "Torque Gap", "N*m"),
                arm_joint_axis_view("right", "Right", "torque_gap", "Torque Gap", "N*m"),
                name="Gap",
            ),
            rrb.Horizontal(
                rrb.TimeSeriesView(
                    name="Left Torque Utilization",
                    origin="/arms/left/fast_status/torque_utilization",
                ),
                rrb.TimeSeriesView(
                    name="Right Torque Utilization",
                    origin="/arms/right/fast_status/torque_utilization",
                ),
                name="Utilization",
            ),
            name="Torque Detail",
        )

    def safety_view():
        return rrb.Tabs(
            rrb.Horizontal(
                arm_joint_axis_view("left", "Left", "velocity_margin", "Velocity Safety Margin", "rad/s"),
                arm_joint_axis_view("right", "Right", "velocity_margin", "Velocity Safety Margin", "rad/s"),
                name="Velocity Margin",
            ),
            rrb.Horizontal(
                arm_joint_axis_view("left", "Left", "limit_margin_low", "Low Limit Margin", "rad"),
                arm_joint_axis_view("right", "Right", "limit_margin_low", "Low Limit Margin", "rad"),
                name="Low Limit Margin",
            ),
            rrb.Horizontal(
                arm_joint_axis_view("left", "Left", "limit_margin_high", "High Limit Margin", "rad"),
                arm_joint_axis_view("right", "Right", "limit_margin_high", "High Limit Margin", "rad"),
                name="High Limit Margin",
            ),
            rrb.Vertical(
                rrb.Horizontal(
                    rrb.TimeSeriesView(
                        name="Left Min Joint Limit Margin (rad)",
                        origin="/arms/left/fast_status/min_joint_limit_margin_rad",
                    ),
                    rrb.TimeSeriesView(
                        name="Right Min Joint Limit Margin (rad)",
                        origin="/arms/right/fast_status/min_joint_limit_margin_rad",
                    ),
                    name="Limit Margin Summary",
                ),
                rrb.Horizontal(
                    rrb.TimeSeriesView(
                        name="Left Min Velocity Margin (rad/s)",
                        origin="/arms/left/fast_status/min_velocity_margin_rad_s",
                    ),
                    rrb.TimeSeriesView(
                        name="Right Min Velocity Margin (rad/s)",
                        origin="/arms/right/fast_status/min_velocity_margin_rad_s",
                    ),
                    name="Velocity Margin Summary",
                ),
                name="Summary",
            ),
            rrb.TextLogView(name="Warnings", origin="/control_link_log"),
            name="Safety",
        )

    def details_view():
        return rrb.Tabs(
            tracking_detail_view(),
            joint_detail_view(),
            torque_detail_view(),
            safety_view(),
            name="Details",
        )

    def performance_view_group():
        overview_views = [view for view in performance_overview_views if view is not None]
        detailed_views = [view for view in performance_detailed_views if view is not None]
        children = [
            rrb.Vertical(*overview_views, name="Overview"),
        ]
        if detailed_views:
            children.append(rrb.Vertical(*detailed_views, name="Detailed"))
        return rrb.Tabs(*children, name="Performance")

    blueprint = rrb.Blueprint(
        rrb.Tabs(
            status_view(),
            rrb.Spatial3DView(name="3D", origin="/trajectory_3d"),
            details_view(),
            performance_view_group(),
            control_link_view(),
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
    pos_reference: np.ndarray = None,
    quat_reference: np.ndarray = None,
    q: np.ndarray = None,
    qd: np.ndarray = None,
    ee_twist: np.ndarray = None,
    tau_raw: np.ndarray = None,
    tau_actual: np.ndarray = None,
    elapsed_s: float = None,
    right_j7_diag: dict = None,
    rx_str: str = None,
    tx_str: str = None,
    tx_label: str = "Torques",
    step_count: int = 0,
    uart_latency_ms: float = None,
    uart_cycle_hz: float = None,
    uart_transfer_kbps: float = None,
    stm32_calc_time_ms: float = None,
    stm32_calc_hz: float = None,
    c_bridge_ms: float = None,
    feedback_wait_ms: float = None,
    tx_overwrite_count: int = None,
    can_backpressure_count: int = None,
    control_target_hz: float = None,
    sim_target_hz: float = None,
    sim_service_ms: float = None,
    sim_mujoco_step_ms: float = None,
    sim_state_packet_ms: float = None,
    sim_rerun_overwrite_count: int = None,
    sim_rerun_drop_count: int = None,
    sim_socket_timeout_count: int = None,
    viewer_sync_count: int = None,
    viewer_skip_count: int = None,
    viewer_sync_ms: float = None,
    viewer_lock_wait_ms: float = None,
):
    """单步记录交互式仿真数据"""
    if not RERUN_AVAILABLE: return
    if Config.RERUN_LOG_STRIDE > 1 and step_count % Config.RERUN_LOG_STRIDE != 0:
        return
    rr.set_time_seconds("time", t)
    pos_actual_by_arm = _as_arm_array(pos_actual, 3)
    pos_desired_by_arm = _as_arm_array(pos_desired, 3)
    quat_actual_by_arm = _as_arm_array(quat_actual, 4)
    quat_desired_by_arm = _as_arm_array(quat_desired, 4)
    pos_reference_by_arm = None if pos_reference is None else _as_arm_array(pos_reference, 3)
    quat_reference_by_arm = None if quat_reference is None else _as_arm_array(quat_reference, 4)
    reference_valid = None
    if pos_reference_by_arm is not None:
        reference_valid = np.all(np.isfinite(pos_reference_by_arm), axis=1)
        if quat_reference_by_arm is not None:
            reference_valid &= np.all(np.isfinite(quat_reference_by_arm), axis=1)
            reference_valid &= np.linalg.norm(quat_reference_by_arm, axis=1) > 1e-12

    for arm, arm_label in enumerate(_ARM_LABELS[: len(pos_actual_by_arm)]):
        pos_actual_display = _position_to_display_units(pos_actual_by_arm[arm])
        pos_desired_display = _position_to_display_units(pos_desired_by_arm[arm])
        has_reference = (
            pos_reference_by_arm is not None
            and arm < len(pos_reference_by_arm)
            and bool(reference_valid[arm])
        )
        if has_reference:
            pos_reference_display = _position_to_display_units(pos_reference_by_arm[arm])
        for i, axis in enumerate(('X', 'Y', 'Z')):
            rr.log(f"arms/{arm_label}/position/{axis}/actual", rr.Scalars(float(pos_actual_display[i])))
            rr.log(f"arms/{arm_label}/position/{axis}/target", rr.Scalars(float(pos_desired_display[i])))
            if has_reference:
                rr.log(f"arms/{arm_label}/position/{axis}/reference", rr.Scalars(float(pos_reference_display[i])))
            rr.log(f"arms/{arm_label}/position_error/{axis}/value", rr.Scalars(float(pos_actual_display[i] - pos_desired_display[i])))
        position_error_norm_mm = float(np.linalg.norm(pos_actual_display - pos_desired_display))
        rr.log(
            f"arms/{arm_label}/fast_status/position_error_norm_mm/value",
            rr.Scalars(position_error_norm_mm),
        )
        rr.log(f"dashboard/position_error_norm_mm/{arm_label}", rr.Scalars(position_error_norm_mm))
        if has_reference:
            reference_error_norm_mm = float(np.linalg.norm(pos_actual_display - pos_reference_display))
            rr.log(
                f"arms/{arm_label}/fast_status/reference_error_norm_mm/value",
                rr.Scalars(reference_error_norm_mm),
            )
            rr.log(f"dashboard/reference_error_norm_mm/{arm_label}", rr.Scalars(reference_error_norm_mm))

        rot_actual = quat_to_euler(quat_actual_by_arm[arm])
        rot_desired = quat_to_euler(quat_desired_by_arm[arm])
        rot_err = compute_rotation_error_single(quat_actual_by_arm[arm], quat_desired_by_arm[arm])
        rot_actual_deg = np.rad2deg(rot_actual)
        rot_desired_deg = np.rad2deg(rot_desired)
        if has_reference and quat_reference_by_arm is not None:
            rot_reference_deg = np.rad2deg(quat_to_euler(quat_reference_by_arm[arm]))

        for i, axis in enumerate(('Roll', 'Pitch', 'Yaw')):
            rr.log(f"arms/{arm_label}/rotation/{axis}/actual", rr.Scalars(float(rot_actual_deg[i])))
            rr.log(f"arms/{arm_label}/rotation/{axis}/target", rr.Scalars(float(rot_desired_deg[i])))
            if has_reference and quat_reference_by_arm is not None:
                rr.log(f"arms/{arm_label}/rotation/{axis}/reference", rr.Scalars(float(rot_reference_deg[i])))
            rr.log(f"arms/{arm_label}/rotation_error/{axis}/value", rr.Scalars(float(rot_err[i])))
        rotation_error_norm_deg = float(np.linalg.norm(rot_err))
        rr.log(
            f"arms/{arm_label}/fast_status/rotation_error_norm_deg/value",
            rr.Scalars(rotation_error_norm_deg),
        )
        rr.log(f"dashboard/rotation_error_norm_deg/{arm_label}", rr.Scalars(rotation_error_norm_deg))

    if q is not None:
        q_by_arm = _as_arm_array(q, Config.ARM_JOINTS)
        for arm, arm_label in enumerate(_ARM_LABELS[: len(q_by_arm)]):
            for i, value in enumerate(q_by_arm[arm]):
                rr.log(f"arms/{arm_label}/joint_q/J{i+1}/value", rr.Scalars(float(value)))
    if qd is not None:
        qd_by_arm = _as_arm_array(qd, Config.ARM_JOINTS)
        for arm, arm_label in enumerate(_ARM_LABELS[: len(qd_by_arm)]):
            for i, value in enumerate(qd_by_arm[arm]):
                rr.log(f"arms/{arm_label}/joint_qd/J{i+1}/value", rr.Scalars(float(value)))

    if ee_twist is not None:
        ee_twist_by_arm = _as_arm_array(ee_twist, 6)
        for arm, arm_label in enumerate(_ARM_LABELS[: len(ee_twist_by_arm)]):
            linear_velocity = ee_twist_by_arm[arm, :3]
            linear_speed = float(np.linalg.norm(linear_velocity))
            for i, axis in enumerate(("X", "Y", "Z")):
                rr.log(
                    f"arms/{arm_label}/tcp_velocity/{axis}/value",
                    rr.Scalars(float(linear_velocity[i])),
                )
            rr.log(f"arms/{arm_label}/tcp_speed/linear/value", rr.Scalars(linear_speed))
            rr.log(f"arms/{arm_label}/fast_status/tcp_speed_mps/value", rr.Scalars(linear_speed))
            rr.log(f"dashboard/tcp_speed_mps/{arm_label}", rr.Scalars(linear_speed))
            rr.log(
                f"arms/{arm_label}/fast_status/tcp_speed_mps/limit",
                rr.Scalars(float(Config.END_EFFECTOR_REAL_SPEED_LIMIT_MPS)),
            )
        rr.log("limits/tcp_speed/value", rr.Scalars(float(Config.END_EFFECTOR_REAL_SPEED_LIMIT_MPS)))
        rr.log("dashboard/tcp_speed_mps/limit", rr.Scalars(float(Config.END_EFFECTOR_REAL_SPEED_LIMIT_MPS)))
        
    tau_by_arm = _as_arm_array(tau_total, Config.ARM_JOINTS)
    tau_raw_by_arm = None if tau_raw is None else _as_arm_array(tau_raw, Config.ARM_JOINTS)
    tau_actual_by_arm = None if tau_actual is None else _as_arm_array(tau_actual, Config.ARM_JOINTS)
    torque_limits_by_arm = _as_arm_array(Config.TORQUE_LIMITS, Config.ARM_JOINTS)
    for arm, arm_label in enumerate(_ARM_LABELS[: len(tau_by_arm)]):
        for i, value in enumerate(tau_by_arm[arm]):
            rr.log(f"arms/{arm_label}/torque/J{i+1}/command", rr.Scalars(float(value)))
            if tau_raw_by_arm is not None:
                rr.log(f"arms/{arm_label}/torque_raw/J{i+1}/value", rr.Scalars(float(tau_raw_by_arm[arm, i])))
            if tau_actual_by_arm is not None:
                rr.log(f"arms/{arm_label}/torque_actual/J{i+1}/value", rr.Scalars(float(tau_actual_by_arm[arm, i])))
                rr.log(
                    f"arms/{arm_label}/torque_gap/J{i+1}/value",
                    rr.Scalars(float(value - tau_actual_by_arm[arm, i])),
                )
        command_torque_utilization = _torque_utilization(tau_by_arm[arm], torque_limits_by_arm[arm])
        rr.log(
            f"arms/{arm_label}/fast_status/torque_utilization/command",
            rr.Scalars(command_torque_utilization),
        )
        rr.log(f"dashboard/torque_utilization/{arm_label}", rr.Scalars(command_torque_utilization))
        if tau_actual_by_arm is not None:
            rr.log(
                f"arms/{arm_label}/fast_status/torque_utilization/actual",
                rr.Scalars(_torque_utilization(tau_actual_by_arm[arm], torque_limits_by_arm[arm])),
            )
        if tau_raw_by_arm is not None:
            rr.log(
                f"arms/{arm_label}/fast_status/torque_utilization/raw",
                rr.Scalars(_torque_utilization(tau_raw_by_arm[arm], torque_limits_by_arm[arm])),
            )

    q_margin_low, q_margin_high, vel_margin = _joint_safety_margins(q, qd)
    if vel_margin is not None:
        vel_margin_by_arm = _as_arm_array(vel_margin, Config.ARM_JOINTS)
        for arm, arm_label in enumerate(_ARM_LABELS[: len(vel_margin_by_arm)]):
            for i, value in enumerate(vel_margin_by_arm[arm]):
                rr.log(f"arms/{arm_label}/velocity_margin/J{i+1}/value", rr.Scalars(float(value)))
            min_velocity_margin = float(np.min(vel_margin_by_arm[arm]))
            rr.log(
                f"arms/{arm_label}/fast_status/min_velocity_margin_rad_s/value",
                rr.Scalars(min_velocity_margin),
            )
            rr.log(f"dashboard/min_velocity_margin_rad_s/{arm_label}", rr.Scalars(min_velocity_margin))
            rr.log(
                f"arms/{arm_label}/fast_status/min_velocity_margin_rad_s/warning_threshold",
                rr.Scalars(float(_SAFETY_LOG_MARGIN_RAD_S)),
            )
            rr.log(
                "dashboard/min_velocity_margin_rad_s/warning_threshold",
                rr.Scalars(float(_SAFETY_LOG_MARGIN_RAD_S)),
            )
    if q_margin_low is not None and q_margin_high is not None:
        low_by_arm = _as_arm_array(q_margin_low, Config.ARM_JOINTS)
        high_by_arm = _as_arm_array(q_margin_high, Config.ARM_JOINTS)
        for arm, arm_label in enumerate(_ARM_LABELS[: len(low_by_arm)]):
            for i, value in enumerate(low_by_arm[arm]):
                rr.log(f"arms/{arm_label}/limit_margin_low/J{i+1}/value", rr.Scalars(float(value)))
                rr.log(f"arms/{arm_label}/limit_margin_high/J{i+1}/value", rr.Scalars(float(high_by_arm[arm, i])))
            min_limit_margin = np.minimum(low_by_arm[arm], high_by_arm[arm])
            min_joint_limit_margin = float(np.min(min_limit_margin))
            rr.log(
                f"arms/{arm_label}/fast_status/min_joint_limit_margin_rad/value",
                rr.Scalars(min_joint_limit_margin),
            )
            rr.log(f"dashboard/min_joint_limit_margin_rad/{arm_label}", rr.Scalars(min_joint_limit_margin))
            rr.log(
                f"arms/{arm_label}/fast_status/min_joint_limit_margin_rad/warning_threshold",
                rr.Scalars(float(_SAFETY_LOG_MARGIN_RAD)),
            )
            rr.log(
                "dashboard/min_joint_limit_margin_rad/warning_threshold",
                rr.Scalars(float(_SAFETY_LOG_MARGIN_RAD)),
            )

    safety_warning = _format_safety_warnings(q_margin_low, q_margin_high, vel_margin)
    if safety_warning and _should_log_safety_warning(safety_warning, step_count):
        text = f"[{step_count}] SAFETY margin warning: {safety_warning}"
        rr.log("control_link_log", rr.TextLog(text))
        print(f"[Rerun Safety] {text}")
    elif not safety_warning:
        _reset_safety_warning_throttle()
            
    # Text Log for Sent/Received Data (Throttled to 10Hz to prevent lag at 1kHz loop)
    if rx_str and tx_str:
        rr.log("control_link_log", rr.TextLog(f"[{step_count}] RX (Positions): {rx_str}\n[{step_count}] TX ({tx_label}): {tx_str}"))

    # Performance
    _log_perf_scalar("performance/c_engine_time", cycle_time)
    _log_perf_scalar("performance/elapsed_s", elapsed_s)
    _log_perf_scalar("performance/link_latency", uart_latency_ms)
    _log_perf_scalar("performance/link_cycle_hz", uart_cycle_hz)
    _log_perf_scalar("performance/link_transfer_kbps", uart_transfer_kbps)
    _log_perf_scalar("performance/stm32_calc_time", stm32_calc_time_ms)
    _log_perf_scalar("performance/stm32_calc_hz", stm32_calc_hz)
    _log_perf_scalar("performance/stm32_calc_time", c_bridge_ms)
    _log_perf_scalar("performance/feedback_wait_ms", feedback_wait_ms)
    _log_perf_scalar("performance/tx_overwrite_count", tx_overwrite_count)
    _log_perf_scalar("performance/can_backpressure_count", can_backpressure_count)
    _log_perf_scalar("performance/control_target_hz", control_target_hz)
    _log_perf_scalar("performance/sim_target_hz", sim_target_hz)
    _log_perf_scalar("performance/sim_service_ms", sim_service_ms)
    _log_perf_scalar("performance/sim_mujoco_step_ms", sim_mujoco_step_ms)
    _log_perf_scalar("performance/sim_state_packet_ms", sim_state_packet_ms)
    _log_perf_scalar("performance/sim_rerun_overwrite_count", sim_rerun_overwrite_count)
    _log_perf_scalar("performance/sim_rerun_drop_count", sim_rerun_drop_count)
    _log_perf_scalar("performance/sim_socket_timeout_count", sim_socket_timeout_count)
    _log_perf_scalar("performance/viewer_sync_count", viewer_sync_count)
    _log_perf_scalar("performance/viewer_skip_count", viewer_skip_count)
    _log_perf_scalar("performance/viewer_sync_ms", viewer_sync_ms)
    _log_perf_scalar("performance/viewer_lock_wait_ms", viewer_lock_wait_ms)

    if right_j7_diag:
        for key in ("q", "qd", "tau_cmd_raw", "tau_cmd_sent", "tau_actual"):
            if key in right_j7_diag:
                rr.log(f"diagnostics/right_j7/{key}", rr.Scalars(float(right_j7_diag[key])))
    
    # 3D
    actual_colors = [[230, 100, 50], [50, 150, 230]][: len(pos_actual_by_arm)]
    target_colors = [[80, 220, 80], [230, 220, 80]][: len(pos_desired_by_arm)]
    labels_actual = [f"{name} Actual" for name in _ARM_DISPLAY_NAMES[: len(pos_actual_by_arm)]]
    labels_target = [f"{name} Target" for name in _ARM_DISPLAY_NAMES[: len(pos_desired_by_arm)]]
    rr.log("trajectory_3d/actual_ee",
           rr.Points3D(pos_actual_by_arm, colors=actual_colors, radii=0.015, labels=labels_actual))
    rr.log("trajectory_3d/target_goal",
           rr.Points3D(pos_desired_by_arm, colors=target_colors, radii=0.015, labels=labels_target))
    if pos_reference_by_arm is not None and reference_valid is not None:
        reference_indices = [
            arm
            for arm in range(min(len(pos_actual_by_arm), len(pos_reference_by_arm)))
            if bool(reference_valid[arm])
        ]
        if reference_indices:
            reference_points = pos_reference_by_arm[reference_indices]
            labels_reference = [
                f"{_ARM_DISPLAY_NAMES[arm]} Reference" for arm in reference_indices
            ]
            rr.log(
                "trajectory_3d/reference_point",
                rr.Points3D(
                    reference_points,
                    colors=[_REFERENCE_COLOR] * len(reference_indices),
                    radii=0.012,
                    labels=labels_reference,
                ),
            )

    error_lines = [
        [pos_actual_by_arm[arm], pos_desired_by_arm[arm]]
        for arm in range(len(pos_actual_by_arm))
    ]
    rr.log("trajectory_3d/error_line",
           rr.LineStrips3D(error_lines, colors=[[255, 0, 0]], radii=0.002))
    if pos_reference_by_arm is not None and reference_valid is not None:
        reference_error_lines = [
            [pos_actual_by_arm[arm], pos_reference_by_arm[arm]]
            for arm in range(min(len(pos_actual_by_arm), len(pos_reference_by_arm)))
            if bool(reference_valid[arm])
        ]
        if reference_error_lines:
            rr.log(
                "trajectory_3d/reference_error_line",
                rr.LineStrips3D(reference_error_lines, colors=[_REFERENCE_COLOR], radii=0.002),
            )



