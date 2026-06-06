"""Performance path gating for Rerun logging."""

from config import Config

ESSENTIAL_PERFORMANCE_PATHS = {
    "performance/c_engine_time",
    "performance/elapsed_s",
    "performance/link_latency",
    "performance/link_cycle_hz",
    "performance/sim_target_hz",
    "performance/sim_service_ms",
    "performance/sim_mujoco_step_ms",
    "performance/sim_state_packet_ms",
    "performance/sim_rerun_overwrite_count",
    "performance/sim_rerun_drop_count",
    "performance/viewer_sync_ms",
    "performance/tx_overwrite_count",
    "performance/can_backpressure_count",
}
DETAILED_PERFORMANCE_PATHS = {
    "performance/link_transfer_kbps",
    "performance/stm32_calc_time",
    "performance/stm32_calc_hz",
    "performance/feedback_wait_ms",
    "performance/control_target_hz",
    "performance/sim_socket_timeout_count",
    "performance/viewer_sync_count",
    "performance/viewer_skip_count",
    "performance/viewer_lock_wait_ms",
}


def _rerun_detailed_perf_enabled() -> bool:
    return bool(getattr(Config, "RERUN_DETAILED_PERF", False))


def _rerun_should_log_perf(path: str) -> bool:
    return path in ESSENTIAL_PERFORMANCE_PATHS or (
        _rerun_detailed_perf_enabled() and path in DETAILED_PERFORMANCE_PATHS
    )
