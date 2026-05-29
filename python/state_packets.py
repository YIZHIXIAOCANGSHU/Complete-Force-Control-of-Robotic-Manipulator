"""Compatibility wrapper for simulation packet helpers."""

from sim import state_packets as _state_packets

globals().update(
    {
        name: getattr(_state_packets, name)
        for name in dir(_state_packets)
        if not name.startswith("__")
    }
)
