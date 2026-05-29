"""Compatibility wrapper for simulation UDP helpers."""

from sim import udp_server as _udp_server

globals().update(
    {
        name: getattr(_udp_server, name)
        for name in dir(_udp_server)
        if not name.startswith("__")
    }
)
