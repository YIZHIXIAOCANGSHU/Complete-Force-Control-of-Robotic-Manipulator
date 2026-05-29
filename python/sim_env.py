"""Compatibility wrapper for the MuJoCo simulation environment."""

from sim import env as _env

globals().update(
    {name: getattr(_env, name) for name in dir(_env) if not name.startswith("__")}
)
