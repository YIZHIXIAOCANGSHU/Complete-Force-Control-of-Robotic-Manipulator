"""Compatibility wrapper for MuJoCo scene construction helpers."""

from sim import scene as _scene

globals().update(
    {
        name: getattr(_scene, name)
        for name in dir(_scene)
        if not name.startswith("__")
    }
)
