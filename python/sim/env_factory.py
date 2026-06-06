"""Factory for the MuJoCo simulation environment used by sim entrypoints."""

from __future__ import annotations


def _create_sim_env():
    try:
        from sim_env import MujocoSimEnv
    except ModuleNotFoundError as exc:
        if exc.name == "mujoco":
            raise RuntimeError(
                "缺少 Python 依赖 mujoco，请先执行 "
                "`python3 -m pip install -r python/requirements.txt`。"
            ) from exc
        raise

    return MujocoSimEnv()
