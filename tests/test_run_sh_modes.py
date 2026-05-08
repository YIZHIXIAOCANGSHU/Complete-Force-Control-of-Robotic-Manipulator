from __future__ import annotations

import os
import subprocess
import tempfile
from pathlib import Path


PROJECT_ROOT = Path(__file__).resolve().parents[1]
RUN_SH_PATH = PROJECT_ROOT / "run.sh"


def _make_env_with_fake_tools() -> tuple[dict[str, str], tempfile.TemporaryDirectory[str]]:
    tempdir = tempfile.TemporaryDirectory()
    fake_dir = Path(tempdir.name)
    fake_python = fake_dir / "python3"
    fake_python.write_text(
        "#!/bin/sh\n"
        "if [ \"$1\" = \"-\" ]; then exit 0; fi\n"
        "printf 'PYTHON_ARGS\\n'\n"
        "for arg in \"$@\"; do printf '%s\\n' \"$arg\"; done\n",
        encoding="utf-8",
    )
    fake_python.chmod(0o755)

    fake_make = fake_dir / "make"
    fake_make.write_text(
        "#!/bin/sh\n"
        "printf 'MAKE_ARGS\\n'\n"
        "for arg in \"$@\"; do printf '%s\\n' \"$arg\"; done\n",
        encoding="utf-8",
    )
    fake_make.chmod(0o755)

    env = os.environ.copy()
    env["PATH"] = f"{fake_dir}:{env.get('PATH', '')}"
    return env, tempdir


def _run_script(*args: str, input_text: str = "") -> subprocess.CompletedProcess[str]:
    env, tempdir = _make_env_with_fake_tools()
    try:
        return subprocess.run(
            [str(RUN_SH_PATH), *args],
            input=input_text,
            text=True,
            capture_output=True,
            cwd=PROJECT_ROOT,
            env=env,
            check=False,
        )
    finally:
        tempdir.cleanup()


def _fake_python_args(completed: subprocess.CompletedProcess[str]) -> list[str]:
    lines = completed.stdout.strip().splitlines()
    if "PYTHON_ARGS" not in lines:
        return []
    start = len(lines) - 1 - lines[::-1].index("PYTHON_ARGS")
    return lines[start + 1 :]


def test_mc_routes_to_monte_carlo_entrypoint():
    completed = _run_script("mc", "-n", "5", "--no-viewer")

    assert completed.returncode == 0, completed.stderr
    assert _fake_python_args(completed) == [
        "python/main_server.py",
        "--monte-carlo",
        "-n",
        "5",
        "--no-viewer",
    ]


def test_real_mode_is_removed():
    completed = _run_script("real")

    assert completed.returncode != 0
    assert "未知模式 'real'" in completed.stdout
    assert _fake_python_args(completed) == []


def test_interactive_menu_can_select_mc():
    completed = _run_script(input_text="2\n")

    assert completed.returncode == 0, completed.stderr
    assert "AM-DPBSURDF0422 Sim 启动状态机" in completed.stdout
    assert _fake_python_args(completed) == ["python/main_server.py", "--monte-carlo"]
