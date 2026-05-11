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


def test_real_serial_backend_keeps_existing_entrypoint():
    completed = _run_script("real", "serial")

    assert completed.returncode == 0, completed.stderr
    assert _fake_python_args(completed) == ["python/real/serial_control.py"]


def test_real_usbfdcan_backend_routes_to_can_entrypoint():
    completed = _run_script("real", "usbfdcan")

    assert completed.returncode == 0, completed.stderr
    assert _fake_python_args(completed) == ["python/real/usb2fdcan_control.py"]


def test_interactive_real_menu_can_select_usbfdcan_backend():
    completed = _run_script(input_text="2\n2\n")

    assert completed.returncode == 0, completed.stderr
    assert "Real 后端" in completed.stdout
    assert _fake_python_args(completed) == ["python/real/usb2fdcan_control.py"]


def test_usbfdcan_sim_mode_routes_to_feedback_mirror_entrypoint():
    completed = _run_script("usbfdcan-sim")

    assert completed.returncode == 0, completed.stderr
    assert _fake_python_args(completed) == ["python/usbfdcan_sim/main.py"]


def test_usb2fdcan_sim_alias_routes_to_feedback_mirror_entrypoint():
    completed = _run_script("usb2fdcan-sim")

    assert completed.returncode == 0, completed.stderr
    assert _fake_python_args(completed) == ["python/usbfdcan_sim/main.py"]


def test_interactive_menu_can_select_usbfdcan_sim_mode():
    completed = _run_script(input_text="4\n")

    assert completed.returncode == 0, completed.stderr
    assert "usbfdcan-sim" in completed.stdout
    assert _fake_python_args(completed) == ["python/usbfdcan_sim/main.py"]
