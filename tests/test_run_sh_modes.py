from __future__ import annotations

import importlib.util
import os
import subprocess
import tempfile
from pathlib import Path
import sys


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
    env["AM_D02_VENV_DIR"] = str(fake_dir / ".venv")
    venv_bin = fake_dir / ".venv" / "bin"
    venv_bin.mkdir(parents=True)
    fake_venv_python = venv_bin / "python"
    fake_venv_python.write_text(fake_python.read_text(encoding="utf-8"), encoding="utf-8")
    fake_venv_python.chmod(0o755)
    return env, tempdir


def _run_script(
    *args: str,
    input_text: str = "",
    env_overrides: dict[str, str] | None = None,
) -> subprocess.CompletedProcess[str]:
    env, tempdir = _make_env_with_fake_tools()
    if env_overrides:
        env.update(env_overrides)
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
        "python/sim/main_server.py",
        "--monte-carlo",
        "-n",
        "5",
        "--no-viewer",
    ]


def test_run_sh_requires_project_venv_when_no_python_override():
    completed = _run_script("mc", env_overrides={"AM_D02_VENV_DIR": "/tmp/am_d02_missing_venv_for_test"})

    assert completed.returncode != 0
    assert "未找到当前项目虚拟环境" in completed.stdout
    assert "./scripts/setup_venv.sh" in completed.stdout


def test_run_sh_allows_explicit_python_override_without_project_venv():
    completed = _run_script(
        "mc",
        env_overrides={
            "AM_D02_VENV_DIR": "/tmp/am_d02_missing_venv_for_test",
            "AM_D02_PYTHON": "python3",
        },
    )

    assert completed.returncode == 0, completed.stderr
    assert _fake_python_args(completed) == ["python/sim/main_server.py", "--monte-carlo"]


def test_real_left_routes_to_real_entrypoint():
    completed = _run_script("real", "left")

    assert completed.returncode == 0, completed.stderr
    assert _fake_python_args(completed) == ["python/real/main.py", "--arm", "left"]


def test_real_right_routes_to_real_entrypoint():
    completed = _run_script("real", "right")

    assert completed.returncode == 0, completed.stderr
    assert _fake_python_args(completed) == ["python/real/main.py", "--arm", "right"]


def test_real_right_pinocchio_routes_to_pinocchio_entrypoint():
    completed = _run_script("real", "right", "pinocchio")

    assert completed.returncode == 0, completed.stderr
    assert _fake_python_args(completed) == ["python/real_pinocchio/main.py", "--arm", "right"]


def test_real_both_routes_to_real_entrypoint():
    completed = _run_script("real", "both")

    assert completed.returncode == 0, completed.stderr
    assert _fake_python_args(completed) == ["python/real/main.py", "--arm", "both"]


def test_real_split_alias_routes_to_real_entrypoint():
    completed = _run_script("real-right")

    assert completed.returncode == 0, completed.stderr
    assert _fake_python_args(completed) == ["python/real/main.py", "--arm", "right"]


def test_interactive_menu_can_select_real_right():
    completed = _run_script(input_text="2\n2\n1\n")

    assert completed.returncode == 0, completed.stderr
    assert "Real 真机模式" in completed.stdout
    assert _fake_python_args(completed) == ["python/real/main.py", "--arm", "right"]


def test_interactive_menu_can_select_real_right_pinocchio():
    completed = _run_script(input_text="2\n2\n2\n")

    assert completed.returncode == 0, completed.stderr
    assert "Real 控制方式" in completed.stdout
    assert _fake_python_args(completed) == ["python/real_pinocchio/main.py", "--arm", "right"]


def test_interactive_menu_can_select_mc():
    completed = _run_script(input_text="3\n")

    assert completed.returncode == 0, completed.stderr
    assert "AM-DPBSURDF0422 Sim 启动状态机" in completed.stdout
    assert _fake_python_args(completed) == ["python/sim/main_server.py", "--monte-carlo"]


def test_python_modes_are_split_into_sim_and_real_directories():
    assert (PROJECT_ROOT / "python" / "sim" / "main_server.py").is_file()
    assert (PROJECT_ROOT / "python" / "sim" / "udp_server.py").is_file()
    assert (PROJECT_ROOT / "python" / "real" / "main.py").is_file()
    assert (PROJECT_ROOT / "python" / "real" / "runtime.py").is_file()
    assert (PROJECT_ROOT / "python" / "real_pinocchio" / "main.py").is_file()
    assert (PROJECT_ROOT / "c_interface" / "real" / "real_controller.c").is_file()


def test_real_entrypoint_bootstraps_python_package_path(monkeypatch):
    python_root = str(PROJECT_ROOT / "python")
    monkeypatch.setattr(sys, "path", [p for p in sys.path if p != python_root])

    spec = importlib.util.spec_from_file_location(
        "real_entrypoint_under_test",
        PROJECT_ROOT / "python" / "real" / "main.py",
    )
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)

    assert python_root in sys.path


def test_real_pinocchio_entrypoint_bootstraps_python_package_path(monkeypatch):
    python_root = str(PROJECT_ROOT / "python")
    monkeypatch.setattr(sys, "path", [p for p in sys.path if p != python_root])

    spec = importlib.util.spec_from_file_location(
        "real_pinocchio_entrypoint_under_test",
        PROJECT_ROOT / "python" / "real_pinocchio" / "main.py",
    )
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)

    assert python_root in sys.path
