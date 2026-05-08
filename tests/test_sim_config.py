from __future__ import annotations

import sys
from pathlib import Path


sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "python"))

from config import Config


def test_config_points_to_am_dpbsurdf0422_left_arm_model():
    assert Path(Config.URDF_PATH).name == "AM-DPBSURDF0422.urdf"
    assert Path(Config.URDF_PATH).is_file()
    assert Config.JOINT_NAMES == [
        "ArmL02_Joint",
        "AM-D02-J14_Joint",
        "ArmL04_Joint",
        "ArmL05_Joint",
        "ArmL06_Joint",
        "ArmL07_Joint",
        "ArmL07Output_Joint",
    ]
    assert Config.TORQUE_LIMITS.tolist() == [40.0, 40.0, 27.0, 27.0, 9.0, 9.0, 9.0]
