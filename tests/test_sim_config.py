from __future__ import annotations

import math
import struct
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np
import pytest


sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "python"))

from config import Config


REAL_ARM_TORQUE_LIMITS = [40.0, 40.0, 27.0, 27.0, 7.0, 7.0, 9.0]
REAL_ARM_JOINT_LIMITS_DEG = [
    (-89.971835, 89.971835),
    (-89.954374, 20.587610),
    (-68.754935, 45.836624),
    (-119.748454, 119.954374),
    (-45.836624, 45.836624),
    (-61.306275, 45.263666),
    (-61.306275, 61.306275),
]


def _repo_root() -> Path:
    return Path(__file__).resolve().parents[1]


def _rpy_to_rotation_matrix(rpy: str) -> np.ndarray:
    roll, pitch, yaw = (float(part) for part in rpy.split())
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)

    rotation_x = np.array([[1.0, 0.0, 0.0], [0.0, cr, -sr], [0.0, sr, cr]])
    rotation_y = np.array([[cp, 0.0, sp], [0.0, 1.0, 0.0], [-sp, 0.0, cp]])
    rotation_z = np.array([[cy, -sy, 0.0], [sy, cy, 0.0], [0.0, 0.0, 1.0]])
    return rotation_z @ rotation_y @ rotation_x


def _read_dual_urdf_zero_pose_axes() -> dict[str, np.ndarray]:
    tree = ET.parse(_repo_root() / "AM-DPBSURDF0422" / "urdf" / "AM-DPBSURDF0422.urdf")
    children_by_parent: dict[str, list[tuple[str, str, np.ndarray, np.ndarray]]] = {}
    for joint in tree.getroot().findall("joint"):
        parent = joint.find("parent").attrib["link"]
        child = joint.find("child").attrib["link"]
        origin = joint.find("origin")
        rpy = origin.attrib.get("rpy", "0 0 0") if origin is not None else "0 0 0"
        axis = joint.find("axis")
        axis_xyz = (
            np.array([float(value) for value in axis.attrib["xyz"].split()], dtype=np.float64)
            if axis is not None
            else np.zeros(3, dtype=np.float64)
        )
        children_by_parent.setdefault(parent, []).append(
            (joint.attrib["name"], child, _rpy_to_rotation_matrix(rpy), axis_xyz)
        )

    axes: dict[str, np.ndarray] = {}

    def visit(link: str, rotation_from_body: np.ndarray) -> None:
        for name, child, origin_rotation, axis_xyz in children_by_parent.get(link, []):
            joint_rotation = rotation_from_body @ origin_rotation
            if np.linalg.norm(axis_xyz) > 0:
                axes[name] = joint_rotation @ axis_xyz
            visit(child, joint_rotation)

    visit("Body0422_Link", np.eye(3))
    return axes


def _binary_stl_vertices(path: Path) -> np.ndarray:
    data = path.read_bytes()
    triangle_count = struct.unpack_from("<I", data, 80)[0]
    vertices = []
    offset = 84
    for _ in range(triangle_count):
        triangle = struct.unpack_from("<12fH", data, offset)
        vertices.extend(
            [
                triangle[3:6],
                triangle[6:9],
                triangle[9:12],
            ]
        )
        offset += 50

    return np.asarray(vertices, dtype=np.float64)


def _resolve_package_mesh_path(filename: str) -> Path:
    if not filename.startswith("package://AM-DPBSURDF0422/"):
        raise ValueError(f"unexpected mesh path: {filename}")
    return _repo_root() / "AM-DPBSURDF0422" / filename.removeprefix("package://AM-DPBSURDF0422/")


def _link_mesh_bbox(link_name: str, geometry_tag: str) -> tuple[np.ndarray, np.ndarray]:
    tree = ET.parse(_repo_root() / "AM-DPBSURDF0422" / "urdf" / "AM-DPBSURDF0422.urdf")
    link = tree.getroot().find(f"./link[@name='{link_name}']")
    assert link is not None

    geometry = link.find(geometry_tag)
    assert geometry is not None
    origin = geometry.find("origin")
    mesh = geometry.find("./geometry/mesh")
    assert origin is not None
    assert mesh is not None

    xyz = np.array([float(value) for value in origin.attrib["xyz"].split()], dtype=np.float64)
    rotation = _rpy_to_rotation_matrix(origin.attrib.get("rpy", "0 0 0"))
    vertices = _binary_stl_vertices(_resolve_package_mesh_path(mesh.attrib["filename"]))
    transformed_vertices = vertices @ rotation.T + xyz
    return transformed_vertices.min(axis=0), transformed_vertices.max(axis=0)


def _assert_same_direction(actual: np.ndarray, expected: list[float]) -> None:
    actual_unit = actual / np.linalg.norm(actual)
    expected_unit = np.asarray(expected, dtype=np.float64)
    expected_unit = expected_unit / np.linalg.norm(expected_unit)
    assert float(actual_unit @ expected_unit) > 0.99


def test_config_keeps_dual_arm_model_and_controls_left_arm():
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
    assert Config.TORQUE_LIMITS.tolist() == REAL_ARM_TORQUE_LIMITS
    np.testing.assert_allclose(Config.JOINT_LIMITS_DEG, REAL_ARM_JOINT_LIMITS_DEG)
    np.testing.assert_allclose(
        Config.JOINT_LIMITS_RAD,
        np.deg2rad(np.array(REAL_ARM_JOINT_LIMITS_DEG, dtype=np.float64)),
    )


def test_initial_target_qpos_is_elbow_raised_90_degrees():
    np.testing.assert_allclose(
        Config.INIT_QPOS,
        [0.0, 0.0, 0.0, np.pi / 2, 0.0, 0.0, 0.0],
    )


def test_left_arm_tcp_offset_is_at_dual_output_link_tip():
    np.testing.assert_allclose(Config.TCP_OFFSET, [0.0, 0.07, -0.03])

    for geometry_tag in ("visual", "collision"):
        bbox_min, bbox_max = _link_mesh_bbox("ArmL07Output_Link", geometry_tag)
        span = bbox_max - bbox_min

        assert bbox_min[0] <= Config.TCP_OFFSET[0] <= bbox_max[0]
        assert bbox_min[2] <= Config.TCP_OFFSET[2] <= bbox_max[2]
        assert Config.TCP_OFFSET[1] > bbox_min[1] + 0.8 * span[1]
        assert Config.TCP_OFFSET[1] <= bbox_max[1] + 0.01


def test_dual_urdf_left_arm_joint_axes_follow_real_machine_rotation_direction():
    axes = _read_dual_urdf_zero_pose_axes()

    expected_axes = {
        "ArmL02_Joint": [0.0, 0.0, 1.0],
        "AM-D02-J14_Joint": [1.0, 0.0, 0.0],
        "ArmL04_Joint": [0.0, -1.0, 0.0],
        "ArmL05_Joint": [0.011054, 0.0, -0.999939],
        "ArmL06_Joint": [0.0, -1.0, 0.0],
        "ArmL07_Joint": [0.999939, 0.0, 0.011054],
        "ArmL07Output_Joint": [0.011054, 0.000656, -0.999939],
    }
    assert expected_axes.keys() <= axes.keys()
    for joint_name, expected_axis in expected_axes.items():
        _assert_same_direction(axes[joint_name], expected_axis)


def test_dual_urdf_right_arm_joint_axes_follow_real_machine_rotation_direction():
    axes = _read_dual_urdf_zero_pose_axes()

    expected_axes = {
        "ArmR01_Joint": [0.0, 0.0, 1.0],
        "AM-D02R-J03_Joint": [1.0, 0.0, 0.0],
        "ArmR04_Joint": [0.0, -1.0, 0.0],
        "ArmR05_Link": [0.011054, 0.0, -0.999939],
        "ArmR06_Link": [0.0, -1.0, 0.0],
        "ArmR07_Link": [0.999939, 0.0, 0.011054],
        "ArmR07Output_Link": [0.011054, 0.000656, -0.999939],
    }
    assert expected_axes.keys() <= axes.keys()
    for joint_name, expected_axis in expected_axes.items():
        _assert_same_direction(axes[joint_name], expected_axis)


def test_mujoco_env_uses_real_machine_joint_limits_for_clipping():
    pytest.importorskip("mujoco")

    from sim_env import MujocoSimEnv

    env = MujocoSimEnv()

    expected_limits = Config.JOINT_LIMITS_RAD
    np.testing.assert_allclose(env.joint_lower, expected_limits[:, 0])
    np.testing.assert_allclose(env.joint_upper, expected_limits[:, 1])
    np.testing.assert_allclose(
        env.model.jnt_range[env.joint_ids],
        expected_limits,
    )

    too_high = expected_limits[:, 1] + 0.25
    env.set_qpos(too_high)
    assert env.enforce_joint_limits()
    np.testing.assert_allclose(env.get_qpos(), expected_limits[:, 1])
