from __future__ import annotations

import math
import os
import struct
import subprocess
import sys
import textwrap
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np
import pytest


sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "python"))

from config import Config


REAL_LEFT_ARM_TORQUE_LIMITS = [40.0, 40.0, 27.0, 27.0, 7.0, 7.0, 9.0]
REAL_RIGHT_ARM_TORQUE_LIMITS = [40.0, 40.0, 27.0, 27.0, 9.0, 9.0, 9.0]
REAL_LEFT_ARM_JOINT_LIMITS_DEG = [
    (-89.971835, 89.971835),
    (-20.587610, 89.954374),
    (-45.836624, 68.754935),
    (-119.748454, 119.954374),
    (-45.836624, 45.836624),
    (-45.263666, 61.306275),
    (-61.306275, 61.306275),
]
REAL_RIGHT_ARM_JOINT_LIMITS_RAD = [
    (-2.405, 2.2175),
    (-0.6605, 2.203),
    (-1.763, 1.594),
    (-0.0165, 2.3235),
    (-1.5935, 1.574),
    (-0.6015, 0.6755),
    (-1.1075, 1.068),
]
OPENARM_FOLLOWER_FRICTION_FC = [0.306, 0.306, 0.400, 0.166, 0.050, 0.093, 0.172]
OPENARM_FOLLOWER_FRICTION_K = [28.417, 28.417, 29.065, 130.038, 151.771, 242.287, 7.888]
OPENARM_FOLLOWER_FRICTION_FV = [0.063, 0.063, 0.604, 0.813, 0.029, 0.072, 0.084]
OPENARM_FOLLOWER_FRICTION_FO = [0.088, 0.088, 0.008, -0.058, 0.005, 0.009, -0.059]


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


def _quat_wxyz_to_rotmat(quat: np.ndarray) -> np.ndarray:
    w, x, y, z = np.asarray(quat, dtype=np.float64)
    return np.array(
        [
            [1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w), 2.0 * (x * z + y * w)],
            [2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - x * w)],
            [2.0 * (x * z - y * w), 2.0 * (y * z + x * w), 1.0 - 2.0 * (x * x + y * y)],
        ],
        dtype=np.float64,
    )


def _quat_multiply_wxyz(q1: np.ndarray, q2: np.ndarray) -> np.ndarray:
    w1, x1, y1, z1 = np.asarray(q1, dtype=np.float64)
    w2, x2, y2, z2 = np.asarray(q2, dtype=np.float64)
    return np.array(
        [
            w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
            w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
            w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
            w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
        ],
        dtype=np.float64,
    )


def _assert_quat_equivalent(actual: np.ndarray, expected: np.ndarray, atol: float = 1e-12) -> None:
    actual = np.asarray(actual, dtype=np.float64)
    expected = np.asarray(expected, dtype=np.float64)
    if actual.shape == (4,):
        if np.linalg.norm(actual + expected) < np.linalg.norm(actual - expected):
            actual = -actual
        np.testing.assert_allclose(actual, expected, atol=atol)
        return
    assert actual.shape == expected.shape
    for index in np.ndindex(actual.shape[:-1]):
        _assert_quat_equivalent(actual[index], expected[index], atol=atol)


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


def test_config_controls_both_arms():
    assert Path(Config.URDF_PATH).name == "AM-DPBSURDF0422.urdf"
    assert Path(Config.URDF_PATH).is_file()
    assert Config.ARM_JOINTS == 7
    assert Config.NUM_ARMS == 2
    assert Config.NUM_JOINTS == 14
    assert Config.LEFT_JOINT_NAMES == [
        "ArmL02_Joint",
        "AM-D02-J14_Joint",
        "ArmL04_Joint",
        "ArmL05_Joint",
        "ArmL06_Joint",
        "ArmL07_Joint",
        "ArmL07Output_Joint",
    ]
    assert Config.RIGHT_JOINT_NAMES == [
        "ArmR01_Joint_duplicate_2",
        "AM-D02R-J03_Joint",
        "ArmR04_Joint",
        "ArmR05_Link",
        "ArmR06_Link",
        "ArmR07_Link",
        "ArmR07Output_Link",
    ]
    assert Config.JOINT_NAMES == Config.LEFT_JOINT_NAMES + Config.RIGHT_JOINT_NAMES
    assert Config.NUM_BODY_JOINTS == 3
    assert Config.BODY_JOINT_NAMES == ["Waist01_Joint", "Waist02_Joint", "Body0422_Joint"]
    np.testing.assert_allclose(
        Config.BODY_JOINT_LIMITS_RAD,
        [[0.0, 2.09], [-2.09, 0.0], [-1.57, 1.57]],
    )
    assert Config.ENABLE_BODY_GUI is True
    assert Config.TORQUE_LIMITS.tolist() == REAL_LEFT_ARM_TORQUE_LIMITS + REAL_RIGHT_ARM_TORQUE_LIMITS
    expected_limits_deg = np.vstack(
        [
            np.array(REAL_LEFT_ARM_JOINT_LIMITS_DEG, dtype=np.float64),
            np.rad2deg(np.array(REAL_RIGHT_ARM_JOINT_LIMITS_RAD, dtype=np.float64)),
        ]
    )
    np.testing.assert_allclose(Config.JOINT_LIMITS_DEG, expected_limits_deg)
    np.testing.assert_allclose(
        Config.JOINT_LIMITS_RAD,
        np.deg2rad(expected_limits_deg),
    )
    assert Config.FIX_UNCONTROLLED_JOINTS is True


def test_fix_uncontrolled_joints_marks_only_non_controlled_joints_fixed():
    from sim_scene import _fix_uncontrolled_joints

    root = ET.fromstring(
        """
        <robot>
          <joint name="ArmL02_Joint" type="revolute">
            <axis xyz="0 0 1" />
            <limit lower="-1" upper="1" />
          </joint>
          <joint name="Waist01_Joint" type="revolute">
            <axis xyz="0 0 1" />
            <limit lower="-1" upper="1" />
            <dynamics damping="1" />
            <mimic joint="other" />
          </joint>
        </robot>
        """
    )

    fixed_names = _fix_uncontrolled_joints(root, ["ArmL02_Joint"])

    controlled = root.find("./joint[@name='ArmL02_Joint']")
    fixed = root.find("./joint[@name='Waist01_Joint']")
    assert fixed_names == ["Waist01_Joint"]
    assert controlled is not None
    assert controlled.attrib["type"] == "revolute"
    assert controlled.find("axis") is not None
    assert controlled.find("limit") is not None
    assert fixed is not None
    assert fixed.attrib["type"] == "fixed"
    assert fixed.find("axis") is None
    assert fixed.find("limit") is None
    assert fixed.find("dynamics") is None
    assert fixed.find("mimic") is None


def test_initial_target_qpos_keeps_elbow_raised_and_home_qpos_starts_safe():
    arm_init = [0.0, 0.0, 0.0, np.pi / 2, 0.0, 0.0, 0.0]
    np.testing.assert_allclose(
        Config.INIT_QPOS,
        arm_init + arm_init,
    )
    np.testing.assert_allclose(Config.LEFT_HOME_QPOS, np.zeros(Config.ARM_JOINTS))
    np.testing.assert_allclose(
        Config.RIGHT_HOME_QPOS,
        [0.0, 0.0, 0.0, 0.03, 0.0, 0.0, 0.0],
    )
    np.testing.assert_allclose(
        Config.HOME_QPOS,
        np.concatenate([Config.LEFT_HOME_QPOS, Config.RIGHT_HOME_QPOS]),
    )

    span = Config.RIGHT_JOINT_LIMITS_RAD[:, 1] - Config.RIGHT_JOINT_LIMITS_RAD[:, 0]
    safe_min = Config.RIGHT_JOINT_LIMITS_RAD[:, 0] + Config.CONTROL_JOINT_LIMIT_INSET_RATIO * span
    safe_max = Config.RIGHT_JOINT_LIMITS_RAD[:, 1] - Config.CONTROL_JOINT_LIMIT_INSET_RATIO * span
    assert np.all(Config.RIGHT_HOME_QPOS >= safe_min)
    assert np.all(Config.RIGHT_HOME_QPOS <= safe_max)


def test_sim_home_starts_safe_while_target_still_uses_init_qpos_fk():
    pytest.importorskip("mujoco")

    from sim_env import MujocoSimEnv

    env = MujocoSimEnv()
    env.reset(Config.INIT_QPOS)
    env.forward()
    target_pos = env.get_all_ee_pos().copy()
    target_quat = env.get_all_ee_quat().copy()

    env.reset(Config.HOME_QPOS)
    env.forward()
    env.set_all_target_poses_base(target_pos, target_quat)

    np.testing.assert_allclose(env.get_qpos(), Config.HOME_QPOS)
    target_pos_base, target_quat_base = env.get_all_target_poses_base()
    np.testing.assert_allclose(target_pos_base, target_pos, atol=1e-12)
    _assert_quat_equivalent(target_quat_base, target_quat, atol=1e-12)
    assert not np.allclose(env.get_all_ee_pos(), target_pos)


def test_left_arm_tcp_offset_is_at_dual_output_link_tip():
    np.testing.assert_allclose(Config.TCP_OFFSET, [0.0, 0.07, -0.03])
    np.testing.assert_allclose(Config.LEFT_TCP_OFFSET, [0.0, 0.07, -0.03])

    for geometry_tag in ("visual", "collision"):
        bbox_min, bbox_max = _link_mesh_bbox("ArmL07Output_Link", geometry_tag)
        span = bbox_max - bbox_min

        assert bbox_min[0] <= Config.TCP_OFFSET[0] <= bbox_max[0]
        assert bbox_min[2] <= Config.TCP_OFFSET[2] <= bbox_max[2]
        assert Config.TCP_OFFSET[1] > bbox_min[1] + 0.8 * span[1]
        assert Config.TCP_OFFSET[1] <= bbox_max[1] + 0.01


def test_right_arm_tcp_offset_is_at_dual_output_link_tip():
    np.testing.assert_allclose(Config.RIGHT_TCP_OFFSET, [0.0, -0.07, 0.03])

    for geometry_tag in ("visual", "collision"):
        bbox_min, bbox_max = _link_mesh_bbox("ArmR07Output_Link", geometry_tag)
        span = bbox_max - bbox_min

        assert bbox_min[0] <= Config.RIGHT_TCP_OFFSET[0] <= bbox_max[0]
        assert bbox_min[2] <= Config.RIGHT_TCP_OFFSET[2] <= bbox_max[2]
        assert Config.RIGHT_TCP_OFFSET[1] < bbox_max[1] - 0.8 * span[1]
        assert Config.RIGHT_TCP_OFFSET[1] >= bbox_min[1] - 0.01


def test_tcp_frame_quats_align_left_tcp_axes_with_right_arm():
    half_sqrt = np.sqrt(0.5)
    np.testing.assert_allclose(Config.LEFT_TCP_FRAME_QUAT, [0.0, 0.0, half_sqrt, half_sqrt])
    np.testing.assert_allclose(Config.RIGHT_TCP_FRAME_QUAT, [half_sqrt, half_sqrt, 0.0, 0.0])
    assert Config.TCP_FRAME_QUATS.shape == (Config.NUM_ARMS, 4)


def test_mujoco_initial_tcp_frames_are_aligned_between_arms():
    pytest.importorskip("mujoco")

    from sim_env import MujocoSimEnv

    env = MujocoSimEnv()
    env.reset(Config.INIT_QPOS)
    env.forward()

    np.testing.assert_allclose(
        env.get_ee_rotmat(Config.LEFT_ARM),
        env.get_ee_rotmat(Config.RIGHT_ARM),
        atol=2e-5,
    )
    robot_forward_base = np.array([1.0, 0.0, 0.0])
    np.testing.assert_allclose(
        env.get_ee_rotmat(Config.LEFT_ARM)[:, 2],
        robot_forward_base,
        atol=2e-5,
    )
    np.testing.assert_allclose(
        env.get_ee_rotmat(Config.RIGHT_ARM)[:, 2],
        robot_forward_base,
        atol=2e-5,
    )


def test_target_frame_marker_uses_body_origin_and_zero_pose_axes():
    pytest.importorskip("mujoco")

    import mujoco

    from sim_env import MujocoSimEnv

    env = MujocoSimEnv()
    env.reset(Config.INIT_QPOS)
    env.forward()

    marker_id = mujoco.mj_name2id(
        env.model,
        mujoco.mjtObj.mjOBJ_BODY,
        Config.TARGET_FRAME_MARKER_BODY,
    )
    assert marker_id >= 0
    np.testing.assert_allclose(
        env.get_target_frame_origin_base(),
        Config.TARGET_FRAME_ORIGIN_BASE_ZERO,
        atol=1e-12,
    )
    np.testing.assert_allclose(env.get_target_frame_rotmat_base(), np.eye(3), atol=1e-12)
    np.testing.assert_allclose(
        env.data.xpos[marker_id],
        Config.TARGET_FRAME_ORIGIN_BASE_ZERO,
        atol=1e-12,
    )
    np.testing.assert_allclose(
        env.data.xmat[marker_id].reshape(3, 3),
        np.eye(3),
        atol=1e-12,
    )


def test_target_pose_is_stored_in_body0422_coordinates_and_displayed_in_world():
    pytest.importorskip("mujoco")

    from sim_env import MujocoSimEnv

    env = MujocoSimEnv()
    env.reset(Config.INIT_QPOS)
    env.forward()

    target_body = np.array([0.1, -0.2, 0.3], dtype=np.float64)
    target_quat = np.array([0.5, -0.5, 0.5, -0.5], dtype=np.float64)
    env.set_target_pose(target_body, target_quat, arm=Config.LEFT_ARM)

    pos_body, quat = env.get_target_pose(Config.LEFT_ARM)
    np.testing.assert_allclose(pos_body, target_body)
    np.testing.assert_allclose(quat, target_quat)
    np.testing.assert_allclose(
        env.data.mocap_pos[env.target_mocap_ids[Config.LEFT_ARM]],
        env.get_target_frame_origin_base() + target_body,
    )


def test_target_frame_rotates_with_body0422_relative_to_zero_pose():
    pytest.importorskip("mujoco")

    import mujoco

    from sim_env import MujocoSimEnv

    env = MujocoSimEnv()
    env.reset(Config.INIT_QPOS)
    env.forward()

    body_id = mujoco.mj_name2id(env.model, mujoco.mjtObj.mjOBJ_BODY, Config.TARGET_FRAME_BODY_NAME)
    marker_id = mujoco.mj_name2id(env.model, mujoco.mjtObj.mjOBJ_BODY, Config.TARGET_FRAME_MARKER_BODY)
    body_zero = env.data.xmat[body_id].reshape(3, 3).copy()

    env.set_body_qpos(np.array([0.2, -0.3, 0.4], dtype=np.float64))
    env.forward()

    body_current = env.data.xmat[body_id].reshape(3, 3).copy()
    expected_rot = body_current @ body_zero.T
    target_body = np.array([0.12, -0.03, 0.08], dtype=np.float64)
    target_quat_body = np.array([0.9238795325, 0.0, 0.0, 0.3826834324], dtype=np.float64)
    env.set_target_pose(target_body, target_quat_body, arm=Config.RIGHT_ARM)
    target_world_pos, target_world_quat = env.get_all_target_poses_base()

    np.testing.assert_allclose(env.get_target_frame_rotmat_base(), expected_rot, atol=1e-12)
    np.testing.assert_allclose(env.data.xpos[marker_id], env.data.xpos[body_id], atol=1e-12)
    np.testing.assert_allclose(env.data.xmat[marker_id].reshape(3, 3), expected_rot, atol=1e-12)
    np.testing.assert_allclose(
        env.data.mocap_pos[env.target_mocap_ids[Config.RIGHT_ARM]],
        env.get_target_frame_origin_base() + expected_rot @ target_body,
        atol=1e-12,
    )
    np.testing.assert_allclose(target_world_pos[Config.RIGHT_ARM], env.get_target_frame_origin_base() + expected_rot @ target_body)
    np.testing.assert_allclose(
        _quat_wxyz_to_rotmat(target_world_quat[Config.RIGHT_ARM]),
        expected_rot @ _quat_wxyz_to_rotmat(target_quat_body),
        atol=1e-9,
    )


def test_dragged_target_mocap_updates_body0422_target_coordinates():
    pytest.importorskip("mujoco")

    from sim_env import MujocoSimEnv

    env = MujocoSimEnv()
    env.reset(Config.INIT_QPOS)
    env.forward()

    env.set_target_pose(np.array([0.1, 0.0, 0.0], dtype=np.float64), arm=Config.LEFT_ARM)
    env.set_body_qpos(np.array([0.2, -0.3, 0.4], dtype=np.float64))
    env.forward()
    target_rot = env.get_target_frame_rotmat_base()
    dragged_world = env.get_target_frame_origin_base() + target_rot @ np.array([0.2, -0.1, 0.05])
    mocap_id = int(env.target_mocap_ids[Config.LEFT_ARM])
    env.data.mocap_pos[mocap_id] = dragged_world

    env.forward()

    pos_body, _ = env.get_target_pose(Config.LEFT_ARM)
    np.testing.assert_allclose(
        pos_body,
        target_rot.T @ (dragged_world - env.get_target_frame_origin_base()),
        atol=1e-12,
    )
    np.testing.assert_allclose(env.data.mocap_pos[mocap_id], dragged_world, atol=1e-12)


def test_target_pose_follows_body0422_translation_when_unfixed():
    pytest.importorskip("mujoco")

    script = textwrap.dedent(
        """
        import os
        import sys
        import numpy as np
        import mujoco

        os.environ["AM_D02_FIX_UNCONTROLLED_JOINTS"] = "0"
        sys.path.insert(0, "python")
        from config import Config
        from sim_env import MujocoSimEnv

        env = MujocoSimEnv()
        env.reset(Config.INIT_QPOS)
        env.forward()

        body_id = mujoco.mj_name2id(env.model, mujoco.mjtObj.mjOBJ_BODY, Config.TARGET_FRAME_BODY_NAME)
        marker_id = mujoco.mj_name2id(env.model, mujoco.mjtObj.mjOBJ_BODY, Config.TARGET_FRAME_MARKER_BODY)
        assert body_id >= 0
        assert marker_id >= 0

        target_body = np.array([0.12, -0.03, 0.08], dtype=np.float64)
        env.set_target_pose(target_body, arm=Config.RIGHT_ARM)
        before_body_target, _ = env.get_target_pose(Config.RIGHT_ARM)

        env.set_body_qpos(np.array([0.2, -0.3, 0.4], dtype=np.float64))
        env.forward()

        origin = env.data.xpos[body_id].copy()
        target_world = env.data.mocap_pos[env.target_mocap_ids[Config.RIGHT_ARM]].copy()
        marker_world = env.data.xpos[marker_id].copy()
        marker_rot = env.data.xmat[marker_id].reshape(3, 3).copy()
        after_body_target, _ = env.get_target_pose(Config.RIGHT_ARM)

        print("origin", *origin)
        print("target_world", *target_world)
        print("marker_world", *marker_world)
        print("marker_rot", *marker_rot.reshape(-1))
        print("before", *before_body_target)
        print("after", *after_body_target)
        """
    )
    result = subprocess.run(
        [sys.executable, "-c", script],
        cwd=_repo_root(),
        check=True,
        text=True,
        capture_output=True,
    )
    rows = {
        line.split()[0]: np.array([float(value) for value in line.split()[1:]], dtype=np.float64)
        for line in result.stdout.strip().splitlines()
    }
    target_body = np.array([0.12, -0.03, 0.08], dtype=np.float64)
    np.testing.assert_allclose(rows["target_world"], rows["origin"] + rows["marker_rot"].reshape(3, 3) @ target_body, atol=1e-12)
    np.testing.assert_allclose(rows["marker_world"], rows["origin"], atol=1e-12)
    np.testing.assert_allclose(rows["before"], target_body, atol=1e-12)
    np.testing.assert_allclose(rows["after"], target_body, atol=1e-12)


def test_dual_urdf_left_arm_joint_axes_follow_real_machine_rotation_direction():
    axes = _read_dual_urdf_zero_pose_axes()

    expected_axes = {
        "ArmL02_Joint": [0.0, 0.0, 1.0],
        "AM-D02-J14_Joint": [-1.0, 0.0, 0.0],
        "ArmL04_Joint": [0.0, 1.0, 0.0],
        "ArmL05_Joint": [0.0, 0.0, -1.0],
        "ArmL06_Joint": [0.0, 1.0, 0.0],
        "ArmL07_Joint": [-1.0, 0.0, 0.0],
        "ArmL07Output_Joint": [0.0, 0.0, -1.0],
    }
    assert expected_axes.keys() <= axes.keys()
    for joint_name, expected_axis in expected_axes.items():
        _assert_same_direction(axes[joint_name], expected_axis)


def test_dual_urdf_right_arm_joint_axes_follow_real_machine_rotation_direction():
    axes = _read_dual_urdf_zero_pose_axes()

    expected_axes = {
        "ArmR01_Joint": [0.0, 0.0, -1.0],
        "AM-D02R-J03_Joint": [-1.0, 0.0, 0.0],
        "ArmR04_Joint": [0.0, 1.0, 0.0],
        "ArmR05_Link": [0.0, 0.0, -1.0],
        "ArmR06_Link": [0.0, 1.0, 0.0],
        "ArmR07_Link": [-1.0, 0.0, 0.0],
        "ArmR07Output_Link": [0.0, 0.0, 1.0],
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


def test_config_exposes_openarm_follower_friction_parameters():
    np.testing.assert_allclose(Config.FOLLOWER_FRICTION_FC, OPENARM_FOLLOWER_FRICTION_FC)
    np.testing.assert_allclose(Config.FOLLOWER_FRICTION_K, OPENARM_FOLLOWER_FRICTION_K)
    np.testing.assert_allclose(Config.FOLLOWER_FRICTION_FV, OPENARM_FOLLOWER_FRICTION_FV)
    np.testing.assert_allclose(Config.FOLLOWER_FRICTION_FO, OPENARM_FOLLOWER_FRICTION_FO)
    np.testing.assert_allclose(
        Config.FOLLOWER_FRICTION_FC_14,
        np.tile(OPENARM_FOLLOWER_FRICTION_FC, Config.NUM_ARMS),
    )
    assert Config.FOLLOWER_FRICTION_FC_14.shape == (Config.NUM_JOINTS,)


def test_openarm_follower_friction_can_be_disabled_by_env():
    script = textwrap.dedent(
        """
        import sys
        sys.path.insert(0, "python")
        from config import Config
        print(Config.ENABLE_FOLLOWER_FRICTION)
        """
    )
    env = os.environ.copy()
    env["AM_D02_ENABLE_FOLLOWER_FRICTION"] = "0"
    result = subprocess.run(
        [sys.executable, "-c", script],
        cwd=_repo_root(),
        env=env,
        check=True,
        text=True,
        capture_output=True,
    )
    assert result.stdout.strip() == "False"


def test_sim_passive_damping_and_armature_are_always_zero():
    script = textwrap.dedent(
        """
        import numpy as np
        import sys
        sys.path.insert(0, "python")
        from config import Config
        print(float(np.linalg.norm(Config.JOINT_DAMPING)))
        print(float(np.linalg.norm(Config.JOINT_ARMATURE)))
        print(Config.ENABLE_FOLLOWER_FRICTION)
        """
    )
    env = os.environ.copy()
    result = subprocess.run(
        [sys.executable, "-c", script],
        cwd=_repo_root(),
        env=env,
        check=True,
        text=True,
        capture_output=True,
    )
    assert result.stdout.strip().splitlines() == ["0.0", "0.0", "True"]


def test_mujoco_env_has_zero_damping_and_armature_but_keeps_follower_friction():
    pytest.importorskip("mujoco")

    script = textwrap.dedent(
        """
        import numpy as np
        import sys
        sys.path.insert(0, "python")
        from config import Config
        from sim_env import MujocoSimEnv

        env = MujocoSimEnv()
        print(float(np.linalg.norm(env.model.dof_damping[env.dof_ids])))
        print(float(np.linalg.norm(env.model.dof_armature[env.dof_ids])))
        print(float(np.linalg.norm(env.get_follower_friction_torque(np.ones(Config.NUM_JOINTS)))))
        """
    )
    result = subprocess.run(
        [sys.executable, "-c", script],
        cwd=_repo_root(),
        env=os.environ.copy(),
        check=True,
        text=True,
        capture_output=True,
    )
    values = [float(value) for value in result.stdout.strip().splitlines()]
    assert values[0] == pytest.approx(0.0)
    assert values[1] == pytest.approx(0.0)
    assert values[2] > 0.0


def test_mujoco_env_computes_openarm_follower_friction_torque():
    pytest.importorskip("mujoco")

    from sim_env import MujocoSimEnv

    env = MujocoSimEnv()
    qvel = np.linspace(-1.2, 1.4, Config.NUM_JOINTS)
    expected = (
        + Config.FOLLOWER_FRICTION_FV_14 * qvel
        + (Config.FOLLOWER_FRICTION_FO_14 + Config.FOLLOWER_FRICTION_FC_14)
        * np.tanh(0.1 * Config.FOLLOWER_FRICTION_K_14 * qvel)
    )

    np.testing.assert_allclose(env.get_follower_friction_torque(qvel), expected)
    np.testing.assert_allclose(
        env.get_follower_friction_torque(np.zeros(Config.NUM_JOINTS)),
        np.zeros(Config.NUM_JOINTS),
    )


def test_mujoco_env_friction_returns_zero_when_disabled():
    pytest.importorskip("mujoco")

    script = textwrap.dedent(
        """
        import numpy as np
        import sys
        sys.path.insert(0, "python")
        from config import Config
        from sim_env import MujocoSimEnv

        env = MujocoSimEnv()
        print(Config.ENABLE_FOLLOWER_FRICTION)
        print(np.linalg.norm(env.get_follower_friction_torque(np.ones(Config.NUM_JOINTS))))
        """
    )
    env = os.environ.copy()
    env["AM_D02_ENABLE_FOLLOWER_FRICTION"] = "0"
    result = subprocess.run(
        [sys.executable, "-c", script],
        cwd=_repo_root(),
        env=env,
        check=True,
        text=True,
        capture_output=True,
    )
    assert result.stdout.strip().splitlines() == ["False", "0.0"]


def test_mujoco_env_friction_changes_arm_acceleration_without_compensation():
    pytest.importorskip("mujoco")

    from sim_env import MujocoSimEnv

    env = MujocoSimEnv()
    env.reset(Config.INIT_QPOS)
    env.set_qvel(np.linspace(-0.5, 0.5, Config.NUM_JOINTS))
    env.forward()

    env.apply_torque(env.get_qfrc_bias())
    env.step()

    assert np.linalg.norm(env.get_applied_friction_torque()) > 0.0
    assert np.linalg.norm(env.data.qacc[env.dof_ids]) > 1e-6


def test_mujoco_env_keeps_body_joints_for_gui_by_default():
    pytest.importorskip("mujoco")

    import mujoco

    from sim_env import MujocoSimEnv

    env = MujocoSimEnv()

    assert env.model.nv == Config.NUM_JOINTS + Config.NUM_BODY_JOINTS
    assert env.has_body_joints()
    assert env.locked_dof_ids.size == 0
    assert env.locked_qpos_ids.size == 0
    assert [
        mujoco.mj_id2name(env.model, mujoco.mjtObj.mjOBJ_JOINT, joint_id)
        for joint_id in range(env.model.njnt)
    ] == Config.BODY_JOINT_NAMES + Config.JOINT_NAMES

    env.reset(Config.INIT_QPOS)
    body_q = np.array([0.3, -0.4, 0.2], dtype=np.float64)
    env.set_body_qpos(body_q)
    env.forward()
    np.testing.assert_allclose(env.get_body_qpos(), body_q)
    np.testing.assert_allclose(env.data.qvel[env.body_dof_ids], 0.0)
    np.testing.assert_allclose(env.data.qacc[env.body_dof_ids], 0.0)
    env.apply_torque(env.get_qfrc_bias() + env.get_follower_friction_torque())
    env.step()
    np.testing.assert_allclose(env.get_body_qpos(), body_q)
    np.testing.assert_allclose(env.data.qvel[env.body_dof_ids], 0.0)
    np.testing.assert_allclose(env.data.qacc[env.body_dof_ids], 0.0)
    np.testing.assert_allclose(env.data.qacc[env.dof_ids], 0.0, atol=1e-9)


def test_commanded_body_sim_applies_viewer_external_force_to_arm_dofs():
    pytest.importorskip("mujoco")

    from sim_env import MujocoSimEnv

    env = MujocoSimEnv()
    env.reset(Config.INIT_QPOS)
    body_q = np.array([0.3, -0.4, 0.2], dtype=np.float64)
    env.set_body_qpos(body_q)
    env.forward()

    env.apply_torque(env.get_qfrc_bias() + env.get_follower_friction_torque())
    env.step()
    qacc_without_force = env.data.qacc[env.dof_ids].copy()

    env.reset(Config.INIT_QPOS)
    env.set_body_qpos(body_q)
    env.forward()
    env.apply_torque(env.get_qfrc_bias() + env.get_follower_friction_torque())
    env.data.xfrc_applied[env.ee_body_ids[Config.LEFT_ARM], :3] = [10.0, 0.0, 0.0]
    env.step()

    assert np.linalg.norm(env.data.qacc[env.dof_ids] - qacc_without_force) > 1e-3
    assert np.linalg.norm(env.data.qacc[env.dof_ids[:Config.ARM_JOINTS]]) > 1e-3
    np.testing.assert_allclose(env.get_body_qpos(), body_q)
    np.testing.assert_allclose(env.data.qvel[env.body_dof_ids], 0.0)
    np.testing.assert_allclose(env.data.qacc[env.body_dof_ids], 0.0)


def test_mujoco_env_fixes_uncontrolled_joints_when_body_gui_disabled():
    pytest.importorskip("mujoco")

    script = textwrap.dedent(
        """
        import sys
        sys.path.insert(0, "python")
        from config import Config
        from sim_env import MujocoSimEnv

        env = MujocoSimEnv()
        print(Config.ENABLE_BODY_GUI)
        print(env.model.nv)
        print(env.has_body_joints())
        print(env.locked_dof_ids.size)
        """
    )
    env = os.environ.copy()
    env["AM_D02_ENABLE_BODY_GUI"] = "0"
    result = subprocess.run(
        [sys.executable, "-c", script],
        cwd=_repo_root(),
        env=env,
        check=True,
        text=True,
        capture_output=True,
    )
    lines = result.stdout.strip().splitlines()

    assert lines == ["False", "14", "False", "0"]


def test_mujoco_env_keeps_body_gui_joints_commanded_when_fixing_disabled():
    pytest.importorskip("mujoco")

    script = textwrap.dedent(
        """
        import sys
        sys.path.insert(0, "python")
        from config import Config
        from sim_env import MujocoSimEnv

        env = MujocoSimEnv()
        print(Config.FIX_UNCONTROLLED_JOINTS)
        print(Config.ENABLE_BODY_GUI)
        print(env.model.nv)
        print(env.has_body_joints())
        print(env.locked_dof_ids.size)
        """
    )
    env = os.environ.copy()
    env["AM_D02_FIX_UNCONTROLLED_JOINTS"] = "0"
    result = subprocess.run(
        [sys.executable, "-c", script],
        cwd=_repo_root(),
        env=env,
        check=True,
        text=True,
        capture_output=True,
    )
    lines = result.stdout.strip().splitlines()

    assert lines == ["False", "True", "17", "True", "0"]
