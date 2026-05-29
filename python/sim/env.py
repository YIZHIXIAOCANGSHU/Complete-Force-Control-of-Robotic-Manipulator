"""MuJoCo 仿真环境封装。"""

from __future__ import annotations

import os

import mujoco
import numpy as np

from config import Config
from sim.scene import build_enhanced_model
from sim.state_packets import CONTROL_INPUT_PACKET_SIZE, fill_control_input_packet


class MujocoSimEnv:
    """AM-DPBSURDF0422 左右双臂十四轴 MuJoCo 仿真环境"""

    def __init__(self, urdf_path: str | None = None):
        if urdf_path is None:
            urdf_path = Config.URDF_PATH

        urdf_dir = os.path.dirname(os.path.abspath(urdf_path))
        urdf_filename = os.path.basename(urdf_path)

        old_cwd = os.getcwd()
        try:
            os.chdir(urdf_dir)
            model_controlled_joint_names = list(Config.JOINT_NAMES)
            if Config.ENABLE_BODY_GUI:
                model_controlled_joint_names = list(Config.BODY_JOINT_NAMES) + model_controlled_joint_names
            self.model = build_enhanced_model(
                urdf_filename,
                Config.TCP_OFFSETS,
                Config.TCP_FRAME_QUATS,
                Config.TARGET_FRAME_ORIGIN_BASE_ZERO,
                Config.TARGET_FRAME_QUAT_BASE,
                Config.TARGET_FRAME_MARKER_BODY,
                controlled_joint_names=model_controlled_joint_names,
                fix_uncontrolled_joints=Config.FIX_UNCONTROLLED_JOINTS,
            )
        finally:
            os.chdir(old_cwd)

        self.data = mujoco.MjData(self.model)
        self.model.opt.timestep = Config.DT

        self.joint_ids, self.dof_ids, self.joint_qpos_ids = self._resolve_joint_ids()
        self.body_joint_ids, self.body_dof_ids, self.body_qpos_ids = self._resolve_body_joint_ids()
        self.body_joint_lower, self.body_joint_upper = self._resolve_body_joint_limits()
        self._body_qpos_command = Config.BODY_INIT_QPOS.copy()
        self._command_body_joints = Config.ENABLE_BODY_GUI and self.has_body_joints()
        self.joint_lower, self.joint_upper = self._resolve_joint_limits()
        self._apply_joint_sim_properties()
        self.ee_body_ids = np.array(
            [self._resolve_required_body_id(name) for name in Config.END_EFFECTOR_BODIES],
            dtype=np.int32,
        )
        self.ee_body_id = int(self.ee_body_ids[Config.LEFT_ARM])
        self.target_mocap_ids = np.array(
            [self._get_mocap_id("target_pose_left"), self._get_mocap_id("target_pose_right")],
            dtype=np.int32,
        )
        self.reported_mocap_ids = np.array(
            [self._get_mocap_id("reported_pose_left"), self._get_mocap_id("reported_pose_right")],
            dtype=np.int32,
        )
        self.target_mocap_id = int(self.target_mocap_ids[Config.LEFT_ARM])
        self.reported_mocap_id = int(self.reported_mocap_ids[Config.LEFT_ARM])
        self.target_frame_body_id = mujoco.mj_name2id(
            self.model,
            mujoco.mjtObj.mjOBJ_BODY,
            Config.TARGET_FRAME_BODY_NAME,
        )
        self.target_frame_marker_mocap_id = self._get_mocap_id(Config.TARGET_FRAME_MARKER_BODY)
        self.locked_dof_ids = self._resolve_uncontrolled_dof_ids()
        self.locked_qpos_ids = self._resolve_uncontrolled_qpos_ids()
        self._torque_buffer = np.empty(Config.NUM_JOINTS, dtype=np.float64)
        self._command_tau = np.zeros(Config.NUM_JOINTS, dtype=np.float64)
        self._friction_tau = np.zeros(Config.NUM_JOINTS, dtype=np.float64)
        self._mass_matrix = np.zeros((self.model.nv, self.model.nv), dtype=np.float64)
        self._external_qfrc = np.zeros(self.model.nv, dtype=np.float64)
        self._arm_rhs = np.empty(Config.NUM_JOINTS, dtype=np.float64)
        self._arm_qacc = np.zeros(Config.NUM_JOINTS, dtype=np.float64)
        self._zero_pos = np.zeros(3, dtype=np.float64)
        self._unit_quat = np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float64)
        self._target_pos_body = np.zeros((Config.NUM_ARMS, 3), dtype=np.float64)
        self._target_quat = np.tile(self._unit_quat, (Config.NUM_ARMS, 1))
        self._last_synced_target_mocap_pos = np.full(
            (Config.NUM_ARMS, 3),
            np.nan,
            dtype=np.float64,
        )
        self._last_synced_target_mocap_quat = np.full(
            (Config.NUM_ARMS, 4),
            np.nan,
            dtype=np.float64,
        )
        self._target_frame_body_xmat_zero = np.eye(3, dtype=np.float64)

        # 禁用机器人连杆之间的碰撞检测，避免重力补偿时出现虚假的自碰撞接触力。
        for i in range(self.model.ngeom):
            self.model.geom_contype[i] = 0
            self.model.geom_conaffinity[i] = 0

        self.reset()
        self._target_frame_body_xmat_zero = self._get_target_frame_body_rotmat_base()
        self._sync_target_frame_markers()
        mujoco.mj_forward(self.model, self.data)

    def _resolve_joint_ids(self) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        joint_ids = []
        dof_ids = []
        qpos_ids = []
        for name in Config.JOINT_NAMES:
            jid = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, name)
            if jid < 0:
                raise ValueError(f"关节 '{name}' 在模型中未找到")
            joint_ids.append(jid)
            dof_ids.append(self.model.jnt_dofadr[jid])
            qpos_ids.append(self.model.jnt_qposadr[jid])
        return (
            np.array(joint_ids, dtype=np.int32),
            np.array(dof_ids, dtype=np.int32),
            np.array(qpos_ids, dtype=np.int32),
        )

    def _resolve_joint_limits(self) -> tuple[np.ndarray, np.ndarray]:
        joint_limits = np.asarray(Config.JOINT_LIMITS_RAD, dtype=np.float64)
        if joint_limits.shape != (Config.NUM_JOINTS, 2):
            raise ValueError(
                f"Config.JOINT_LIMITS_RAD 形状必须为 ({Config.NUM_JOINTS}, 2)"
            )

        joint_lower = joint_limits[:, 0].copy()
        joint_upper = joint_limits[:, 1].copy()
        for i, jid in enumerate(self.joint_ids):
            self.model.jnt_limited[jid] = 1
            self.model.jnt_range[jid] = [joint_lower[i], joint_upper[i]]
        return joint_lower, joint_upper

    def _resolve_body_joint_ids(self) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        joint_ids = []
        dof_ids = []
        qpos_ids = []
        for name in Config.BODY_JOINT_NAMES:
            jid = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, name)
            if jid < 0:
                continue
            joint_ids.append(jid)
            dof_ids.append(self.model.jnt_dofadr[jid])
            qpos_ids.append(self.model.jnt_qposadr[jid])
        return (
            np.array(joint_ids, dtype=np.int32),
            np.array(dof_ids, dtype=np.int32),
            np.array(qpos_ids, dtype=np.int32),
        )

    def _resolve_body_joint_limits(self) -> tuple[np.ndarray, np.ndarray]:
        limits = np.asarray(Config.BODY_JOINT_LIMITS_RAD, dtype=np.float64)
        if limits.shape != (Config.NUM_BODY_JOINTS, 2):
            raise ValueError(
                f"Config.BODY_JOINT_LIMITS_RAD 形状必须为 ({Config.NUM_BODY_JOINTS}, 2)"
            )

        if len(self.body_joint_ids) == Config.NUM_BODY_JOINTS:
            for i, jid in enumerate(self.body_joint_ids):
                self.model.jnt_limited[jid] = 1
                self.model.jnt_range[jid] = limits[i]
        return limits[:, 0].copy(), limits[:, 1].copy()

    def _apply_joint_sim_properties(self) -> None:
        self.model.dof_damping[self.dof_ids] = Config.JOINT_DAMPING
        self.model.dof_armature[self.dof_ids] = Config.JOINT_ARMATURE

    def _resolve_uncontrolled_dof_ids(self) -> np.ndarray:
        controlled = set(int(dof_id) for dof_id in self.dof_ids)
        if self._command_body_joints:
            controlled.update(int(dof_id) for dof_id in self.body_dof_ids)
        return np.array(
            [dof_id for dof_id in range(self.model.nv) if dof_id not in controlled],
            dtype=np.int32,
        )

    def _resolve_uncontrolled_qpos_ids(self) -> np.ndarray:
        controlled = set(int(qpos_id) for qpos_id in self.joint_qpos_ids)
        if self._command_body_joints:
            controlled.update(int(qpos_id) for qpos_id in self.body_qpos_ids)
        qpos_ids = []
        for jid in range(self.model.njnt):
            qpos_id = int(self.model.jnt_qposadr[jid])
            if qpos_id not in controlled:
                qpos_ids.append(qpos_id)
        return np.array(qpos_ids, dtype=np.int32)

    def _resolve_required_body_id(self, body_name: str) -> int:
        body_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, body_name)
        if body_id < 0:
            raise ValueError(f"末端 body '{body_name}' 在模型中未找到")
        return body_id

    def _get_mocap_id(self, body_name: str) -> int:
        body_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, body_name)
        if body_id < 0:
            return -1
        return self.model.body_mocapid[body_id]

    def reset(self, qpos: np.ndarray | None = None) -> None:
        """重置仿真状态"""
        mujoco.mj_resetData(self.model, self.data)
        self.set_body_qpos(Config.BODY_INIT_QPOS)
        self.set_qpos(Config.HOME_QPOS if qpos is None else qpos)
        self._lock_uncontrolled_joints()
        mujoco.mj_forward(self.model, self.data)
        self._sync_target_frame_markers()
        mujoco.mj_forward(self.model, self.data)
        self._lock_uncontrolled_joints()

    def step(self) -> None:
        """执行一步仿真"""
        self._lock_uncontrolled_joints()
        if self._command_body_joints:
            self._step_with_commanded_body_joints()
        else:
            mujoco.mj_step(self.model, self.data)
            self._lock_uncontrolled_joints()
        mujoco.mj_forward(self.model, self.data)
        self._sync_target_frame_markers()
        mujoco.mj_forward(self.model, self.data)
        self._lock_uncontrolled_joints()
        if self._command_body_joints:
            self.data.qacc[self.dof_ids] = self._arm_qacc

    def forward(self) -> None:
        """前向运动学计算（不步进时间）"""
        self._lock_uncontrolled_joints()
        mujoco.mj_forward(self.model, self.data)
        self._sync_target_frame_markers()
        mujoco.mj_forward(self.model, self.data)
        self._lock_uncontrolled_joints()

    def get_qpos(self) -> np.ndarray:
        """获取 14 轴关节角度 (rad)"""
        return self.data.qpos[self.joint_qpos_ids].copy()

    def set_qpos(self, qpos: np.ndarray) -> None:
        """设置 14 轴关节角度"""
        self.data.qpos[self.joint_qpos_ids] = qpos

    def get_qvel(self) -> np.ndarray:
        """获取 14 轴关节角速度 (rad/s)"""
        return self.data.qvel[self.dof_ids].copy()

    def set_qvel(self, qvel: np.ndarray) -> None:
        """设置 14 轴关节角速度"""
        self.data.qvel[self.dof_ids] = qvel

    def has_body_joints(self) -> bool:
        """当前 MuJoCo 模型是否保留了 3 个躯干关节。"""
        return len(self.body_qpos_ids) == Config.NUM_BODY_JOINTS

    def get_body_qpos(self) -> np.ndarray:
        """获取 3 个躯干关节角度 (rad)。"""
        if not self.has_body_joints():
            return self._body_qpos_command.copy()
        return self.data.qpos[self.body_qpos_ids].copy()

    def set_body_qpos(self, body_qpos: np.ndarray) -> None:
        """设置 3 个躯干关节的外部命令角度。"""
        body_qpos = np.asarray(body_qpos, dtype=np.float64)
        if body_qpos.shape != (Config.NUM_BODY_JOINTS,):
            raise ValueError(f"body_qpos must have shape ({Config.NUM_BODY_JOINTS},)")
        self._body_qpos_command = np.clip(
            body_qpos,
            self.body_joint_lower,
            self.body_joint_upper,
        )
        if self.has_body_joints():
            self.data.qpos[self.body_qpos_ids] = self._body_qpos_command
            self.data.qvel[self.body_dof_ids] = 0.0
            self.data.qacc[self.body_dof_ids] = 0.0
            self.data.qfrc_applied[self.body_dof_ids] = 0.0

    def set_body_joint_value(self, index: int, value: float) -> None:
        """设置单个躯干关节角度，供 GUI 滑条回调使用。"""
        body_qpos = self._body_qpos_command.copy()
        body_qpos[int(index)] = float(value)
        self.set_body_qpos(body_qpos)

    def arm_slice(self, arm: int) -> slice:
        """返回指定机械臂在 14 轴数组中的切片。"""
        start = int(arm) * Config.ARM_JOINTS
        return slice(start, start + Config.ARM_JOINTS)

    def get_arm_qpos(self, arm: int) -> np.ndarray:
        """获取指定机械臂 7 轴关节角度。"""
        return self.get_qpos()[self.arm_slice(arm)]

    def get_arm_qvel(self, arm: int) -> np.ndarray:
        """获取指定机械臂 7 轴关节角速度。"""
        return self.get_qvel()[self.arm_slice(arm)]

    def get_ee_pos(self, arm: int = Config.LEFT_ARM) -> np.ndarray:
        """获取 TCP 位置 [x, y, z] (m)"""
        return self.data.xpos[self.ee_body_ids[int(arm)]].copy()

    def get_ee_quat(self, arm: int = Config.LEFT_ARM) -> np.ndarray:
        """获取末端四元数 [w, x, y, z]（MuJoCo 格式）"""
        return self.data.xquat[self.ee_body_ids[int(arm)]].copy()

    def get_ee_rotmat(self, arm: int = Config.LEFT_ARM) -> np.ndarray:
        """获取末端旋转矩阵 (3x3)"""
        return self.data.xmat[self.ee_body_ids[int(arm)]].reshape(3, 3).copy()

    def get_all_ee_pos(self) -> np.ndarray:
        """获取左右 TCP 位置，形状为 (2, 3)。"""
        return self.data.xpos[self.ee_body_ids].copy()

    def get_all_ee_quat(self) -> np.ndarray:
        """获取左右 TCP 四元数，形状为 (2, 4)。"""
        return self.data.xquat[self.ee_body_ids].copy()

    def get_target_frame_origin_base(self) -> np.ndarray:
        """获取 Body0422 动态目标坐标系原点在 base/world 下的位置。"""
        if self.target_frame_body_id >= 0:
            return self.data.xpos[self.target_frame_body_id].copy()
        return Config.TARGET_FRAME_ORIGIN_BASE_ZERO.copy()

    def _get_target_frame_body_rotmat_base(self) -> np.ndarray:
        if self.target_frame_body_id >= 0:
            return self.data.xmat[self.target_frame_body_id].reshape(3, 3).copy()
        return np.eye(3, dtype=np.float64)

    def get_target_frame_rotmat_base(self) -> np.ndarray:
        """获取 Body0422 动态目标坐标系相对零位的旋转矩阵。"""
        if self.target_frame_body_id < 0:
            return np.eye(3, dtype=np.float64)
        return self._get_target_frame_body_rotmat_base() @ self._target_frame_body_xmat_zero.T

    def _rotmat_to_mujoco_quat(self, rotmat: np.ndarray) -> np.ndarray:
        quat = np.empty(4, dtype=np.float64)
        mujoco.mju_mat2Quat(quat, np.asarray(rotmat, dtype=np.float64).reshape(9))
        if quat[0] < 0.0:
            quat *= -1.0
        return quat

    def _quat_wxyz_to_rotmat(self, quat: np.ndarray) -> np.ndarray:
        w, x, y, z = np.asarray(quat, dtype=np.float64)
        return np.array(
            [
                [1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w), 2.0 * (x * z + y * w)],
                [2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - x * w)],
                [2.0 * (x * z - y * w), 2.0 * (y * z + x * w), 1.0 - 2.0 * (x * x + y * y)],
            ],
            dtype=np.float64,
        )

    def target_frame_to_base_quat(self, quat_body: np.ndarray) -> np.ndarray:
        """Body0422 动态目标姿态 -> base/world 四元数。"""
        quat_body = np.asarray(quat_body, dtype=np.float64)
        target_rot = self.get_target_frame_rotmat_base()
        if quat_body.shape == (4,):
            rot_world = target_rot @ self._quat_wxyz_to_rotmat(quat_body)
            return self._rotmat_to_mujoco_quat(rot_world)
        return np.asarray(
            [self._rotmat_to_mujoco_quat(target_rot @ self._quat_wxyz_to_rotmat(quat)) for quat in quat_body],
            dtype=np.float64,
        )

    def base_to_target_frame_quat(self, quat_base: np.ndarray) -> np.ndarray:
        """base/world 四元数 -> Body0422 动态目标姿态。"""
        quat_base = np.asarray(quat_base, dtype=np.float64)
        target_rot_t = self.get_target_frame_rotmat_base().T
        if quat_base.shape == (4,):
            rot_body = target_rot_t @ self._quat_wxyz_to_rotmat(quat_base)
            return self._rotmat_to_mujoco_quat(rot_body)
        return np.asarray(
            [self._rotmat_to_mujoco_quat(target_rot_t @ self._quat_wxyz_to_rotmat(quat)) for quat in quat_base],
            dtype=np.float64,
        )

    def target_frame_to_base_pos(self, pos_body: np.ndarray) -> np.ndarray:
        """Body0422 动态目标坐标 -> base/world 坐标。"""
        pos_body = np.asarray(pos_body, dtype=np.float64)
        return self.get_target_frame_origin_base() + pos_body @ self.get_target_frame_rotmat_base().T

    def base_to_target_frame_pos(self, pos_base: np.ndarray) -> np.ndarray:
        """base/world 坐标 -> Body0422 动态目标坐标。"""
        pos_base = np.asarray(pos_base, dtype=np.float64)
        return (pos_base - self.get_target_frame_origin_base()) @ self.get_target_frame_rotmat_base()

    def get_state_snapshot(
        self,
    ) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
        """一次性取出 UDP / Rerun 所需状态，减少热路径上的重复函数往返。"""
        pos_desired, quat_desired = self.get_all_target_poses()
        pos_desired_base = self.target_frame_to_base_pos(pos_desired)
        quat_desired_base = self.target_frame_to_base_quat(quat_desired)
        return (
            self.data.qpos[self.joint_qpos_ids].copy(),
            self.data.qvel[self.dof_ids].copy(),
            self.data.xpos[self.ee_body_ids].copy(),
            self.data.xquat[self.ee_body_ids].copy(),
            pos_desired_base,
            quat_desired_base,
        )

    def write_state_packet(self, state_packet: np.ndarray) -> None:
        """直接将当前反馈和 Body0422 目标坐标写入预分配 C 控制输入包。"""
        if state_packet.shape[0] != CONTROL_INPUT_PACKET_SIZE:
            raise ValueError(
                f"control input packet must contain {CONTROL_INPUT_PACKET_SIZE} doubles"
            )
        target_pos, target_quat = self.get_all_target_poses()

        fill_control_input_packet(
            state_packet,
            self.data.qpos[self.joint_qpos_ids],
            self.data.qvel[self.dof_ids],
            self.get_body_qpos(),
            target_pos[Config.LEFT_ARM],
            target_quat[Config.LEFT_ARM],
            target_pos[Config.RIGHT_ARM],
            target_quat[Config.RIGHT_ARM],
        )

    def get_jacobian(self, arm: int = Config.LEFT_ARM) -> tuple[np.ndarray, np.ndarray]:
        """
        计算 TCP 处的几何雅可比矩阵
        返回: (jacp, jacr) 各为 (3, nv) 的矩阵
        """
        jacp = np.zeros((3, self.model.nv))
        jacr = np.zeros((3, self.model.nv))
        mujoco.mj_jacBody(self.model, self.data, jacp, jacr, int(self.ee_body_ids[int(arm)]))
        return jacp, jacr

    def set_target_pose(
        self,
        pos: np.ndarray,
        quat: np.ndarray | None = None,
        arm: int = Config.LEFT_ARM,
    ) -> None:
        """设置目标位姿；位置使用 Body0422 动态目标坐标系。"""
        arm_index = int(arm)
        self._target_pos_body[arm_index] = pos
        if quat is not None:
            self._target_quat[arm_index] = quat
        self._sync_target_frame_markers()
        mujoco.mj_forward(self.model, self.data)

    def set_target_pose_base(
        self,
        pos: np.ndarray,
        quat: np.ndarray | None = None,
        arm: int = Config.LEFT_ARM,
    ) -> None:
        """用 base/world 位姿设置目标，内部转换为 Body0422 动态目标坐标。"""
        target_quat = None if quat is None else self.base_to_target_frame_quat(quat)
        self.set_target_pose(self.base_to_target_frame_pos(pos), target_quat, arm=arm)

    def set_all_target_poses(
        self,
        pos: np.ndarray,
        quat: np.ndarray | None = None,
    ) -> None:
        """设置左右目标可视化物体的位姿。"""
        for arm in range(Config.NUM_ARMS):
            arm_quat = None if quat is None else quat[arm]
            self.set_target_pose(pos[arm], arm_quat, arm=arm)

    def set_all_target_poses_base(
        self,
        pos: np.ndarray,
        quat: np.ndarray | None = None,
    ) -> None:
        """用 base/world 位置设置左右目标，内部转换为 Body0422 动态目标坐标。"""
        for arm in range(Config.NUM_ARMS):
            arm_quat = None if quat is None else quat[arm]
            self.set_target_pose_base(pos[arm], arm_quat, arm=arm)

    def get_target_pose(self, arm: int = Config.LEFT_ARM) -> tuple[np.ndarray, np.ndarray]:
        """获取目标位姿；位置使用 Body0422 动态目标坐标系。"""
        arm_index = int(arm)
        return self._target_pos_body[arm_index].copy(), self._target_quat[arm_index].copy()

    def get_all_target_poses(self) -> tuple[np.ndarray, np.ndarray]:
        """获取左右目标位姿；位置使用 Body0422 动态目标坐标系。"""
        return self._target_pos_body.copy(), self._target_quat.copy()

    def get_all_target_poses_base(self) -> tuple[np.ndarray, np.ndarray]:
        """获取左右目标位姿；位置转换到 base/world 坐标用于显示。"""
        return (
            self.target_frame_to_base_pos(self._target_pos_body),
            self.target_frame_to_base_quat(self._target_quat),
        )

    def _sync_target_frame_markers(self) -> None:
        """把 Body0422 坐标目标同步到 MuJoCo mocap 显示对象。"""
        origin = self.get_target_frame_origin_base()
        target_rot = self.get_target_frame_rotmat_base()
        target_frame_quat = self._rotmat_to_mujoco_quat(target_rot)
        if self.target_frame_marker_mocap_id >= 0:
            self.data.mocap_pos[self.target_frame_marker_mocap_id] = origin
            self.data.mocap_quat[self.target_frame_marker_mocap_id] = target_frame_quat

        for arm in range(Config.NUM_ARMS):
            mocap_id = int(self.target_mocap_ids[arm])
            if mocap_id < 0:
                continue
            current_mocap_pos = self.data.mocap_pos[mocap_id].copy()
            current_mocap_quat = self.data.mocap_quat[mocap_id].copy()
            last_synced_pos = self._last_synced_target_mocap_pos[arm]
            last_synced_quat = self._last_synced_target_mocap_quat[arm]
            if np.all(np.isfinite(last_synced_pos)) and not np.allclose(
                current_mocap_pos,
                last_synced_pos,
                atol=1e-10,
                rtol=0.0,
            ):
                self._target_pos_body[arm] = target_rot.T @ (current_mocap_pos - origin)
            if np.all(np.isfinite(last_synced_quat)) and not np.allclose(
                current_mocap_quat,
                last_synced_quat,
                atol=1e-10,
                rtol=0.0,
            ):
                self._target_quat[arm] = self.base_to_target_frame_quat(current_mocap_quat)

        target_base = self.target_frame_to_base_pos(self._target_pos_body)
        target_quat_base = self.target_frame_to_base_quat(self._target_quat)
        for arm in range(Config.NUM_ARMS):
            mocap_id = int(self.target_mocap_ids[arm])
            if mocap_id < 0:
                continue
            self.data.mocap_pos[mocap_id] = target_base[arm]
            self.data.mocap_quat[mocap_id] = target_quat_base[arm]
            self._last_synced_target_mocap_pos[arm] = target_base[arm]
            self._last_synced_target_mocap_quat[arm] = target_quat_base[arm]

    def get_jacobian_7dof(self, arm: int = Config.LEFT_ARM) -> np.ndarray:
        """
        获取 6x7 几何雅可比矩阵（仅 7 个关节自由度）
        返回: J (6, 7)，前 3 行为线速度，后 3 行为角速度
        """
        jacp, jacr = self.get_jacobian(arm)
        arm_dof_ids = self.dof_ids[self.arm_slice(arm)]
        return np.vstack([jacp[:, arm_dof_ids], jacr[:, arm_dof_ids]])

    def get_qfrc_bias(self) -> np.ndarray:
        """
        获取 MuJoCo 计算的 qfrc_bias（重力 + Coriolis）
        返回: tau (14,)
        """
        return self.data.qfrc_bias[self.dof_ids].copy()

    def apply_torque(self, tau: np.ndarray) -> None:
        """
        施加关节力矩
        tau: (14,) 各关节力矩 (N·m)
        """
        self._command_tau[:] = np.asarray(tau, dtype=np.float64)
        self._friction_tau[:] = self.get_follower_friction_torque()
        self.data.qfrc_applied[self.dof_ids] = self._command_tau - self._friction_tau

    def get_follower_friction_torque(self, qvel: np.ndarray | None = None) -> np.ndarray:
        """计算 OpenArm Follower 七轴 tanh 摩擦模型，左右臂各复制一份。"""
        if not Config.ENABLE_FOLLOWER_FRICTION:
            return np.zeros(Config.NUM_JOINTS, dtype=np.float64)

        if qvel is None:
            qvel = self.get_qvel()
        qvel = np.asarray(qvel, dtype=np.float64)
        if qvel.shape != (Config.NUM_JOINTS,):
            raise ValueError(f"qvel must have shape ({Config.NUM_JOINTS},)")

        return (
            Config.FOLLOWER_FRICTION_FO_14
            + Config.FOLLOWER_FRICTION_FV_14 * qvel
            + Config.FOLLOWER_FRICTION_FC_14
            * np.tanh(0.1 * Config.FOLLOWER_FRICTION_K_14 * qvel)
        )

    def get_applied_friction_torque(self) -> np.ndarray:
        """返回最近一次 apply_torque() 计算出的 14 轴物理摩擦力矩。"""
        return self._friction_tau.copy()

    def _step_with_commanded_body_joints(self) -> None:
        """在躯干关节外部锁定时，只对 14 个双臂自由度做动力学积分。"""
        self.set_body_qpos(self._body_qpos_command)
        mujoco.mj_forward(self.model, self.data)
        mujoco.mj_fullM(self.model, self._mass_matrix, self.data.qM)
        self._external_qfrc.fill(0.0)
        for body_id in np.nonzero(np.any(self.data.xfrc_applied != 0.0, axis=1))[0]:
            mujoco.mj_applyFT(
                self.model,
                self.data,
                self.data.xfrc_applied[body_id, :3],
                self.data.xfrc_applied[body_id, 3:],
                self.data.xipos[body_id],
                int(body_id),
                self._external_qfrc,
            )

        arm_mass = self._mass_matrix[np.ix_(self.dof_ids, self.dof_ids)]
        self._arm_rhs[:] = (
            self._command_tau
            + self._external_qfrc[self.dof_ids]
            - self._friction_tau
            - self.data.qfrc_bias[self.dof_ids]
        )
        self._arm_qacc[:] = np.linalg.solve(arm_mass, self._arm_rhs)
        self.data.qacc[self.dof_ids] = self._arm_qacc

        dt = float(self.model.opt.timestep)
        self.data.qvel[self.dof_ids] += self._arm_qacc * dt
        self.data.qpos[self.joint_qpos_ids] += self.data.qvel[self.dof_ids] * dt
        self.data.time += dt
        self.set_body_qpos(self._body_qpos_command)

    def _lock_uncontrolled_joints(self) -> None:
        """锁住 AM-DPBSURDF0422 中未接入 14 轴控制器的自由度。"""
        self.set_body_qpos(self._body_qpos_command)
        if len(self.locked_qpos_ids):
            self.data.qpos[self.locked_qpos_ids] = 0.0
        if len(self.locked_dof_ids):
            self.data.qvel[self.locked_dof_ids] = 0.0
            self.data.qacc[self.locked_dof_ids] = 0.0
            self.data.qfrc_applied[self.locked_dof_ids] = 0.0

    def clip_torque(self, tau: np.ndarray) -> np.ndarray:
        """限幅力矩到关节力矩限制范围"""
        np.clip(tau, -Config.TORQUE_LIMITS, Config.TORQUE_LIMITS, out=self._torque_buffer)
        return self._torque_buffer

    def clip_qpos(self, qpos: np.ndarray) -> np.ndarray:
        """将关节角裁剪到 URDF 定义的角度限位范围内"""
        return np.clip(qpos, self.joint_lower, self.joint_upper)

    def enforce_joint_limits(self) -> bool:
        """
        强制执行关节角度限位：
        1. 将越界的 qpos 裁剪回限位范围
        2. 将越界关节的 qvel 清零（防止速度在下一步把位置推回越界）
        """
        q = self.data.qpos[self.joint_qpos_ids]
        q_clipped = np.clip(q, self.joint_lower, self.joint_upper)
        violated = q != q_clipped
        if not np.any(violated):
            return False

        self.data.qpos[self.joint_qpos_ids] = q_clipped
        qd = self.data.qvel[self.dof_ids]
        qd[violated] = 0.0
        self.data.qvel[self.dof_ids] = qd
        return True

    def get_time(self) -> float:
        """获取当前仿真时间 (秒)"""
        return self.data.time


def check_mujoco_version():
    """检查 MuJoCo 版本"""
    version = mujoco.__version__
    print(f"MuJoCo 版本: {version}")
    major, minor = map(int, version.split(".")[:2])
    if major < 3:
        raise RuntimeError(f"需要 MuJoCo >= 3.0，当前版本 {version}")
    return version
