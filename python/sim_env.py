"""MuJoCo 仿真环境封装。"""

from __future__ import annotations

import os

import mujoco
import numpy as np

from config import Config
from sim_scene import build_enhanced_model
from state_packets import CONTROL_INPUT_PACKET_SIZE, fill_control_input_packet


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
            self.model = build_enhanced_model(
                urdf_filename,
                Config.TCP_OFFSETS,
                Config.TCP_FRAME_QUATS,
                controlled_joint_names=Config.JOINT_NAMES,
                fix_uncontrolled_joints=Config.FIX_UNCONTROLLED_JOINTS,
            )
        finally:
            os.chdir(old_cwd)

        self.data = mujoco.MjData(self.model)
        self.model.opt.timestep = Config.DT

        self.joint_ids, self.dof_ids = self._resolve_joint_ids()
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
        self.locked_dof_ids = self._resolve_uncontrolled_dof_ids()
        self.locked_qpos_ids = self._resolve_uncontrolled_qpos_ids()
        self._torque_buffer = np.empty(Config.NUM_JOINTS, dtype=np.float64)
        self._zero_pos = np.zeros(3, dtype=np.float64)
        self._unit_quat = np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float64)

        # 禁用机器人连杆之间的碰撞检测，避免重力补偿时出现虚假的自碰撞接触力。
        for i in range(self.model.ngeom):
            self.model.geom_contype[i] = 0
            self.model.geom_conaffinity[i] = 0

        self.reset()

    def _resolve_joint_ids(self) -> tuple[np.ndarray, np.ndarray]:
        joint_ids = []
        dof_ids = []
        for name in Config.JOINT_NAMES:
            jid = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, name)
            if jid < 0:
                raise ValueError(f"关节 '{name}' 在模型中未找到")
            joint_ids.append(jid)
            dof_ids.append(self.model.jnt_dofadr[jid])
        return np.array(joint_ids, dtype=np.int32), np.array(dof_ids, dtype=np.int32)

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

    def _apply_joint_sim_properties(self) -> None:
        self.model.dof_damping[self.dof_ids] = Config.JOINT_DAMPING
        self.model.dof_armature[self.dof_ids] = Config.JOINT_ARMATURE

    def _resolve_uncontrolled_dof_ids(self) -> np.ndarray:
        controlled = set(int(dof_id) for dof_id in self.dof_ids)
        return np.array(
            [dof_id for dof_id in range(self.model.nv) if dof_id not in controlled],
            dtype=np.int32,
        )

    def _resolve_uncontrolled_qpos_ids(self) -> np.ndarray:
        controlled = set(int(self.model.jnt_qposadr[jid]) for jid in self.joint_ids)
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
        self.set_qpos(Config.HOME_QPOS if qpos is None else qpos)
        self._lock_uncontrolled_joints()
        mujoco.mj_forward(self.model, self.data)

    def step(self) -> None:
        """执行一步仿真"""
        self._lock_uncontrolled_joints()
        mujoco.mj_step(self.model, self.data)
        self._lock_uncontrolled_joints()

    def forward(self) -> None:
        """前向运动学计算（不步进时间）"""
        mujoco.mj_forward(self.model, self.data)

    def get_qpos(self) -> np.ndarray:
        """获取 14 轴关节角度 (rad)"""
        return self.data.qpos[self.dof_ids].copy()

    def set_qpos(self, qpos: np.ndarray) -> None:
        """设置 14 轴关节角度"""
        self.data.qpos[self.dof_ids] = qpos

    def get_qvel(self) -> np.ndarray:
        """获取 14 轴关节角速度 (rad/s)"""
        return self.data.qvel[self.dof_ids].copy()

    def set_qvel(self, qvel: np.ndarray) -> None:
        """设置 14 轴关节角速度"""
        self.data.qvel[self.dof_ids] = qvel

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

    def get_state_snapshot(
        self,
    ) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
        """一次性取出 UDP / Rerun 所需状态，减少热路径上的重复函数往返。"""
        pos_desired, quat_desired = self.get_all_target_poses()
        return (
            self.data.qpos[self.dof_ids].copy(),
            self.data.qvel[self.dof_ids].copy(),
            self.data.xpos[self.ee_body_ids].copy(),
            self.data.xquat[self.ee_body_ids].copy(),
            pos_desired,
            quat_desired,
        )

    def write_state_packet(self, state_packet: np.ndarray) -> None:
        """直接将当前反馈和 base_link 目标写入预分配 C 控制输入包。"""
        if state_packet.shape[0] != CONTROL_INPUT_PACKET_SIZE:
            raise ValueError(
                f"control input packet must contain {CONTROL_INPUT_PACKET_SIZE} doubles"
            )
        target_pos, target_quat = self.get_all_target_poses()

        fill_control_input_packet(
            state_packet,
            self.data.qpos[self.dof_ids],
            self.data.qvel[self.dof_ids],
            target_pos[Config.LEFT_ARM],
            target_quat[Config.LEFT_ARM],
            target_pos[Config.RIGHT_ARM],
            target_quat[Config.RIGHT_ARM],
            Config.DT,
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
        """设置目标可视化物体的位姿 (mocap body)"""
        mocap_id = int(self.target_mocap_ids[int(arm)])
        if mocap_id < 0:
            return
        self.data.mocap_pos[mocap_id] = pos
        if quat is not None:
            self.data.mocap_quat[mocap_id] = quat

    def set_all_target_poses(
        self,
        pos: np.ndarray,
        quat: np.ndarray | None = None,
    ) -> None:
        """设置左右目标可视化物体的位姿。"""
        for arm in range(Config.NUM_ARMS):
            arm_quat = None if quat is None else quat[arm]
            self.set_target_pose(pos[arm], arm_quat, arm=arm)

    def get_target_pose(self, arm: int = Config.LEFT_ARM) -> tuple[np.ndarray, np.ndarray]:
        """获取目标可视化物体的当前位姿 (mocap body)"""
        mocap_id = int(self.target_mocap_ids[int(arm)])
        if mocap_id >= 0:
            return (
                self.data.mocap_pos[mocap_id].copy(),
                self.data.mocap_quat[mocap_id].copy(),
            )
        return self._zero_pos.copy(), self._unit_quat.copy()

    def get_all_target_poses(self) -> tuple[np.ndarray, np.ndarray]:
        """获取左右目标可视化物体的当前位姿。"""
        pos = np.zeros((Config.NUM_ARMS, 3), dtype=np.float64)
        quat = np.zeros((Config.NUM_ARMS, 4), dtype=np.float64)
        for arm in range(Config.NUM_ARMS):
            pos[arm], quat[arm] = self.get_target_pose(arm)
        return pos, quat

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
        self.data.qfrc_applied[self.dof_ids] = tau

    def _lock_uncontrolled_joints(self) -> None:
        """锁住 AM-DPBSURDF0422 中未接入 14 轴控制器的自由度。"""
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
        q = self.data.qpos[self.dof_ids]
        q_clipped = np.clip(q, self.joint_lower, self.joint_upper)
        violated = q != q_clipped
        if not np.any(violated):
            return False

        self.data.qpos[self.dof_ids] = q_clipped
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
