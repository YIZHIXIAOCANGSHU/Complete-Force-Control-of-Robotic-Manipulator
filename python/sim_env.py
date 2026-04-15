"""
MuJoCo 仿真环境封装
直接加载 URDF，通过 qfrc_applied 施加关节力矩
"""

from __future__ import annotations

import os

import mujoco
import numpy as np

from config import Config
from sim_scene import build_enhanced_model


class MujocoSimEnv:
    """AM-D02 七轴机械臂 MuJoCo 仿真环境"""

    def __init__(self, urdf_path: str | None = None):
        if urdf_path is None:
            urdf_path = Config.URDF_PATH

        urdf_dir = os.path.dirname(os.path.abspath(urdf_path))
        urdf_filename = os.path.basename(urdf_path)

        old_cwd = os.getcwd()
        try:
            os.chdir(urdf_dir)
            self.model = build_enhanced_model(urdf_filename, Config.TCP_OFFSET)
        finally:
            os.chdir(old_cwd)

        self.data = mujoco.MjData(self.model)
        self.model.opt.timestep = Config.DT

        self.joint_ids, self.dof_ids = self._resolve_joint_ids()
        self.joint_lower, self.joint_upper = self._resolve_joint_limits()
        self.ee_body_id = self._resolve_required_body_id(Config.END_EFFECTOR_BODY)
        self.target_mocap_id = self._get_mocap_id("target_pose")
        self.reported_mocap_id = self._get_mocap_id("reported_pose")
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
        joint_lower = np.full(Config.NUM_JOINTS, -np.inf, dtype=np.float64)
        joint_upper = np.full(Config.NUM_JOINTS, np.inf, dtype=np.float64)
        for i, jid in enumerate(self.joint_ids):
            if not self.model.jnt_limited[jid]:
                continue
            lo, hi = self.model.jnt_range[jid]
            joint_lower[i] = min(lo, hi)
            joint_upper[i] = max(lo, hi)
        return joint_lower, joint_upper

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
        mujoco.mj_forward(self.model, self.data)

    def step(self) -> None:
        """执行一步仿真"""
        mujoco.mj_step(self.model, self.data)

    def forward(self) -> None:
        """前向运动学计算（不步进时间）"""
        mujoco.mj_forward(self.model, self.data)

    def get_qpos(self) -> np.ndarray:
        """获取 7 轴关节角度 (rad)"""
        return self.data.qpos[self.dof_ids].copy()

    def set_qpos(self, qpos: np.ndarray) -> None:
        """设置 7 轴关节角度"""
        self.data.qpos[self.dof_ids] = qpos

    def get_qvel(self) -> np.ndarray:
        """获取 7 轴关节角速度 (rad/s)"""
        return self.data.qvel[self.dof_ids].copy()

    def set_qvel(self, qvel: np.ndarray) -> None:
        """设置 7 轴关节角速度"""
        self.data.qvel[self.dof_ids] = qvel

    def get_ee_pos(self) -> np.ndarray:
        """获取 TCP 位置 [x, y, z] (m)"""
        return self.data.xpos[self.ee_body_id].copy()

    def get_ee_quat(self) -> np.ndarray:
        """获取末端四元数 [w, x, y, z]（MuJoCo 格式）"""
        return self.data.xquat[self.ee_body_id].copy()

    def get_ee_rotmat(self) -> np.ndarray:
        """获取末端旋转矩阵 (3x3)"""
        return self.data.xmat[self.ee_body_id].reshape(3, 3).copy()

    def get_state_snapshot(
        self,
    ) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
        """一次性取出 UDP / Rerun 所需状态，减少热路径上的重复函数往返。"""
        pos_desired, quat_desired = self.get_target_pose()
        return (
            self.data.qpos[self.dof_ids].copy(),
            self.data.qvel[self.dof_ids].copy(),
            self.data.xpos[self.ee_body_id].copy(),
            self.data.xquat[self.ee_body_id].copy(),
            pos_desired,
            quat_desired,
        )

    def write_state_packet(self, state_packet: np.ndarray) -> None:
        """直接将当前状态写入预分配 UDP 包，避免中间数组拼装。"""
        state_packet[0:7] = self.data.qpos[self.dof_ids]
        state_packet[7:14] = self.data.qvel[self.dof_ids]
        state_packet[14:17] = self.data.xpos[self.ee_body_id]
        state_packet[17:21] = self.data.xquat[self.ee_body_id]
        if self.target_mocap_id >= 0:
            state_packet[21:24] = self.data.mocap_pos[self.target_mocap_id]
            state_packet[24:28] = self.data.mocap_quat[self.target_mocap_id]
        else:
            state_packet[21:24] = self._zero_pos
            state_packet[24:28] = self._unit_quat

    def get_jacobian(self) -> tuple[np.ndarray, np.ndarray]:
        """
        计算 TCP 处的几何雅可比矩阵
        返回: (jacp, jacr) 各为 (3, nv) 的矩阵
        """
        jacp = np.zeros((3, self.model.nv))
        jacr = np.zeros((3, self.model.nv))
        mujoco.mj_jacBody(self.model, self.data, jacp, jacr, self.ee_body_id)
        return jacp, jacr

    def set_target_pose(self, pos: np.ndarray, quat: np.ndarray | None = None) -> None:
        """设置目标可视化物体的位姿 (mocap body)"""
        if self.target_mocap_id < 0:
            return
        self.data.mocap_pos[self.target_mocap_id] = pos
        if quat is not None:
            self.data.mocap_quat[self.target_mocap_id] = quat

    def get_target_pose(self) -> tuple[np.ndarray, np.ndarray]:
        """获取目标可视化物体的当前位姿 (mocap body)"""
        if self.target_mocap_id >= 0:
            return (
                self.data.mocap_pos[self.target_mocap_id].copy(),
                self.data.mocap_quat[self.target_mocap_id].copy(),
            )
        return self._zero_pos.copy(), self._unit_quat.copy()

    def get_jacobian_7dof(self) -> np.ndarray:
        """
        获取 6x7 几何雅可比矩阵（仅 7 个关节自由度）
        返回: J (6, 7)，前 3 行为线速度，后 3 行为角速度
        """
        jacp, jacr = self.get_jacobian()
        return np.vstack([jacp[:, self.dof_ids], jacr[:, self.dof_ids]])

    def get_qfrc_bias(self) -> np.ndarray:
        """
        获取 MuJoCo 计算的 qfrc_bias（重力 + Coriolis）
        返回: tau (7,)
        """
        return self.data.qfrc_bias[self.dof_ids].copy()

    def apply_torque(self, tau: np.ndarray) -> None:
        """
        施加关节力矩
        tau: (7,) 各关节力矩 (N·m)
        """
        self.data.qfrc_applied[self.dof_ids] = tau

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
