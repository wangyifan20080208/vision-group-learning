# custom/schemes/your_scheme/your_pose_solver.py
"""位姿解算：一帧 2D 检测 → 一帧三维观测。

用平台提供的 IPPE 单板解算器（custom/components/armor_solvers/ippe.py），
它是"单应分解 + 高斯牛顿精化"的全 6 自由度 PnP。

关键坐标系（见 docs/coordinate_systems.md §5）：
    像素 --PNP--> C'（相机绝对方向系，原点在相机）
    C' --加相机相对云台的平移--> G 系（原点在云台中心）
    G  --加云台世界位置--> W 系
"""
from __future__ import annotations

import math

import numpy as np

from config import cfg
from core.algorithms.math.transform import (
    abs_camera_to_gimbal,
    gimbal_to_world,
    rpy_to_R,
)
from core.bus.dataflow import ArmorObservation, Observation
from core.interfaces.pose_solver import PoseSolver, register_pose_solver
from custom.components.armor_solvers.ippe import IppeSolver


@register_pose_solver("yours")
class YourPoseSolver(PoseSolver):
    description = "我的方案：IPPE 单板 PnP + 坐标变换"

    def __init__(self):
        self.solver = IppeSolver()
        self.frame = str(cfg.sim.frame).upper()
        # 装甲板安装俯仰：车辆装甲朝上抬 15°（配置里 FLU 抬头为负）
        self.robot_tilt = math.radians(float(cfg.robot.armor_tilt))
        try:
            self.outpost_tilt = math.radians(float(cfg.outpost.armor_tilt))
        except Exception:
            self.outpost_tilt = self.robot_tilt

    def solve(self, frame, self_state) -> Observation:
        out = []
        if frame is None or not getattr(frame, "detections", None):
            return Observation([], float(getattr(frame, "timestamp", 0.0)))

        K = np.asarray(self_state.camera.K, float)
        M = rpy_to_R(self_state.camera.rpy)          # C' 相对世界方向的姿态
        cam_in_gimbal = (np.asarray(self_state.camera.pos, float)
                         - np.asarray(self_state.gimbal.pos, float))
        gimbal_world = np.asarray(self_state.gimbal.pos, float)

        for det in frame.detections:
            if det.image_points_2d is None:
                continue
            name = str(getattr(det.entity_type, "name", "")).lower()
            tilt = self.outpost_tilt if name == "outpost" else self.robot_tilt
            pos_c, rpy_c = self.solver.estimate(
                camera_k=K,
                armor_size=det.armor_size,
                image_points_2d=det.image_points_2d,
                rot_camera_to_abs_camera=M,
                mount_pitch=tilt,
            )
            if pos_c is None:
                continue

            # C' → G
            pos_g, R_abs = abs_camera_to_gimbal(
                pos_c, M, camera_pos_gimbal=cam_in_gimbal)
            if self.frame == "W":
                pos_out, _ = gimbal_to_world(pos_g, R_abs,
                                             gimbal_pos_world=gimbal_world)
            else:
                pos_out = pos_g

            out.append(ArmorObservation(
                entity_type=det.entity_type,
                rel_pos=np.asarray(pos_out, float),
                rel_rpy=np.asarray(rpy_c, float),
                armor_size=det.armor_size,
                image_points_2d=det.image_points_2d,
                lit=bool(det.lit),
            ))

        return Observation(out, float(frame.timestamp))
