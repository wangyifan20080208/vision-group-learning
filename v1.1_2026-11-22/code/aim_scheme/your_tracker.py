# custom/schemes/your_scheme/your_tracker.py
"""跟踪器：一帧观测 → 一帧预测 + 云台指令。

第一版目标：让云台指向"离得最近的那块板"，并允许开火。

后面可以在这里逐步加：
    ① 选板：按朝向/距离/在画面中的位置挑最该打的一块
    ② 滤波：EKF 把离散观测串成连续估计
    ③ 提前量：按弹丸飞行时间 + 系统延迟前推
    ④ 弹道：补偿重力下坠（实测 5m 处落 20cm，10m 处落 78cm）
"""
from __future__ import annotations

import math
from typing import List

import numpy as np

from config import cfg
from core.bus import public_bus
from core.bus.dataflow import Prediction, ShotSolution
from core.bus.messages import PublicMessages
from core.entities.property.entity_types import EntityType, TargetType
from core.interfaces.tracker import Tracker, register_tracker


@register_tracker("yours")
class YourTracker(Tracker):
    description = "我的方案：挑最近的板 + 重力补偿"

    def __init__(self):
        self.frame = str(cfg.sim.frame).upper()
        # 17mm 弹丸初速（规则上限 25 m/s，实际按配置取）
        self.v0 = 25.0
        self.g = 9.8
        self.last_target = None

    # ---------- 工具 ----------
    def _to_gimbal(self, pos, self_state):
        """把观测位置统一成「相对云台中心」的向量。"""
        p = np.asarray(pos, float).reshape(3)
        if self.frame == "W":
            p = p - np.asarray(self_state.gimbal.pos, float)
        return p

    def _pick(self, obs, self_state):
        """选板：优先近的，其次选朝向自己的（法线点积大）。"""
        best = None
        for a in obs.obs_armors:
            d = self._to_gimbal(a.rel_pos, self_state)
            rng = float(np.linalg.norm(d))
            if rng < 0.3 or rng > 20.0:
                continue
            yaw_face = math.atan2(d[1], d[0])
            psi = float(a.rel_rpy[2])
            # 板法线与「板→我」方向的夹角，越接近 0 越正对
            facing = math.cos(psi - (yaw_face + math.pi))
            score = rng - 2.0 * facing
            if best is None or score < best[0]:
                best = (score, a, d, rng)
        return best

    def _ballistic(self, d, rng):
        """重力补偿：迭代求抬升角。"""
        if rng < 1e-3:
            return 0.0
        t = rng / self.v0
        for _ in range(3):
            drop = 0.5 * self.g * t * t
            pitch_up = math.atan2(d[2] + drop, math.hypot(d[0], d[1]))
            t = rng / (self.v0 * max(0.2, math.cos(pitch_up)))
        return pitch_up

    # ---------- 主入口 ----------
    def step(self, obs, self_state) -> List[Prediction]:
        if obs is None or not obs.obs_armors:
            self.last_target = None
            return []

        picked = self._pick(obs, self_state)
        if picked is None:
            return []
        _score, armor, d, rng = picked

        # 目标类型
        et = armor.entity_type
        if et == EntityType.Rune:
            tt = TargetType.RUNE
        elif et == EntityType.Outpost:
            tt = TargetType.OUTPOST
        else:
            tt = TargetType.ROBOT

        # 预测位置：这一版先直接用观测（后续加 EKF + 提前量）
        pos = np.asarray(armor.rel_pos, float).copy()

        pred = Prediction(
            target_type=tt,
            center=pos,
            armors=[pos],
            timestamp=obs.timestamp,
            is_tracking=True,
        )

        # 云台角度（G 系下 d 已经是相对云台中心）
        yaw = math.atan2(d[1], d[0])
        pitch_up = self._ballistic(d, rng)

        # 角速度前馈（简单差分）
        yaw_rate = pitch_rate = 0.0
        if self.last_target is not None:
            pass  # 第一版先不做前馈
        self.last_target = (yaw, pitch_up, obs.timestamp)

        # 只在角度误差小的时候允许开火（平台还会做物理检查）
        gimbal_rpy = np.asarray(self_state.gimbal.rpy, float)
        err_yaw = abs((yaw - float(gimbal_rpy[2]) + math.pi) % (2 * math.pi) - math.pi)
        err_pitch = abs(pitch_up - (-float(gimbal_rpy[1])))
        can_fire = err_yaw < math.radians(2.0) and err_pitch < math.radians(2.0)

        solution = ShotSolution(
            yaw=float(yaw),
            pitch=float(pitch_up),
            yaw_rate=float(yaw_rate),
            pitch_rate=float(pitch_rate),
            fire=bool(can_fire),
            fire_mode="auto",
            timestamp=obs.timestamp,
        )
        public_bus().publish(PublicMessages.CONTROL, solution)

        return [pred]
