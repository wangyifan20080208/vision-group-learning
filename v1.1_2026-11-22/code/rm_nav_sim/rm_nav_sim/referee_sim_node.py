#!/usr/bin/env python3
"""裁判系统模拟节点。

真实机器人上，血量/弹量/热量/比赛阶段来自裁判系统串口（115200 8N1）：
  0x0001  比赛状态（阶段、剩余时间）      1 Hz
  0x0003  双方机器人血量                  1 Hz
本节点在仿真里复现这些数据，让决策层「不用改代码就能上真车」。

发布（JSON 放在 std_msgs/String，便于 ros2 topic echo 直接看）：
  /referee/state
订阅：
  /ground_truth/pose    机器人位置（仿真真值）
  /referee/damage       外部注入伤害，用于测试
  /sentry/fire          开火事件（Bool），扣弹量、加热量
"""
import json
import math

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from std_msgs.msg import Bool, Float32, String


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y),
                      1 - 2 * (q.y * q.y + q.z * q.z))


class RefereeSim(Node):
    def __init__(self):
        super().__init__("referee_sim")
        d = [
            ("hp_max", 400),              # 哨兵上限血量（规则表 3-5）
            ("ammo_max", 750),            # 初始允许发弹量
            ("heat_max", 260),            # 射击热量上限
            ("heat_cooling", 30.0),       # 热量冷却 /秒
            ("heat_per_shot", 10.0),      # 每发产生的热量
            ("match_duration", 300.0),    # 比赛时长 5 分钟
            ("heal_rate_frac", 0.25),     # 补给区回血 25%上限/秒
            ("damage_per_sec", 0.0),      # 持续受击速率（模拟被对枪）
            ("damage_only_zone", "center"),   # 只有在该区域内才受击（""=全场）
            ("death_penalty_control", 10.0),   # 战亡扣控制度
            ("respawn_base_ticks", 5.0),  # 首次复活读条
            ("pub_hz", 10.0),
            # 区域划分（map 系，来自 CAD 解析）
            ("zones_json", ""),
        ]
        for n, v in d:
            self.declare_parameter(n, v)
        p = self.get_parameter

        self.hp = float(p("hp_max").value)
        self.hp_max = float(p("hp_max").value)
        self.ammo = float(p("ammo_max").value)
        self.heat = 0.0
        self.heat_max = float(p("heat_max").value)
        self.cooling = float(p("heat_cooling").value)
        self.heat_shot = float(p("heat_per_shot").value)
        self.duration = float(p("match_duration").value)
        self.heal_frac = float(p("heal_rate_frac").value)
        self.dps = float(p("damage_per_sec").value)
        self.dmg_zone = str(p("damage_only_zone").value)
        self.death_pen = float(p("death_penalty_control").value)
        self.respawn_ticks_base = float(p("respawn_base_ticks").value)
        hz = float(p("pub_hz").value)

        self.elapsed = 0.0
        self.stage = 4                 # 4 = 比赛中
        self.alive = True
        self.weak = False
        self.invincible = False
        self.invincible_time = 0.0
        self.invincible_max = 30.0     # 规则：复活后无敌 30 秒
        self.respawn_ticks = 0.0
        self.deaths = 0
        self.control_red = 600.0
        self.control_blue = 600.0
        self.zone = "none"
        self.pose = None
        self.last_t = self.get_clock().now()

        # 控制区/补给区（与 control_zones.json 一致）
        zj = p("zones_json").value
        if zj:
            self.zones = json.loads(zj)
        else:
            self.zones = {
                "red_highland": {"x": [-3.50, -1.50], "y": [-1.00, 3.06]},
                "center": {"x": [-1.50, 1.50], "y": [-1.50, 1.50]},
                "blue_highland": {"x": [1.50, 3.50], "y": [-3.06, 1.00]},
                "supply_red": {"x": [-6.00, -3.50], "y": [2.00, 4.00]},
            }

        self.create_subscription(PoseStamped, "/ground_truth/pose",
                                 self.on_pose, 10)
        self.create_subscription(Float32, "/referee/damage",
                                 self.on_damage, 10)
        self.create_subscription(Bool, "/sentry/fire", self.on_fire, 10)
        self.pub = self.create_publisher(String, "/referee/state", 10)
        self.create_timer(1.0 / hz, self.tick)
        self.get_logger().info(
            f"裁判系统模拟启动 | HP {self.hp_max:.0f} 弹量 {self.ammo:.0f} "
            f"热量 {self.heat_max:.0f} 时长 {self.duration:.0f}s")

    # ---------- 订阅 ----------
    def on_pose(self, m):
        self.pose = (m.pose.position.x, m.pose.position.y)

    def on_damage(self, m):
        if self.alive and not self.invincible and m.data > 0:
            self.hp -= float(m.data)

    def on_fire(self, m):
        if not m.data or not self.alive:
            return
        if self.weak:
            return
        if self.ammo <= 0:
            self.get_logger().warn("没弹了")
            return
        if self.heat + self.heat_shot > self.heat_max:
            self.get_logger().warn("热量超限，发射机构锁定")
            return
        self.ammo -= 1
        self.heat = min(self.heat_max, self.heat + self.heat_shot)

    # ---------- 区域判定 ----------
    def which_zone(self):
        if self.pose is None:
            return "none"
        x, y = self.pose
        for name, b in self.zones.items():
            if b["x"][0] <= x <= b["x"][1] and b["y"][0] <= y <= b["y"][1]:
                return name
        return "none"

    # ---------- 主循环 ----------
    def tick(self):
        now = self.get_clock().now()
        dt = (now - self.last_t).nanoseconds * 1e-9
        self.last_t = now
        dt = min(max(dt, 1e-3), 0.5)

        if self.alive:
            self.elapsed += dt

        self.zone = self.which_zone()

        # 比赛结束
        if self.elapsed >= self.duration and self.stage == 4:
            self.stage = 5
            self.get_logger().info("比赛结束")

        # 热量冷却
        self.heat = max(0.0, self.heat - self.cooling * dt)

        if self.alive:
            # 补给区回血 + 解除虚弱/无敌
            if self.zone.startswith("supply"):
                self.hp = min(self.hp_max, self.hp + self.hp_max * self.heal_frac * dt)
                if self.weak or self.invincible:
                    self.weak = False
                    self.invincible = False
                    self.invincible_time = 0.0
                    self.get_logger().info("检测到补给区，虚弱与无敌解除")
            # 无敌倒计时
            if self.invincible:
                self.invincible_time -= dt
                if self.invincible_time <= 0:
                    self.invincible = False
            # 持续受击（默认只在中央区受击 = 模拟中央被争夺）
            if (self.dps > 0 and not self.invincible
                    and (not self.dmg_zone or self.zone == self.dmg_zone)):
                self.hp -= self.dps * dt
            # 战亡
            if self.hp <= 0:
                self.hp = 0.0
                self.alive = False
                self.deaths += 1
                self.heat = 0.0
                self.respawn_ticks = self.respawn_ticks_base * self.deaths
                self.control_blue += self.death_pen    # 我方战亡，对方控制度不扣
                self.get_logger().warn(
                    f"哨兵战亡（第 {self.deaths} 次），需读条 {self.respawn_ticks:.0f}")
        else:
            # 复活读条：每秒 +1
            if self.respawn_ticks > 0:
                self.respawn_ticks -= dt
            if self.respawn_ticks <= 0:
                self.alive = True
                self.hp = self.hp_max * 0.2
                self.weak = True
                self.invincible = True
                self.invincible_time = self.invincible_max
                self.get_logger().info(
                    "复活：20% 血量 + 虚弱 + 无敌 30 秒（发射机构锁定、无法占点）")

        # 控制度：己方控制中央区则对方扣
        if self.alive and not self.weak and self.zone == "center":
            self.control_blue -= dt

        self.publish()

    def publish(self):
        s = String()
        s.data = json.dumps({
            "stamp": self.get_clock().now().nanoseconds * 1e-9,
            "stage": self.stage,
            "time_left": max(0.0, self.duration - self.elapsed),
            "alive": self.alive,
            "weak": self.weak,
            "invincible": self.invincible,
            "invincible_left": round(max(0.0, self.invincible_time), 1),
            "hp": round(self.hp, 1),
            "hp_max": self.hp_max,
            "hp_frac": round(self.hp / self.hp_max, 3),
            "ammo": int(self.ammo),
            "heat": round(self.heat, 1),
            "heat_max": self.heat_max,
            "zone": self.zone,
            "deaths": self.deaths,
            "control_self": round(self.control_blue if True else 0, 1),
            "control_enemy": round(self.control_red, 1),
        }, ensure_ascii=False)
        self.pub.publish(s)


def main():
    rclpy.init()
    n = RefereeSim()
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    finally:
        n.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
