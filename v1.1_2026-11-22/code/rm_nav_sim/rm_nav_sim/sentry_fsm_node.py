#!/usr/bin/env python3
"""哨兵决策状态机（守点型）。

战术定位（见 docs/哨兵战术定位与地面航点.md）：
  哨兵只在平地作战，主任务是占住中央控制区；高地由步兵负责。

分层设计（参考 RMUC-OfflineRL 的做法：战术层低频、执行层高频）：
  本节点       1~5 Hz   输出「去哪」+「开不开火」
  Nav2         20 Hz    输出速度
  自瞄（另做） 100+ Hz   输出云台角度

三条铁律：
  ① 不追击 —— 离开控制区，自己的占领进度每秒 -1
  ② 每个状态必须有超时 —— 全自动、无操作手，卡死就是 5 分钟不动
  ③ 分工赛前定死 —— 规则禁止哨兵多机通信，运行时不协调
"""
import json
import math

import rclpy
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node
from std_msgs.msg import Bool, String


def quat_from_yaw(y):
    return (0.0, 0.0, math.sin(y * 0.5), math.cos(y * 0.5))


class SentryFSM(Node):
    # 状态定义
    IDLE = "IDLE"
    GOTO_CENTER = "GOTO_CENTER"
    HOLD = "HOLD"
    RETREAT = "RETREAT"
    HEAL = "HEAL"
    DEAD = "DEAD"
    FALLBACK = "FALLBACK"

    def __init__(self):
        super().__init__("sentry_fsm")
        d = [
            ("hold_point", [0.0, 0.0]),        # 中央守点（控制区正中）
            ("shift_points", [0.0, 0.9, 0.0, -0.9]),   # 区内变位点
            ("supply_point", [-4.75, 3.0]),    # 己方补给区
            ("hp_retreat", 0.30),              # 低于此比例撤退
            ("hp_resume", 0.90),               # 回满到此比例返回
            ("arrive_dist", 0.35),             # 到达判定距离
            ("shift_period", 6.0),             # 变位周期（秒）
            ("tick_hz", 5.0),
            ("state_timeout", 30.0),           # 单状态最长停留（兜底）
            ("sim_fire", True),                # 仿真里自动开火（练习热量/弹量逻辑）
            ("fire_interval", 0.25),           # 开火间隔（秒）
            ("enable", True),
        ]
        for n, v in d:
            self.declare_parameter(n, v)
        p = self.get_parameter

        self.hold = list(p("hold_point").value)
        sv = list(p("shift_points").value)
        self.shifts = [(sv[i], sv[i + 1]) for i in range(0, len(sv) - 1, 2)]
        self.supply = list(p("supply_point").value)
        self.hp_retreat = float(p("hp_retreat").value)
        self.hp_resume = float(p("hp_resume").value)
        self.arrive = float(p("arrive_dist").value)
        self.shift_period = float(p("shift_period").value)
        self.timeout = float(p("state_timeout").value)
        self.sim_fire = bool(p("sim_fire").value)
        self.fire_interval = float(p("fire_interval").value)
        self.enable = bool(p("enable").value)
        hz = float(p("tick_hz").value)

        self.state = self.IDLE
        self.state_t = 0.0
        self.ref = {}
        self.pose = None
        self.cur_goal = None
        self.want_goal = None          # 当前状态想要去的目标（用于看门狗重发）
        self.since_goal_try = 0.0
        self.goal_handle = None
        self.shift_i = 0
        self.since_shift = 0.0
        self.since_fire = 0.0
        self.fire_enabled = False
        self.last_t = self.get_clock().now()
        self.log_rows = []

        self.create_subscription(String, "/referee/state", self.on_ref, 10)
        self.create_subscription(PoseStamped, "/ground_truth/pose",
                                 self.on_pose, 10)
        self.fire_en_pub = self.create_publisher(Bool, "/sentry/fire_enable", 10)
        self.fire_pub = self.create_publisher(Bool, "/sentry/fire", 10)
        self.state_pub = self.create_publisher(String, "/sentry/state", 10)
        self.nav = ActionClient(self, NavigateToPose, "navigate_to_pose")
        self.create_timer(1.0 / hz, self.tick)
        self.get_logger().info("哨兵状态机启动 | 主任务：占住中央控制区")

    # ---------- 订阅 ----------
    def on_ref(self, m):
        try:
            self.ref = json.loads(m.data)
        except Exception:
            pass

    def on_pose(self, m):
        self.pose = (m.pose.position.x, m.pose.position.y)

    # ---------- Nav2 目标 ----------
    def send_goal(self, x, y, yaw=0.0):
        # 注意：Nav2 可能还没起来，wait_for_server 失败时必须返回 False，
        # 由 tick() 里的看门狗重试 —— 否则状态机会永远卡在"想发但没发出去"。
        if not self.nav.wait_for_server(timeout_sec=0.5):
            return False
        g = NavigateToPose.Goal()
        g.pose.header.frame_id = "map"
        g.pose.header.stamp = self.get_clock().now().to_msg()
        g.pose.pose.position.x = float(x)
        g.pose.pose.position.y = float(y)
        q = quat_from_yaw(yaw)
        g.pose.pose.orientation.z = q[2]
        g.pose.pose.orientation.w = q[3]
        if self.goal_handle is not None:
            self.goal_handle.cancel_goal_async()
        fut = self.nav.send_goal_async(g)
        fut.add_done_callback(self._goal_resp)
        self.cur_goal = (float(x), float(y))
        return True

    def _goal_resp(self, fut):
        try:
            self.goal_handle = fut.result()
        except Exception:
            self.goal_handle = None

    def cancel_goal(self):
        if self.goal_handle is not None:
            try:
                self.goal_handle.cancel_goal_async()
            except Exception:
                pass
        self.cur_goal = None

    def dist_to(self, x, y):
        if self.pose is None:
            return 1e9
        return math.hypot(self.pose[0] - x, self.pose[1] - y)

    # ---------- 状态切换 ----------
    def goto(self, st, goal=None):
        if st != self.state:
            self.get_logger().info(
                f"[{self.state}] -> [{st}]  "
                f"hp={self.ref.get('hp_frac', 0)*100:.0f}%  "
                f"zone={self.ref.get('zone','?')}")
            self.state = st
            self.state_t = 0.0
            self.since_shift = 0.0
            self.want_goal = goal
            if goal is not None:
                self.send_goal(*goal)

    # ---------- 主循环 ----------
    def tick(self):
        now = self.get_clock().now()
        dt = (now - self.last_t).nanoseconds * 1e-9
        self.last_t = now
        dt = min(max(dt, 1e-3), 0.5)
        self.state_t += dt
        self.since_shift += dt
        self.since_goal_try += dt

        # 目标看门狗：当前状态应该有目标但没有 → 每秒重试一次
        if (self.want_goal is not None and self.cur_goal is None
                and self.since_goal_try > 1.0):
            if self.send_goal(*self.want_goal):
                self.since_goal_try = 0.0
                self.get_logger().info(
                    f"[{self.state}] 目标重发成功 → {self.want_goal}")

        if not self.enable or not self.ref:
            self.publish()
            return

        hp = float(self.ref.get("hp_frac", 1.0))
        alive = bool(self.ref.get("alive", True))
        weak = bool(self.ref.get("weak", False))
        stage = int(self.ref.get("stage", 4))
        zone = str(self.ref.get("zone", "none"))

        # ---- 状态转移 ----
        if not alive:
            self.fire_enabled = False
            if self.state != self.DEAD:
                # 战亡后必须撤销当前目标，否则机器人会带着旧目标继续跑
                self.cancel_goal()
            self.goto(self.DEAD)
        elif stage != 4:
            # 比赛未开始 / 已结束
            self.fire_enabled = False
            if self.state != self.IDLE:
                self.goto(self.IDLE)
        elif self.state == self.DEAD:
            # 复活后血量只有 20% 且处于虚弱：必须先去补给区解除虚弱并回血，
            # 不能直接冲回中央（否则会反复送死）。
            if weak or hp < self.hp_resume:
                self.goto(self.RETREAT, self.supply)
            else:
                self.goto(self.GOTO_CENTER, self.hold)
        elif self.state == self.IDLE:
            self.goto(self.GOTO_CENTER, self.hold)
        elif hp < self.hp_retreat and self.state not in (self.RETREAT, self.HEAL):
            self.goto(self.RETREAT, self.supply)
        elif self.state == self.RETREAT:
            self.fire_enabled = False
            if hp >= self.hp_resume and not weak:
                self.goto(self.GOTO_CENTER, self.hold)
            elif self.dist_to(*self.supply) < self.arrive:
                self.goto(self.HEAL)
        elif self.state == self.HEAL:
            self.fire_enabled = False
            # 回血靠裁判系统（进补给区自动回），这里只等
            if hp >= self.hp_resume and not weak:
                self.goto(self.GOTO_CENTER, self.hold)
            elif self.dist_to(*self.supply) > self.arrive * 1.5:
                # 被推出补给区，回去
                self.send_goal(*self.supply)
            elif self.state_t > self.timeout:
                self.goto(self.GOTO_CENTER, self.hold)
        elif self.state == self.GOTO_CENTER:
            self.fire_enabled = True
            if zone == "center":
                self.goto(self.HOLD)
            elif self.state_t > self.timeout and self.dist_to(*self.hold) > 2.0:
                # 走太久没到，重发
                self.send_goal(*self.hold)
                self.state_t = 0.0
        elif self.state == self.HOLD:
            self.fire_enabled = True
            if zone != "center":
                # 被打出控制区 → 回去
                self.send_goal(*self.hold)
                self.goto(self.GOTO_CENTER, self.hold)
            elif self.since_shift > self.shift_period:
                # 区内变位，避免被预瞄
                self.shift_i = (self.shift_i + 1) % max(1, len(self.shifts))
                sx, sy = self.shifts[self.shift_i]
                self.send_goal(sx, sy)
                self.since_shift = 0.0
        elif self.state == self.FALLBACK:
            if self.dist_to(*self.hold) < self.arrive:
                self.goto(self.GOTO_CENTER, self.hold)

        # ---- 兜底：任何状态超时都回中央 ----
        if (self.state not in (self.IDLE, self.DEAD, self.FALLBACK)
                and self.state_t > self.timeout * 2):
            self.get_logger().warn(f"[{self.state}] 超时，强制回中央")
            self.goto(self.FALLBACK, self.hold)

        # ---- 仿真开火（练习热量/弹量逻辑）----
        if self.sim_fire:
            self.since_fire += dt
            if self.fire_enabled and self.since_fire >= self.fire_interval:
                self.since_fire = 0.0
                b = Bool(); b.data = True
                self.fire_pub.publish(b)

        self.log(zone, hp)
        self.publish()

    def log(self, zone, hp):
        row = (round(self.state_t, 1), self.state, round(hp, 2), zone,
               int(self.ref.get("ammo", 0)), round(self.ref.get("heat", 0), 1))
        self.log_rows.append(row)
        if len(self.log_rows) > 3000:
            self.log_rows = self.log_rows[-3000:]

    def publish(self):
        b = Bool(); b.data = self.fire_enabled
        self.fire_en_pub.publish(b)
        s = String()
        s.data = json.dumps({
            "state": self.state,
            "state_time": round(self.state_t, 1),
            "fire_enable": self.fire_enabled,
            "goal": self.cur_goal,
            "hp_frac": self.ref.get("hp_frac"),
            "zone": self.ref.get("zone"),
        }, ensure_ascii=False)
        self.state_pub.publish(s)

    def dump(self, path="/tmp/sentry_fsm.csv"):
        with open(path, "w", encoding="utf-8") as f:
            f.write("t,state,hp,zone,ammo,heat\n")
            for r in self.log_rows:
                f.write(",".join(str(v) for v in r) + "\n")
        self.get_logger().info(f"日志已写出 {path}")


def main():
    rclpy.init()
    n = SentryFSM()
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        n.dump()
    finally:
        n.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
