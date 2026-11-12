#!/usr/bin/env python3
"""极简 2D 网格导航仿真器（性能版）。

关键设计：
1. 用**真实经过时间**积分 —— 定时器掉帧也不会让机器人"少走"（之前固定 dt 是致命的）
2. 物理 / 雷达 / 订阅 分属三个回调组 + 多线程执行器，慢的射线投射不阻塞里程计
3. 射线投射用预分配 buffer，采样量减少到约 1/3
4. 按车身轮廓（9 点）判碰撞，与 Nav2 代价地图口径一致

发布：/odom, /scan, TF(odom->base_footprint, base_footprint->laser), /ground_truth/pose
订阅：/cmd_vel
"""
import math
import threading

import numpy as np
import rclpy
import yaml
from geometry_msgs.msg import PoseStamped, TransformStamped, Twist
from nav_msgs.msg import Odometry
from PIL import Image
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster


def yaw_to_quat(y):
    return (0.0, 0.0, math.sin(y * 0.5), math.cos(y * 0.5))


class GridSim(Node):
    def __init__(self):
        super().__init__("grid_sim")
        for name, val in (
            ("map_yaml", ""),
            ("start_x", 0.0),
            ("start_y", -2.5),
            ("start_yaw", 0.0),
            ("n_beams", 180),
            ("range_max", 8.0),
            ("ray_step", 0.10),
            ("physics_hz", 50.0),
            ("scan_hz", 10.0),
            ("range_noise", 0.01),
            ("robot_radius", 0.18),
            ("laser_height", 0.20),
            ("cmd_timeout", 0.5),
        ):
            self.declare_parameter(name, val)

        p = self.get_parameter
        map_yaml = p("map_yaml").value
        if not map_yaml:
            raise RuntimeError("必须给 map_yaml 参数")

        with open(map_yaml, encoding="utf-8") as f:
            meta = yaml.safe_load(f)
        img = meta["image"]
        if not img.startswith("/"):
            img = str(__import__("pathlib").Path(map_yaml).parent / img)
        self.occ = np.asarray(Image.open(img).convert("L")) < 128
        self.occ_flat = self.occ.ravel()
        self.res = float(meta["resolution"])
        self.ox = float(meta["origin"][0])
        self.oy = float(meta["origin"][1])
        self.H, self.W = self.occ.shape

        self.x = float(p("start_x").value)
        self.y = float(p("start_y").value)
        self.th = float(p("start_yaw").value)

        self.n = int(p("n_beams").value)
        self.rmax = float(p("range_max").value)
        self.noise = float(p("range_noise").value)
        self.radius = float(p("robot_radius").value)
        self.timeout = float(p("cmd_timeout").value)
        self.scan_hz = float(p("scan_hz").value)

        # 车身轮廓采样点
        self.footprint = [(0.0, 0.0)] + [
            (self.radius * math.cos(a), self.radius * math.sin(a))
            for a in np.linspace(0, 2 * math.pi, 8, endpoint=False)
        ]

        # 射线投射预计算
        step = float(p("ray_step").value)
        self.ds = np.arange(0.0, self.rmax, step)
        S = len(self.ds)
        self.angles = np.linspace(-math.pi, math.pi, self.n, endpoint=False)
        self._gxf = (np.outer(self.ds, np.cos(self.angles)) / self.res).astype(np.float32)
        self._gyf = (np.outer(self.ds, np.sin(self.angles)) / self.res).astype(np.float32)
        self._t1 = np.empty((S, self.n), dtype=np.float32)
        self._t2 = np.empty((S, self.n), dtype=np.float32)

        self.lock = threading.Lock()
        self.cmd_v = 0.0
        self.cmd_w = 0.0
        self.cmd_time = self.get_clock().now()
        self._last_t = self.get_clock().now()

        self.odom_pub = self.create_publisher(Odometry, "/odom", 10)
        self.scan_pub = self.create_publisher(LaserScan, "/scan", 10)
        self.gt_pub = self.create_publisher(PoseStamped, "/ground_truth/pose", 10)
        self.tf_broad = TransformBroadcaster(self)
        self.static_broad = StaticTransformBroadcaster(self)
        self._publish_static_tf()

        cbg_phys = MutuallyExclusiveCallbackGroup()
        cbg_scan = MutuallyExclusiveCallbackGroup()
        cbg_sub = MutuallyExclusiveCallbackGroup()
        self.create_timer(1.0 / float(p("physics_hz").value), self.on_physics,
                          callback_group=cbg_phys)
        self.create_timer(1.0 / self.scan_hz, self.on_scan, callback_group=cbg_scan)
        self.create_subscription(Twist, "/cmd_vel", self.on_cmd, 10,
                                 callback_group=cbg_sub)

        self.ticks = 0
        self.get_logger().info(
            f"grid_sim up | map {self.W}x{self.H}@{self.res}m | beams={self.n} "
            f"step={step}m | start=({self.x:.2f},{self.y:.2f},{self.th:.2f})")

    # ---------------- 基础 ----------------
    def _publish_static_tf(self):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = "base_footprint"
        t.child_frame_id = "laser"
        t.transform.translation.z = float(self.get_parameter("laser_height").value)
        t.transform.rotation.w = 1.0
        self.static_broad.sendTransform([t])

    def on_cmd(self, msg):
        self.cmd_v = msg.linear.x
        self.cmd_w = msg.angular.z
        self.cmd_time = self.get_clock().now()

    def _cell_occ(self, x, y):
        gx = int(round((x - self.ox) / self.res))
        gy = int(round((y - self.oy) / self.res))
        if gx < 0 or gx >= self.W or gy < 0 or gy >= self.H:
            return True
        return bool(self.occ[self.H - 1 - gy, gx])

    def _blocked(self, x, y):
        for dx, dy in self.footprint:
            if self._cell_occ(x + dx, y + dy):
                return True
        return False

    # ---------------- 物理 ----------------
    def on_physics(self):
        now = self.get_clock().now()
        with self.lock:
            dt = (now - self._last_t).nanoseconds * 1e-9
            self._last_t = now
        dt = min(max(dt, 1e-4), 0.20)      # 关键：真实 dt，掉帧也不打折

        age = (now - self.cmd_time).nanoseconds * 1e-9
        v, w = (self.cmd_v, self.cmd_w) if age < self.timeout else (0.0, 0.0)

        with self.lock:
            nx = self.x + v * math.cos(self.th) * dt
            ny = self.y + v * math.sin(self.th) * dt
            nth = self.th + w * dt
            if not self._blocked(nx, ny):
                self.x, self.y, self.th = nx, ny, nth
            else:
                if not self._blocked(nx, self.y):
                    self.x = nx
                if not self._blocked(self.x, ny):
                    self.y = ny
                self.th = nth
            x, y, th = self.x, self.y, self.th

        stamp = now.to_msg()
        self._pub_odom(stamp, v, w, x, y, th)
        self.ticks += 1
        if self.ticks % 2 == 0:
            self._pub_tf(stamp, x, y, th)

    def _pub_odom(self, stamp, v, w, x, y, th):
        m = Odometry()
        m.header.stamp = stamp
        m.header.frame_id = "odom"
        m.child_frame_id = "base_footprint"
        m.pose.pose.position.x = x
        m.pose.pose.position.y = y
        q = yaw_to_quat(th)
        m.pose.pose.orientation.z = q[2]
        m.pose.pose.orientation.w = q[3]
        m.twist.twist.linear.x = v
        m.twist.twist.angular.z = w
        self.odom_pub.publish(m)

        gt = PoseStamped()
        gt.header.stamp = stamp
        gt.header.frame_id = "map"
        gt.pose = m.pose.pose
        self.gt_pub.publish(gt)

    def _pub_tf(self, stamp, x, y, th):
        t = TransformStamped()
        t.header.stamp = stamp
        t.header.frame_id = "odom"
        t.child_frame_id = "base_footprint"
        t.transform.translation.x = x
        t.transform.translation.y = y
        q = yaw_to_quat(th)
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]
        self.tf_broad.sendTransform([t])

    # ---------------- 雷达 ----------------
    def on_scan(self):
        with self.lock:
            x, y, th = self.x, self.y, self.th

        c, s = math.cos(th), math.sin(th)
        # gx = round((x-ox)/res + [cos, -sin]·offset)
        np.multiply(self._gxf, c, out=self._t1)
        np.multiply(self._gyf, -s, out=self._t2)
        np.add(self._t1, self._t2, out=self._t1)
        np.add(self._t1, (x - self.ox) / self.res, out=self._t1)
        np.rint(self._t1, out=self._t1)
        gxi = self._t1.astype(np.int32)

        np.multiply(self._gxf, s, out=self._t1)
        np.multiply(self._gyf, c, out=self._t2)
        np.add(self._t1, self._t2, out=self._t1)
        np.add(self._t1, (y - self.oy) / self.res, out=self._t1)
        np.rint(self._t1, out=self._t1)
        gyi = self._t1.astype(np.int32)

        valid = (gxi >= 0) & (gxi < self.W) & (gyi >= 0) & (gyi < self.H)
        np.clip(gxi, 0, self.W - 1, out=gxi)
        np.clip(gyi, 0, self.H - 1, out=gyi)
        flat = (self.H - 1 - gyi) * self.W + gxi
        hit = self.occ_flat[flat]
        hit &= valid

        any_hit = hit.any(axis=0)
        first = hit.argmax(axis=0)
        rng = np.where(any_hit, self.ds[first], self.rmax).astype(np.float64)
        rng += np.random.normal(0.0, self.noise, size=rng.shape)
        np.clip(rng, 0.15, self.rmax, out=rng)

        m = LaserScan()
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = "laser"
        m.angle_min = -math.pi
        m.angle_max = math.pi - 2 * math.pi / self.n
        m.angle_increment = 2 * math.pi / self.n
        m.scan_time = 1.0 / self.scan_hz
        m.range_min = 0.15
        m.range_max = self.rmax
        m.ranges = rng.astype(np.float32).tolist()
        self.scan_pub.publish(m)


def main():
    rclpy.init()
    node = GridSim()
    ex = MultiThreadedExecutor(num_threads=3)
    ex.add_node(node)
    try:
        ex.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
