#!/usr/bin/env python3
"""Livox (CustomMsg) → 去畸变 → 2D LaserScan。

真机与仿真最大的差别：仿真的 sim_node 直接发 LaserScan，而真机上雷达发的是
livox_ros_driver2/CustomMsg（自带逐点 offset_time）。本节点负责把这一层补上。

数据流
------
    /livox/lidar  CustomMsg  (frame = livox_frame)
    /livox/imu    Imu        (同一雷达内置 IMU，200 Hz)
            │
            ├─ 逐点去畸变：按 offset_time 积分 IMU 的 z 轴角速度
            ├─ 倒置处理：roll=180° → y 取反、IMU z 取反
            ├─ 一次性 TF：用帧首时刻把点云变换到 target_frame
            └─ 2D 切片：每方位取最近点
            ▼
    /scan  LaserScan (frame = target_frame)

为什么去畸变要在这里做
----------------------
Nav2 只吃 LaserScan，而 LaserScan 没有逐点时间戳——一旦转成 LaserScan，
帧内旋转造成的误差就永久固化了，下游再也无法修正。
所以必须在点云阶段、丢掉时间戳之前做完。
"""
from __future__ import annotations

import array
import math

import numpy as np
import rclpy
import tf2_ros
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Imu, LaserScan, PointCloud2, PointField
from std_msgs.msg import Header

try:
    from livox_ros_driver2.msg import CustomMsg
    HAVE_CUSTOM = True
except Exception:                                   # 没装驱动时也能 import
    CustomMsg = None
    HAVE_CUSTOM = False


class LivoxScanNode(Node):
    def __init__(self):
        super().__init__("livox_scan")

        d = [
            ("inverted", True),            # 倒置安装（roll 180°）
            ("deskew", True),              # 逐点去畸变
            ("imu_z_sign", -1.0),          # 倒置时 IMU 的 z 轴取反
            ("target_frame", "base_footprint"),
            ("scan_topic", "/scan"),
            ("cloud_topic", "/cloud_deskewed"),
            ("publish_cloud", True),
            ("range_min", 0.35),           # 近处盲区 + 自车遮挡
            ("range_max", 40.0),
            ("height_min", -0.30),         # 相对雷达的切片高度范围
            ("height_max", 0.60),
            ("angle_bins", 720),
            ("imu_buffer_sec", 2.0),
            ("tf_timeout", 0.15),
        ]
        for n, v in d:
            self.declare_parameter(n, v)
        p = self.get_parameter

        self.inverted = bool(p("inverted").value)
        self.deskew = bool(p("deskew").value)
        self.imu_sign = float(p("imu_z_sign").value)
        self.target_frame = str(p("target_frame").value)
        self.rmin = float(p("range_min").value)
        self.rmax = float(p("range_max").value)
        self.hmin = float(p("height_min").value)
        self.hmax = float(p("height_max").value)
        self.nbins = int(p("angle_bins").value)
        self.buf_sec = float(p("imu_buffer_sec").value)
        self.tf_timeout = float(p("tf_timeout").value)

        self.angle_min = -math.pi
        self.angle_max = math.pi
        self.angle_inc = 2 * math.pi / self.nbins

        # IMU 环形缓冲
        self.imu_t: list[float] = []
        self.imu_wz: list[float] = []

        self.tfbuf = tf2_ros.Buffer()
        self.tfl = tf2_ros.TransformListener(self.tfbuf, self)

        qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT)
        if HAVE_CUSTOM:
            self.create_subscription(CustomMsg, "/livox/lidar",
                                     self.on_cloud, qos)
        else:
            self.get_logger().warn("没有 livox_ros_driver2，CustomMsg 订阅不可用")
        self.create_subscription(Imu, "/livox/imu", self.on_imu, 200)

        self.scan_pub = self.create_publisher(
            LaserScan, str(p("scan_topic").value), 10)
        self.cloud_pub = (self.create_publisher(
            PointCloud2, str(p("cloud_topic").value), 5)
            if bool(p("publish_cloud").value) else None)

        self.n_frames = 0
        self.log_t = self.get_clock().now()
        self.get_logger().info(
            f"livox_scan 启动 | inverted={self.inverted} "
            f"deskew={self.deskew} imu_z_sign={self.imu_sign:+g} "
            f"target={self.target_frame} range=[{self.rmin},{self.rmax}]")

    # ---------------- IMU ----------------
    def on_imu(self, m: Imu):
        t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        self.imu_t.append(t)
        self.imu_wz.append(float(m.angular_velocity.z))
        if len(self.imu_t) > 4 and self.imu_t[-1] - self.imu_t[0] > self.buf_sec:
            cut = 0
            while (cut < len(self.imu_t) - 3
                   and self.imu_t[-1] - self.imu_t[cut] > self.buf_sec):
                cut += 1
            self.imu_t = self.imu_t[cut:]
            self.imu_wz = self.imu_wz[cut:]

    def integrate_angle(self, t0: float, ts: np.ndarray) -> np.ndarray:
        """∫[t0, t] ω dt —— 逐点累计转角（rad）。"""
        if len(self.imu_t) < 2:
            return np.zeros_like(ts)
        tt = np.asarray(self.imu_t, float)
        ww = np.asarray(self.imu_wz, float) * self.imu_sign
        # 在 IMU 采样点上算累计角，再插值到各束时间
        cum = np.concatenate([[0.0], np.cumsum(
            0.5 * (ww[1:] + ww[:-1]) * np.diff(tt))])
        base = float(np.interp(t0, tt, cum))
        return np.interp(ts, tt, cum) - base

    # ---------------- 点云 ----------------
    def on_cloud(self, msg):
        n = len(msg.points)
        if n == 0:
            return
        t0 = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

        # 逐点提取（列表推导比逐字段访问快得多）
        pts = np.empty((n, 3), np.float32)
        off = np.empty(n, np.float32)
        for i, q in enumerate(msg.points):
            pts[i, 0] = q.x
            pts[i, 1] = q.y
            pts[i, 2] = q.z
            off[i] = q.offset_time * 1e-9          # ns → s

        # 丢掉无回波点（协议规定为 0,0,0）
        valid = np.any(pts != 0.0, axis=1)
        pts, off = pts[valid], off[valid]
        if pts.shape[0] == 0:
            return

        # ---- 逐点去畸变（必须在雷达自身坐标系里做）----
        #
        # 倒置的处理**不在这里**，而是在 TF 里：
        #   gimbal_yaw --(roll 180°)--> laser --(identity)--> livox_frame
        # 因为「倒置」是安装姿态，属于坐标系定义，不是数据处理。
        #
        # 但 IMU 的符号必须在这里处理：倒置后雷达 z 轴朝下，
        # 云台绕世界 z 转 +ω，IMU 测到的是 −ω。所以：
        #   直立 imu_z_sign = +1，倒置 imu_z_sign = −1
        # 符号搞反的症状很有迷惑性：看起来补偿了，实际误差翻倍。
        if self.deskew:
            tb = t0 + off
            dth = self.integrate_angle(t0, tb)
            c, s = np.cos(-dth), np.sin(-dth)
            x, y = pts[:, 0].copy(), pts[:, 1].copy()
            pts[:, 0] = c * x - s * y
            pts[:, 1] = s * x + c * y

        # ---- 变换到 target_frame（用帧首时刻做一次，省算力）----
        R = None
        try:
            tr = self.tfbuf.lookup_transform(
                self.target_frame, msg.header.frame_id,
                rclpy.time.Time(seconds=t0),
                timeout=rclpy.duration.Duration(seconds=self.tf_timeout))
            q = tr.transform.rotation
            t = tr.transform.translation
            R = self._quat_to_R(q.x, q.y, q.z, q.w)
            pts = (R @ pts.T).T + np.array([t.x, t.y, t.z], np.float32)
        except Exception as e:  # noqa: BLE001
            self.get_logger().warn(f"TF {self.target_frame}<-"
                                   f"{msg.header.frame_id} 查询失败: {e}",
                                   throttle_duration_sec=5.0)
            return

        # ---- 发布去畸变点云 ----
        if self.cloud_pub is not None:
            self._pub_cloud(pts, t0, msg.header.frame_id)

        # ---- 2D 切片 ----
        self._pub_scan(pts, t0)

        self.n_frames += 1
        now = self.get_clock().now()
        if (now - self.log_t).nanoseconds * 1e-9 > 10.0:
            self.log_t = now
            self.get_logger().info(
                f"帧 {self.n_frames} | 有效点 {pts.shape[0]} | "
                f"点云在 {self.target_frame} 系")

    @staticmethod
    def _quat_to_R(x, y, z, w):
        return np.array([
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ], np.float32)

    def _pub_cloud(self, pts, t0, frame):
        m = PointCloud2()
        m.header.stamp.sec = int(t0)
        m.header.stamp.nanosec = int((t0 - int(t0)) * 1e9)
        m.header.frame_id = self.target_frame
        m.height = 1
        m.width = pts.shape[0]
        m.fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        m.point_step = 12
        m.row_step = 12 * pts.shape[0]
        m.is_dense = True
        # 同 hik_camera_node：bytes 会走逐字节转换，必须用 array.array('B')
        m.data = array.array("B", pts.astype(np.float32).tobytes())
        self.cloud_pub.publish(m)

    def _pub_scan(self, pts, t0):
        # 高度切片（相对 target_frame 原点；装车时该原点在 base_footprint）
        zmask = (pts[:, 2] >= self.hmin) & (pts[:, 2] <= self.hmax)
        P = pts[zmask]
        if P.shape[0] == 0:
            return
        r = np.hypot(P[:, 0], P[:, 1])
        amask = (r >= self.rmin) & (r <= self.rmax)
        P, r = P[amask], r[amask]
        if P.shape[0] == 0:
            return

        ang = np.arctan2(P[:, 1], P[:, 0])
        idx = np.clip(((ang + math.pi) / self.angle_inc).astype(np.int32),
                      0, self.nbins - 1)
        ranges = np.full(self.nbins, np.inf, np.float32)
        np.minimum.at(ranges, idx, r.astype(np.float32))

        m = LaserScan()
        m.header.stamp.sec = int(t0)
        m.header.stamp.nanosec = int((t0 - int(t0)) * 1e9)
        m.header.frame_id = self.target_frame
        m.angle_min = self.angle_min
        m.angle_max = self.angle_max
        m.angle_increment = self.angle_inc
        m.scan_time = 0.1
        m.time_increment = 0.1 / self.nbins
        m.range_min = float(self.rmin)
        m.range_max = float(self.rmax)
        out = ranges.copy()
        out[~np.isfinite(out)] = 0.0
        m.ranges = out.tolist()
        self.scan_pub.publish(m)


def main():
    rclpy.init()
    n = LivoxScanNode()
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    finally:
        n.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
