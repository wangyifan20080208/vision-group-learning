#!/usr/bin/env python3
"""用雷达内置 IMU 做旋转去畸变 + 车体自遮挡滤波，输出干净的 /scan。

输入：
  /scan_raw   LaserScan，frame = laser（含畸变与自遮挡）
  /livox/imu  Imu，frame = laser（雷达内置 IMU）
输出：
  /scan       LaserScan，frame = base_footprint（已去畸变、已滤自车）

原理
----
倒置（绕 x 轴 180°）使方位角取负，因此

    base 系方位角  β = θ_gimbal + (−α)
    世界系方位角   ψ = θ_robot + θ_gimbal − α

一帧扫描期间云台转过 Δθ，某束在 t_i 时刻测得，若当成 t0 时刻的朝向解读，
就会错位 Δθ_i = ∫ω dt。用 IMU 的 z 轴角速度积分即可还原：

    修正后方位角  α' = α + ∫(ω_z) dt

（倒置使 IMU 的 z 轴翻转，故程序中取负，推导见 docs）

车体自遮挡
----------
倒置雷达 52° 下视场会扫到自车。二维切片取"每方位最近点"，自车点永远最近，
所以不去掉就会整圈都读成自车轮廓。处理：
  1) 本体先验（底盘多边形、云台载荷多边形）已在仿真/驱动层产生近距回波
  2) 本节点用 ego_min_range 一刀切掉近距点
  3) 可选：按底盘在 base 系的固定扇形做掩膜（去畸变后底盘遮挡是静止的）
"""
import math

import numpy as np
import rclpy
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import Imu, JointState, LaserScan
from tf2_ros import Buffer, TransformListener


class DeskewNode(Node):
    def __init__(self):
        super().__init__("deskew_node")
        self.declare_parameter("in_topic", "/scan_raw")
        self.declare_parameter("out_topic", "/scan")
        self.declare_parameter("imu_topic", "/livox/imu")
        self.declare_parameter("joint_topic", "/joint_states")
        self.declare_parameter("enable_deskew", True)
        self.declare_parameter("ego_min_range", 0.0)
        self.declare_parameter("gimbal_frame", "gimbal_yaw")
        self.declare_parameter("robot_frame", "base_footprint")
        self.declare_parameter("imu_buffer_sec", 2.0)
        self.declare_parameter("n_out_beams", 720)    # 输出角度栅格，比输入更细

        p = self.get_parameter
        self.enable_deskew = bool(p("enable_deskew").value)
        self.ego_min = float(p("ego_min_range").value)
        self.gimbal_frame = str(p("gimbal_frame").value)
        self.robot_frame = str(p("robot_frame").value)
        self.buf_sec = float(p("imu_buffer_sec").value)
        self.n_out_cfg = int(p("n_out_beams").value)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.imu_t = []
        self.imu_wz = []
        self.gy_t = []
        self.gy_val = []
        self.scans = 0
        self.dropped_ego = 0
        self.total_beams = 0
        self.last_dtheta = 0.0
        self.tf_ok = 0
        self.tf_fail = 0

        self.pub = self.create_publisher(
            LaserScan, str(p("out_topic").value), 10)
        self.create_subscription(
            LaserScan, str(p("in_topic").value), self.on_scan, 10)
        self.create_subscription(
            Imu, str(p("imu_topic").value), self.on_imu, 200)
        self.create_subscription(
            JointState, str(p("joint_topic").value), self.on_joint, 200)

        self.get_logger().info(
            f"deskew_node | deskew={self.enable_deskew} "
            f"ego_min_range={self.ego_min}m")

    def on_imu(self, m):
        t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        self.imu_t.append(t)
        self.imu_wz.append(m.angular_velocity.z)
        if len(self.imu_t) > 2 and self.imu_t[-1] - self.imu_t[0] > self.buf_sec:
            cut = 0
            while cut < len(self.imu_t) - 2 and \
                    self.imu_t[-1] - self.imu_t[cut] > self.buf_sec:
                cut += 1
            self.imu_t = self.imu_t[cut:]
            self.imu_wz = self.imu_wz[cut:]

    def on_joint(self, m):
        try:
            i = list(m.name).index("gimbal_yaw")
        except ValueError:
            return
        t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        self.gy_t.append(t)
        self.gy_val.append(float(m.position[i]))
        if len(self.gy_t) > 2 and self.gy_t[-1] - self.gy_t[0] > self.buf_sec:
            cut = 0
            while cut < len(self.gy_t) - 2 and \
                    self.gy_t[-1] - self.gy_t[cut] > self.buf_sec:
                cut += 1
            self.gy_t = self.gy_t[cut:]
            self.gy_val = self.gy_val[cut:]

    def _gimbal_yaw(self, stamp):
        """优先用 /joint_states 的云台角（带时间戳、无 TF 时间边界问题）。

        真机上云台角一般来自云台控制器状态话题，这里与此一致。
        TF 仅作兜底。
        """
        t = stamp.sec + stamp.nanosec * 1e-9
        if len(self.gy_t) >= 2:
            return float(np.interp(t, np.asarray(self.gy_t),
                                   np.unwrap(np.asarray(self.gy_val))))
        for attempt in range(2):
            try:
                tr = self.tf_buffer.lookup_transform(
                    self.robot_frame, self.gimbal_frame, stamp,
                    timeout=Duration(seconds=0.2))
                q = tr.transform.rotation
                self.tf_ok += 1
                return math.atan2(2 * (q.w * q.z + q.x * q.y),
                                  1 - 2 * (q.y * q.y + q.z * q.z))
            except Exception as e:  # noqa: BLE001
                if attempt == 0:
                    continue
                self.tf_fail += 1
                if self.tf_fail <= 3:
                    self.get_logger().warn(
                        f"云台 TF 查询失败（第 {self.tf_fail} 次）：{e}")
        # 兜底：取最新可用变换
        try:
            tr = self.tf_buffer.lookup_transform(
                self.robot_frame, self.gimbal_frame, Time())
            q = tr.transform.rotation
            return math.atan2(2 * (q.w * q.z + q.x * q.y),
                              1 - 2 * (q.y * q.y + q.z * q.z))
        except Exception:
            return 0.0

    def on_scan(self, msg):
        n = len(msg.ranges)
        alpha = msg.angle_min + np.arange(n) * msg.angle_increment
        rng = np.array(msg.ranges, dtype=np.float64)
        rng[~np.isfinite(rng)] = np.inf

        t_ref = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        period = msg.scan_time if msg.scan_time > 0 else 0.1
        dt_beam = (msg.time_increment if msg.time_increment > 0
                   else period / n)
        # 时间戳 = 扫描起始时刻（真实驱动惯例），各束在其后 dt_beam 递增
        t_beam = t_ref + (np.arange(n) + 0.5) * dt_beam

        # ---- IMU 积分得到每束相对"帧起始时刻"的旋转量 ----
        #
        # 下游（Nav2/RViz）用消息时间戳 t_ref 的 TF 来解释这帧数据，
        # 所以必须把每束归算到 t_ref 时刻的姿态：
        #     alpha_out = alpha + ∫_{t_ref}^{t_i} ω_z dt
        # 参考时刻若取错（例如取帧结束时刻），会整体多/少补偿一帧的转角
        # （180°/s、10Hz 时是 18°，足以让误差翻倍）。
        if self.enable_deskew and len(self.imu_t) >= 3:
            ts = np.asarray(self.imu_t)
            wz = np.asarray(self.imu_wz)
            cum = np.concatenate([[0.0], np.cumsum(
                0.5 * (wz[1:] + wz[:-1]) * np.diff(ts))])
            c0 = np.interp(t_ref, ts, cum)
            dtheta = np.interp(t_beam, ts, cum) - c0
            self.last_dtheta = float(dtheta[-1] - dtheta[0])
        else:
            dtheta = np.zeros(n)

        # ---- 去畸变：方位角加积分量 ----
        alpha_corr = alpha + dtheta

        # ---- 自遮挡滤波（第二道防线；主滤波应在三维点云阶段完成） ----
        keep = np.isfinite(rng) & (rng >= self.ego_min)
        self.total_beams += n
        self.dropped_ego += int((np.isfinite(rng) & (rng < self.ego_min)).sum())

        # ---- 转到 base_footprint 系并重排到均匀栅格 ----
        gy = self._gimbal_yaw(msg.header.stamp)
        beta = gy - alpha_corr                      # 倒置使方位角取负
        beta = (beta + math.pi) % (2 * math.pi) - math.pi

        n_out = self.n_out_cfg if self.n_out_cfg > 0 else n
        out_inc = 2 * math.pi / n_out
        bins = np.floor((beta + math.pi) / out_inc).astype(np.int64)
        bins = np.clip(bins, 0, n_out - 1)

        out = np.full(n_out, np.inf)
        idx = np.nonzero(keep)[0]
        if len(idx):
            np.minimum.at(out, bins[idx], rng[idx])

        m = LaserScan()
        m.header.stamp = msg.header.stamp
        m.header.frame_id = self.robot_frame
        m.angle_min = -math.pi
        m.angle_max = math.pi - out_inc
        m.angle_increment = out_inc
        m.scan_time = period
        m.time_increment = period / n_out
        m.range_min = msg.range_min
        m.range_max = msg.range_max
        m.ranges = out.astype(np.float32).tolist()
        self.pub.publish(m)

        self.scans += 1
        if self.scans % 50 == 0:
            pct = (self.dropped_ego / max(1, self.total_beams)) * 100
            self.get_logger().info(
                f"scans={self.scans} | 丢弃自车点 {pct:.1f}% | "
                f"本帧云台转过 {math.degrees(self.last_dtheta):.1f}° | "
                f"TF ok={self.tf_ok} fail={self.tf_fail}")


def main():
    rclpy.init()
    node = DeskewNode()
    # 必须是多线程执行器：lookup_transform 带超时等待时会阻塞当前线程，
    # 单线程下 TF 订阅回调无法执行，缓冲区拿不到新数据，
    # 查询必然超时并退化到过期变换（实测滞后 0.16 s，180°/s 即 29° 误差）。
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
