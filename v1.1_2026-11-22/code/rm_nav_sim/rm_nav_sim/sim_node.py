#!/usr/bin/env python3
"""2D 网格导航仿真器（支持云台载倒置雷达）。

v1.1 新增：
  - 雷达可装在云台偏航轴（lidar_mount=gimbal），随云台旋转
  - 支持倒置安装（lidar_roll_deg=180）
  - 逐束时间戳：模拟单帧扫描期间的旋转畸变
  - 车体自遮挡：底盘多边形（base 系）+ 云台载荷多边形（laser 系）
  - 发布 /livox/imu，供去畸变节点按雷达内置 IMU 做旋转补偿

发布：
  /odom, /ground_truth/pose, TF(odom->base_footprint->gimbal_yaw->laser)
  /scan_raw   原始扫描（含畸变与自遮挡），frame_id = laser
  /livox/imu  雷达内置 IMU，frame_id = laser
订阅：
  /cmd_vel, /cmd_gimbal_rate
"""
import math
import threading
from collections import deque

import numpy as np
import rclpy
import yaml
from geometry_msgs.msg import PoseStamped, TransformStamped, Twist
from nav_msgs.msg import Odometry
from PIL import Image
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import Imu, JointState, LaserScan
from std_msgs.msg import Float64
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster


def yaw_to_quat(y):
    return (0.0, 0.0, math.sin(y * 0.5), math.cos(y * 0.5))


class GridSim(Node):
    def __init__(self):
        super().__init__("grid_sim")
        defaults = (
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
            ("robot_radius", 0.28),
            ("laser_height", 0.20),
            ("cmd_timeout", 0.5),
            # ---- v1.1 雷达/云台 ----
            ("lidar_mount", "chassis"),        # chassis | gimbal
            ("lidar_roll_deg", 0.0),           # 180 = 倒置
            ("lidar_height", 0.50),            # 光心离地高度
            ("gimbal_height", 0.14),           # 云台偏航轴相对 base 的高度
            ("gimbal_rate_deg", 0.0),          # 固定云台角速度
            ("gimbal_mode", "spin"),           # spin | lock
            ("enable_distortion", True),       # 逐束时间戳
            ("enable_ego", False),             # 是否模拟车体自遮挡
            ("ego_filter", True),              # 是否在取最近点之前剔除自车点
            ("ego_range", 0.45),               # 自车滤波距离门限
            ("chassis_poly_xy", [0.32, 0.26, 0.32, -0.26,
                                 -0.32, -0.26, -0.32, 0.26]),
            ("payload_poly_xy", [0.38, 0.07, 0.38, -0.07,
                                 0.13, -0.07, 0.13, 0.07]),
            ("publish_imu", True),
            ("imu_hz", 200.0),
            # ---- 安装位姿（底盘系）：偏心 + 前倾 ----
            ("lidar_offset_x", 0.0),      # 相对旋转中心前移
            ("lidar_offset_y", 0.0),      # 相对旋转中心左移
            ("lidar_pitch_deg", 0.0),     # 前倾角，正=朝下俯
            ("lidar_fov_min_deg", -7.0),  # Mid-360 垂直视场下沿
            ("lidar_fov_max_deg", 52.0),  # 垂直视场上沿
            ("chassis_spin_dps", 0.0),    # 底盘自转角速度，360=小陀螺
            # 虚拟基座系：把小陀螺的旋转从 TF/速度指令里剥掉，
            # 让 Nav2 看到一个"不转的基座"（等价于北极熊的 fake_vel_transform）
            ("spin_vel_transform", True),
        )
        for name, val in defaults:
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

        # ---- 雷达 / 云台 ----
        self.mount = str(p("lidar_mount").value)
        self.roll = math.radians(float(p("lidar_roll_deg").value))
        self.laser_h = float(p("lidar_height").value)
        self.gimbal_h = float(p("gimbal_height").value)
        self.gimbal_rate = math.radians(float(p("gimbal_rate_deg").value))
        self.gimbal_mode = str(p("gimbal_mode").value)
        self.enable_distortion = bool(p("enable_distortion").value)
        self.enable_ego = bool(p("enable_ego").value)
        self.ego_filter = bool(p("ego_filter").value)
        self.ego_range = float(p("ego_range").value)
        self.publish_imu = bool(p("publish_imu").value)

        # 安装位姿：偏心 / 前倾 / 垂直视场
        self.lofs_x = float(p("lidar_offset_x").value)
        self.lofs_y = float(p("lidar_offset_y").value)
        self.pitch = math.radians(float(p("lidar_pitch_deg").value))
        self.fov_min = float(p("lidar_fov_min_deg").value)
        self.fov_max = float(p("lidar_fov_max_deg").value)
        # 前倾后，水平面只在一部分方位角内落在视场里。
        # 条件：atan(cos(az)·tan α) ≥ fov_min  →  cos(az) ≥ tan(fov_min)/tan(α)
        if abs(self.pitch) > 1e-6 and abs(math.sin(self.pitch)) > 1e-6:
            _lim = math.tan(math.radians(self.fov_min)) / math.tan(self.pitch)
            self._cover_lim = max(-1.0, min(1.0, _lim))
        else:
            self._cover_lim = -1.0
        self.blind_deg = 0.0
        if self._cover_lim > -1.0:
            _half = math.degrees(math.acos(self._cover_lim))
            self.blind_deg = 360.0 - 2.0 * _half
            self._cover_half = _half
        else:
            self._cover_half = 180.0
        self.spin_rate = math.radians(float(p("chassis_spin_dps").value))
        self.spin_vel_transform = bool(p("spin_vel_transform").value)
        self.spin_accum = 0.0     # 累计自转角，用于构造虚拟基座系

        cp = p("chassis_poly_xy").value
        self.chassis_poly = (np.array(cp, dtype=float).reshape(-1, 2)
                             if len(cp) >= 6 else None)
        pp = p("payload_poly_xy").value
        self.payload_poly = (np.array(pp, dtype=float).reshape(-1, 2)
                             if len(pp) >= 6 else None)

        self.gimbal_yaw = 0.0
        self.last_ego_hits = 0
        self.gimbal_cmd_rate = None
        self.gimbal_cmd_time = self.get_clock().now()
        self.last_yaw_rate = 0.0

        # 车身轮廓采样点
        self.footprint = [(0.0, 0.0)] + [
            (self.radius * math.cos(a), self.radius * math.sin(a))
            for a in np.linspace(0, 2 * math.pi, 8, endpoint=False)
        ]

        # 射线投射预计算
        step = float(p("ray_step").value)
        self.ds = np.arange(0.0, self.rmax, step)
        self.angle_min = -math.pi
        self.angle_inc = 2 * math.pi / self.n
        self.angle_max = math.pi - self.angle_inc
        self.alphas = self.angle_min + np.arange(self.n) * self.angle_inc

        # 位姿/云台历史（供逐束时间戳查询）。
        # 必须是"单个 deque 存元组"：物理线程与扫描线程并发，
        # 若用多个独立 deque 逐个 append，扫描线程会读到长度不一致的中间状态，
        # 导致 np.interp 抛 "fp and xp are not of the same length"。
        self.hist = deque(maxlen=600)

        self.lock = threading.Lock()
        self.cmd_v = 0.0
        self.cmd_vy = 0.0
        self.cmd_w = 0.0
        self.cmd_time = self.get_clock().now()
        self._last_t = self.get_clock().now()

        self.odom_pub = self.create_publisher(Odometry, "/odom", 10)
        self.scan_pub = self.create_publisher(LaserScan, "/scan_raw", 10)
        self.gt_pub = self.create_publisher(PoseStamped, "/ground_truth/pose", 10)
        self.imu_pub = self.create_publisher(Imu, "/livox/imu", 10)
        # 云台角以 JointState 发布（带时间戳）。真机上这个角度通常来自云台
        # 控制器的状态话题；用话题而不是 TF 查询，可避免 TF 时间边界的脆弱性。
        self.joint_pub = self.create_publisher(JointState, "/joint_states", 10)
        self.tf_broad = TransformBroadcaster(self)
        self.static_broad = StaticTransformBroadcaster(self)
        self._publish_static_tf()

        cbg_phys = MutuallyExclusiveCallbackGroup()
        cbg_scan = MutuallyExclusiveCallbackGroup()
        cbg_sub = MutuallyExclusiveCallbackGroup()
        self.create_timer(1.0 / float(p("physics_hz").value), self.on_physics,
                          callback_group=cbg_phys)
        self.create_timer(1.0 / self.scan_hz, self.on_scan, callback_group=cbg_scan)
        self.create_timer(1.0 / float(p("imu_hz").value), self.on_imu_tick,
                          callback_group=cbg_scan)
        self.create_subscription(Twist, "/cmd_vel", self.on_cmd, 10,
                                 callback_group=cbg_sub)
        self.create_subscription(Float64, "/cmd_gimbal_rate", self.on_gimbal, 10,
                                 callback_group=cbg_sub)

        self.ticks = 0
        self.get_logger().info(
            f"grid_sim v1.1 | map {self.W}x{self.H}@{self.res}m | beams={self.n} "
            f"| mount={self.mount} roll={math.degrees(self.roll):.0f}° "
            f"h={self.laser_h:.2f}m | gimbal={math.degrees(self.gimbal_rate):.0f}°/s "
            f"| distortion={self.enable_distortion} ego={self.enable_ego}")

    # ---------------- TF / 订阅 ----------------
    def _publish_static_tf(self):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        if self.mount == "gimbal":
            t.header.frame_id = "gimbal_yaw"
        else:
            t.header.frame_id = "base_footprint"
        t.child_frame_id = "laser"
        t.transform.translation.x = self.lofs_x
        t.transform.translation.y = self.lofs_y
        t.transform.translation.z = self.laser_h
        # 只应用 roll（倒置180°时 XY 平面仍是水平的），**不应用 pitch**。
        # 原因：pitch 会把 laser 系的 XY 平面掰出水平面，而 2D LaserScan 只能
        # 表达水平面上的测量。前倾的几何效果已经由 _cover_lim 盲区掩码建模，
        # 若再在 TF 里转一次，等于把倾斜算了两次，扫描会被拉歪。
        t.transform.rotation.x = math.sin(self.roll * 0.5)
        t.transform.rotation.w = math.cos(self.roll * 0.5)
        self.static_broad.sendTransform([t])

    def on_cmd(self, msg):
        self.cmd_v = msg.linear.x
        self.cmd_vy = msg.linear.y
        self.cmd_w = msg.angular.z
        self.cmd_time = self.get_clock().now()

    def on_gimbal(self, msg):
        self.gimbal_cmd_rate = float(msg.data)
        self.gimbal_cmd_time = self.get_clock().now()

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

    def _gimbal_rate_now(self, now):
        if self.mount != "gimbal" or self.gimbal_mode == "lock":
            return 0.0
        age = (now - self.gimbal_cmd_time).nanoseconds * 1e-9
        if self.gimbal_cmd_rate is not None and age < self.timeout:
            return self.gimbal_cmd_rate
        return self.gimbal_rate

    # ---------------- 物理 ----------------
    def on_physics(self):
        now = self.get_clock().now()
        with self.lock:
            dt = (now - self._last_t).nanoseconds * 1e-9
            self._last_t = now
        dt = min(max(dt, 1e-4), 0.20)

        age = (now - self.cmd_time).nanoseconds * 1e-9
        if age < self.timeout:
            v, vy, w = self.cmd_v, self.cmd_vy, self.cmd_w
        else:
            v, vy, w = 0.0, 0.0, 0.0

        gy_rate = self._gimbal_rate_now(now)
        # 虚拟基座朝向 = 物理朝向 - 累计自角。
        # cmd_vel 表达在虚拟基座里，若不剥掉自转，同一个(0.6,0)会让车画圈。
        th_cmd = self.th - (self.spin_accum if self.spin_vel_transform else 0.0)
        vx_w = v * math.cos(th_cmd) - vy * math.sin(th_cmd)
        vy_w = v * math.sin(th_cmd) + vy * math.cos(th_cmd)

        with self.lock:
            nx = self.x + vx_w * dt
            ny = self.y + vy_w * dt
            nth = self.th + (w + self.spin_rate) * dt
            if not self._blocked(nx, ny):
                self.x, self.y, self.th = nx, ny, nth
            else:
                if not self._blocked(nx, self.y):
                    self.x = nx
                if not self._blocked(self.x, ny):
                    self.y = ny
                self.th = nth
            self.gimbal_yaw += gy_rate * dt
            if self.gimbal_yaw > math.pi:
                self.gimbal_yaw -= 2 * math.pi
            elif self.gimbal_yaw < -math.pi:
                self.gimbal_yaw += 2 * math.pi
            self.spin_accum += self.spin_rate * dt
            x, y, th, gy = self.x, self.y, self.th, self.gimbal_yaw

        self.hist.append((now.nanoseconds * 1e-9, x, y, th, gy))
        self.last_yaw_rate = w

        stamp = now.to_msg()
        th_v = th - self.spin_accum if self.spin_vel_transform else th
        self._pub_odom(stamp, v, vy, w, x, y, th_v)
        if self.mount == "gimbal":
            self._pub_gimbal_tf(stamp, gy)
        self.ticks += 1
        if self.ticks % 2 == 0:
            self._pub_tf(stamp, x, y, th_v)

    def on_imu_tick(self):
        """雷达内置 IMU 与云台角以 200 Hz 发布（与真机一致）。"""
        now = self.get_clock().now()
        gy_rate = self._gimbal_rate_now(now)
        if self.publish_imu:
            self._pub_imu(now.to_msg(), self.last_yaw_rate, gy_rate)

        js = JointState()
        js.header.stamp = now.to_msg()
        js.name = ["gimbal_yaw"]
        with self.lock:
            js.position = [self.gimbal_yaw]
        self.joint_pub.publish(js)

    def _pub_odom(self, stamp, v, vy, w, x, y, th):
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
        m.twist.twist.linear.y = vy
        m.twist.twist.angular.z = w
        self.odom_pub.publish(m)

        gt = PoseStamped()
        gt.header.stamp = stamp
        gt.header.frame_id = "map"
        gt.pose = m.pose.pose
        self.gt_pub.publish(gt)

    def _pub_gimbal_tf(self, stamp, gy):
        t = TransformStamped()
        t.header.stamp = stamp
        t.header.frame_id = "base_footprint"
        t.child_frame_id = "gimbal_yaw"
        t.transform.translation.z = self.gimbal_h
        q = yaw_to_quat(gy)
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]
        self.tf_broad.sendTransform([t])

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

    def _pub_imu(self, stamp, yaw_rate, gy_rate):
        """雷达内置 IMU。倒置后 z 轴翻转，故角速度 z 分量取负。

        注意：MID-360 内置 IMU 的加速度单位是 g（静止模长≈1.0），不是 m/s²。
        倒置时 z 分量约为 +1.0 g。
        """
        m = Imu()
        m.header.stamp = stamp
        m.header.frame_id = "laser"
        m.angular_velocity.z = -(yaw_rate + gy_rate)
        m.linear_acceleration.z = -math.cos(self.roll)
        m.orientation_covariance[0] = -1.0
        self.imu_pub.publish(m)

    # ---------------- 射线-多边形求交 ----------------
    @staticmethod
    def _ray_poly(dirs, poly):
        """dirs: (n,2) 单位向量；poly: (m,2)。返回 (n,) 距离，未命中为 inf。"""
        if poly is None or len(poly) < 3:
            return np.full(len(dirs), np.inf)
        A = poly
        B = np.roll(poly, -1, axis=0)
        E = B - A
        d = dirs
        denom = d[:, 0:1] * E[None, :, 1] - d[:, 1:2] * E[None, :, 0]
        num_t = A[:, 0] * E[:, 1] - A[:, 1] * E[:, 0]
        num_s = A[None, :, 0] * d[:, 1:2] - A[None, :, 1] * d[:, 0:1]
        with np.errstate(divide="ignore", invalid="ignore"):
            t = np.where(np.abs(denom) > 1e-12, num_t[None, :] / denom, np.inf)
            s = np.where(np.abs(denom) > 1e-12, num_s / denom, np.inf)
        ok = (t > 0) & (s >= 0) & (s <= 1)
        t = np.where(ok, t, np.inf)
        return t.min(axis=1)

    # ---------------- 扫描 ----------------
    def on_scan(self):
        now = self.get_clock().now()
        t_end = now.nanoseconds * 1e-9
        period = 1.0 / self.scan_hz
        t0 = t_end - period

        n = self.n
        if self.enable_distortion:
            t_beam = t0 + (np.arange(n) + 0.5) / n * period
        else:
            t_beam = np.full(n, t_end)

        h = np.asarray(self.hist, dtype=np.float64)
        if len(h) < 2:
            return
        ht = h[:, 0]
        xb = np.interp(t_beam, ht, h[:, 1])
        yb = np.interp(t_beam, ht, h[:, 2])
        tb = np.interp(t_beam, ht, h[:, 3])
        gb = np.interp(t_beam, ht, h[:, 4])

        # 雷达装在底盘上偏离旋转中心时，射线原点要跟着转（这是自转时的主要误差源）
        if self.lofs_x or self.lofs_y:
            ct, st = np.cos(tb), np.sin(tb)
            ox_w = self.lofs_x * ct - self.lofs_y * st
            oy_w = self.lofs_x * st + self.lofs_y * ct
            xb = xb + ox_w
            yb = yb + oy_w

        alpha = self.alphas                       # laser 系方位角
        # 虚拟基座系：把自转从“车头朝向”里剥掉，扫描按不转的基座表达。
        # 否则 TF 用虚拟朝向、扫描却按物理朝向算，点云会整体偏移 spin_accum。
        if self.spin_vel_transform and abs(self.spin_rate) > 1e-9:
            tb = tb - (self.spin_accum - self.spin_rate * (t_end - t_beam))
        # 倒置（绕 x 转 180°）使 y 反向 → 方位角取负
        gim_az = -alpha
        beta = gb + gim_az                        # base 系方位角
        psi = tb + beta                           # 世界系方位角

        cospsi = np.cos(psi)
        sinpsi = np.sin(psi)

        # 地图射线投射（每束独立的原点与方向）
        xs = self.ds[:, None] * cospsi[None, :] + xb[None, :]
        ys = self.ds[:, None] * sinpsi[None, :] + yb[None, :]
        gxi = np.rint((xs - self.ox) / self.res).astype(np.int32)
        gyi = np.rint((ys - self.oy) / self.res).astype(np.int32)
        valid = (gxi >= 0) & (gxi < self.W) & (gyi >= 0) & (gyi < self.H)
        np.clip(gxi, 0, self.W - 1, out=gxi)
        np.clip(gyi, 0, self.H - 1, out=gyi)
        flat = (self.H - 1 - gyi) * self.W + gxi
        hit = self.occ_flat[flat]
        hit &= valid
        any_hit = hit.any(axis=0)
        first = hit.argmax(axis=0)
        rng = np.where(any_hit, self.ds[first], np.inf).astype(np.float64)

        # 前倾导致的水平盲区：视场下沿只有 -7°，倾角一大，后背方位就看不到水平面
        if self._cover_lim > -1.0:
            covered = np.cos(alpha) >= self._cover_lim
            rng = np.where(covered, rng, np.inf)

        # 车体自遮挡
        #
        # 关键：三维点云转二维扫描时是"每个方位取最近点"（pointcloud_to_laserscan
        # 的行为）。自车回波约 0.3 m，永远比环境近，所以
        #     r = min(r_env, r_ego)
        # 会让整圈都变成自车轮廓，环境信息被覆盖且无法恢复。
        #
        # 正确做法：在"取最近点"之前就把自车点剔除（即三维阶段的 body filter，
        # 或 pointcloud_to_laserscan 的 range_min —— 它是在取 min 之前跳过近点的）。
        #   ego_filter=True  → 模拟正确管线：丢掉近点，只保留环境
        #   ego_filter=False → 模拟忘记处理：整圈被自车轮廓占满
        if self.enable_ego:
            d_base = np.stack([np.cos(beta), np.sin(beta)], axis=1)
            # 雷达偏心时，车体在雷达系里要反向平移
            poly_c = self.chassis_poly
            if poly_c is not None and (self.lofs_x or self.lofs_y):
                poly_c = poly_c - np.array([self.lofs_x, self.lofs_y])
            r_chassis = self._ray_poly(d_base, poly_c)
            d_laser = np.stack([np.cos(alpha), np.sin(alpha)], axis=1)
            r_payload = self._ray_poly(d_laser, self.payload_poly)
            r_ego = np.minimum(r_chassis, r_payload)
            self.last_ego_hits = int(np.isfinite(r_ego).sum())
            if self.ego_filter:
                # 自车点（含过近的回波）在取最近点之前被丢弃
                rng = np.where(rng >= self.ego_range, rng, np.inf)
            else:
                rng = np.minimum(rng, r_ego)

        # 噪声只加在有效回波上；无回波保持 inf（不存在的远墙）
        finite = np.isfinite(rng)
        if finite.any():
            rng[finite] += np.random.normal(0.0, self.noise, size=int(finite.sum()))
            np.clip(rng, 0.05, self.rmax, out=rng, where=finite)

        m = LaserScan()
        # 时间戳取"扫描起始时刻"，与真实雷达驱动惯例一致（Mid-360 的 timebase
        # 也是本帧首点时间）。这样下游按该时刻查 TF 一定落在过去、必然可查；
        # 若戳"发布时刻"，查询会因超前而失败，去畸变基准随之出错。
        sec = int(t0)
        m.header.stamp.sec = sec
        m.header.stamp.nanosec = int((t0 - sec) * 1e9)
        m.header.frame_id = "laser"
        m.angle_min = self.angle_min
        m.angle_max = self.angle_max
        m.angle_increment = self.angle_inc
        m.scan_time = period
        m.time_increment = period / n
        m.range_min = 0.10
        m.range_max = self.rmax
        m.ranges = rng.astype(np.float32).tolist()
        self.scan_pub.publish(m)


def main():
    rclpy.init()
    node = GridSim()
    ex = MultiThreadedExecutor(num_threads=4)
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
