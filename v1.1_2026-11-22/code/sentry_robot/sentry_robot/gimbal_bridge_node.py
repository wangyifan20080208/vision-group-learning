#!/usr/bin/env python3
"""云台角度桥：串口 → ROS TF + 话题。

数据链：
    GM6020(yaw) --CAN--> 云台 C 板 --UART--> 本节点 --TF--> base_footprint → gimbal_yaw

为什么需要这个节点
------------------
雷达挂在云台上，云台一转，雷达就跟着转。如果 TF 不知道这件事，
base_footprint 系下的 /scan 会整体旋转 —— 实测 150°/s 时每帧偏 15°，
240°/s 时偏 24°，Nav2 的代价地图会彻底乱掉。

只要把云台角度如实发成 TF，`livox_scan_node` 查 TF 时会自动补偿掉，
下游 Nav2 完全无感。

协议（C 板 → NUC）
------------------
  偏移  长度  内容
  0     1     帧头 0xA5
  1     1     帧头 0x5A
  2     1     负载长度 = 16
  3     1     类型 = 0x01（云台角度）
  4     4     yaw        float32  rad  连续角度（已解零点跳变）
  8     4     pitch      float32  rad  连续角度
  12    4     yaw_rate   float32  rad/s
  16    4     tick_ms    uint32   毫秒时基
  20    1     校验 = 前 20 字节的异或
  共 21 字节，小端

测试模式
--------
没有 C 板时用 simulate:=true，节点自己发一个正弦摆动，
可以先把 ROS 侧和 TF 链验证通。
"""
from __future__ import annotations

import math
import struct
import time

import rclpy
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from tf2_ros import TransformBroadcaster

HEAD = b"\xA5\x5A"
PAYLOAD_LEN = 16
TYPE_GIMBAL = 0x01
FRAME_LEN = 2 + 1 + 1 + PAYLOAD_LEN + 1      # 21


def quat_from_rpy(roll: float, pitch: float, yaw: float):
    """RPY(rad) → 四元数 (x, y, z, w)。"""
    cr, sr = math.cos(roll * 0.5), math.sin(roll * 0.5)
    cp, sp = math.cos(pitch * 0.5), math.sin(pitch * 0.5)
    cy, sy = math.cos(yaw * 0.5), math.sin(yaw * 0.5)
    return (
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    )


class GimbalBridge(Node):
    def __init__(self):
        super().__init__("gimbal_bridge")

        d = [
            ("port", "/dev/ttyUSB0"),
            ("baudrate", 921600),
            ("simulate", False),          # 无 C 板时用正弦信号自测
            ("sim_amplitude_deg", 90.0),
            ("sim_period", 6.0),
            ("parent_frame", "base_footprint"),
            ("child_frame", "gimbal_yaw"),
            # 云台偏航轴相对车体的位置（装配后实测）
            ("offset_x", 0.0),
            ("offset_y", 0.0),
            ("offset_z", 0.14),
            ("publish_tf", True),
            ("timeout_warn_sec", 1.0),
        ]
        for n, v in d:
            self.declare_parameter(n, v)
        p = self.get_parameter

        self.port = str(p("port").value)
        self.baud = int(p("baudrate").value)
        self.simulate = bool(p("simulate").value)
        self.sim_amp = math.radians(float(p("sim_amplitude_deg").value))
        self.sim_T = float(p("sim_period").value)
        self.parent = str(p("parent_frame").value)
        self.child = str(p("child_frame").value)
        self.off = (float(p("offset_x").value),
                    float(p("offset_y").value),
                    float(p("offset_z").value))
        self.pub_tf = bool(p("publish_tf").value)

        self.serial = None
        self.buf = b""
        self.last_rx = 0.0
        self.n_rx = 0
        self.last_log = self.get_clock().now()
        self.yaw = 0.0
        self.pitch = 0.0
        self.rate = 0.0
        self.t_start = time.time()

        self.tf = TransformBroadcaster(self)
        self.state_pub = self.create_publisher(
            Float32MultiArray, "/gimbal/state", 10)

        if not self.simulate:
            self._open_serial()

        hz = 200.0 if not self.simulate else 100.0
        self.create_timer(1.0 / hz, self.tick)
        self.get_logger().info(
            f"云台角度桥启动 | "
            f"{'仿真' if self.simulate else self.port + ' @ ' + str(self.baud)} "
            f"| {self.parent} → {self.child}")

    # ---------------- 串口 ----------------
    def _open_serial(self):
        try:
            import serial
            self.serial = serial.Serial(self.port, self.baud, timeout=0)
        except Exception as e:              # noqa: BLE001
            self.get_logger().error(f"打开 {self.port} 失败: {e}")
            self.get_logger().error("可用 simulate:=true 先验证 ROS 侧")
            self.serial = None

    def _read(self):
        try:
            n = self.serial.in_waiting
            if n:
                self.buf += self.serial.read(n)
        except Exception:
            return
        # 找帧头 + 校验
        while True:
            i = self.buf.find(HEAD)
            if i < 0:
                self.buf = self.buf[-1:]
                return
            if i > 0:
                self.buf = self.buf[i:]
            if len(self.buf) < FRAME_LEN:
                return
            frame = self.buf[:FRAME_LEN]
            if frame[2] != PAYLOAD_LEN or frame[3] != TYPE_GIMBAL:
                self.buf = self.buf[1:]
                continue
            x = 0
            for b in frame[:FRAME_LEN - 1]:
                x ^= b
            if x != frame[-1]:
                self.buf = self.buf[1:]
                continue
            yaw, pitch, yr, _tick = struct.unpack("<ffff", frame[4:20])
            self.buf = self.buf[FRAME_LEN:]
            self._accept(yaw, pitch, yr)

    def _accept(self, yaw, pitch, yr):
        if not all(map(math.isfinite, (yaw, pitch, yr))):
            return
        self.yaw, self.pitch, self.rate = float(yaw), float(pitch), float(yr)
        self.last_rx = time.time()
        self.n_rx += 1

    # ---------------- 仿真 ----------------
    def _sim(self):
        t = time.time() - self.t_start
        w = 2 * math.pi / self.sim_T
        self.yaw = self.sim_amp * math.sin(w * t)
        self.pitch = 0.0
        self.rate = self.sim_amp * w * math.cos(w * t)

    # ---------------- 主循环 ----------------
    def tick(self):
        if self.simulate:
            self._sim()
        elif self.serial is not None:
            self._read()

        now = self.get_clock().now()
        q = quat_from_rpy(0.0, 0.0, self.yaw)

        if self.pub_tf:
            t = TransformStamped()
            t.header.stamp = now.to_msg()
            t.header.frame_id = self.parent
            t.child_frame_id = self.child
            t.transform.translation.x = self.off[0]
            t.transform.translation.y = self.off[1]
            t.transform.translation.z = self.off[2]
            t.transform.rotation.x = q[0]
            t.transform.rotation.y = q[1]
            t.transform.rotation.z = q[2]
            t.transform.rotation.w = q[3]
            self.tf.sendTransform(t)

        m = Float32MultiArray()
        m.data = [self.yaw, self.pitch, self.rate]
        self.state_pub.publish(m)

        # 状态日志（每秒一次）
        if (now - self.last_log).nanoseconds * 1e-9 > 5.0:
            self.last_log = now
            age = time.time() - self.last_rx if self.last_rx else -1
            if not self.simulate and (age < 0 or age > 1.0):
                self.get_logger().warn(
                    f"已经 {age:.1f}s 没收到 C 板数据"
                    f"（累计 {self.n_rx} 帧）")
            else:
                self.get_logger().info(
                    f"yaw {math.degrees(self.yaw):+7.1f}°  "
                    f"pitch {math.degrees(self.pitch):+6.1f}°  "
                    f"rate {math.degrees(self.rate):+7.1f}°/s  "
                    f"累计 {self.n_rx} 帧")


def main():
    rclpy.init()
    n = GimbalBridge()
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    finally:
        if n.serial is not None:
            n.serial.close()
        n.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
