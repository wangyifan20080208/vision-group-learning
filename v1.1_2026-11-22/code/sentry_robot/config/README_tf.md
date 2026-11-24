# 雷达挂云台（倒置同轴）静态 TF
#
# 坐标系链：
#   map → odom → base_footprint → gimbal_yaw → laser
#
# gimbal_yaw 由电控/云台驱动发布（动态）。这里只发两段静态的：
#   base_footprint → gimbal_yaw   云台偏航轴相对车体的位置
#   gimbal_yaw     → laser        雷达相对云台偏航轴的位置与姿态（倒置）
#
# ⚠️ 下面这些数值是「待填」的占位值。机械装配完成后必须实测：
#   1. 用卷尺量 gimbal 中心相对 base_footprint 的 x/y/z
#   2. 量雷达光心相对云台偏航轴的高度
#   3. 用水平仪确认同轴偏差 < 5 mm
#
# 倒置 = 绕 x 轴 180°：
#   roll=π → 四元数 (x, y, z, w) = (1, 0, 0, 0)
#
# 如果实测发现雷达标称坐标系与 Mid-360 不同（Mid-360s 的连接器位置改过），
# 还要叠加一个绕自身 z 轴的 yaw 修正（参数 laser_yaw_offset_deg）。
