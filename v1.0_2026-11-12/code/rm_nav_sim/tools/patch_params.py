#!/usr/bin/env python3
"""从 nav2_bringup 默认参数生成一份适配 grid_sim 的参数文件。"""
import argparse
import os

import yaml

DEFAULT = "/opt/ros/humble/share/nav2_bringup/params/nav2_params.yaml"


def iter_params(node):
    """递归找出所有 ros__parameters 字典（Nav2 的参数层级不统一）。"""
    if isinstance(node, dict):
        prm = node.get("ros__parameters")
        if isinstance(prm, dict):
            yield prm
        for v in node.values():
            yield from iter_params(v)


def find_node(p, key):
    """找到某个节点名下的 ros__parameters（可能套一层同名键）。"""
    node = p.get(key, {})
    if "ros__parameters" in node:
        return node["ros__parameters"]
    inner = node.get(key, {})
    if isinstance(inner, dict) and "ros__parameters" in inner:
        return inner["ros__parameters"]
    for prm in iter_params(node):
        return prm
    raise KeyError(key)


def replace_frame(node, old="base_link", new="base_footprint"):
    """Nav2 默认参数写死了 base_link，这里全局替换成我们的基座系。"""
    if isinstance(node, dict):
        for k, v in node.items():
            if v == old:
                node[k] = new
            else:
                replace_frame(v, old, new)
    elif isinstance(node, list):
        for i, v in enumerate(node):
            if v == old:
                node[i] = new
            else:
                replace_frame(v, old, new)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", required=True)
    ap.add_argument("--map", default="")
    ap.add_argument("--start-x", type=float, default=0.0)
    ap.add_argument("--start-y", type=float, default=-1.8)
    ap.add_argument("--start-yaw", type=float, default=0.0)
    ap.add_argument("--radius", type=float, default=0.28)
    args = ap.parse_args()

    with open(DEFAULT, encoding="utf-8") as f:
        p = yaml.safe_load(f)

    # 1) 全部节点关掉 sim time（我们用墙钟）
    for prm in iter_params(p):
        prm["use_sim_time"] = False

    # 2) 地图
    if args.map:
        find_node(p, "map_server")["yaml_filename"] = args.map

    # 3) AMCL
    amcl = find_node(p, "amcl")
    amcl.update({
        "set_initial_pose": True,
        "initial_pose": {"x": args.start_x, "y": args.start_y,
                         "z": 0.0, "yaw": args.start_yaw},
        "scan_topic": "/scan",
        "base_frame_id": "base_footprint",
        "odom_frame_id": "odom",
        "global_frame_id": "map",
        "update_min_d": 0.02,
        "update_min_a": 0.02,
        "laser_max_range": 8.0,
        # 仿真里程计是完美的：把运动模型噪声调小，别让 AMCL 去"纠正"它
        "alpha1": 0.02, "alpha2": 0.02, "alpha3": 0.02,
        "alpha4": 0.02, "alpha5": 0.02,
        "max_particles": 4000,
        "min_particles": 800,
    })

    # 4) 任何带 robot_radius 的参数块 = costmap，统一改基座系和膨胀
    for prm in iter_params(p):
        if "robot_radius" in prm:
            prm["robot_base_frame"] = "base_footprint"
            prm["robot_radius"] = args.radius
            inf = prm.get("inflation_layer")
            if isinstance(inf, dict):
                inf["inflation_radius"] = 0.32
                inf["cost_scaling_factor"] = 3.5

    # 5) 控制器速度与里程计
    cs = find_node(p, "controller_server")
    cs["odom_topic"] = "/odom"
    cs["controller_frequency"] = 15.0
    fp = cs.get("FollowPath", {})
    fp.update({"plugin": "dwb_core::DWBLocalPlanner",
               "max_vel_x": 0.6, "max_vel_theta": 1.0,
               "min_speed_xy": 0.0, "max_speed_xy": 0.6,
               "acc_lim_x": 1.2, "acc_lim_theta": 1.8,
               "trans_stopped_velocity": 0.05})
    cs["FollowPath"] = fp

    # 5b) velocity_smoother 的默认上限是 0.26 m/s，必须放开（这是最容易踩的坑）
    vs = find_node(p, "velocity_smoother")
    vs["max_velocity"] = [1.0, 0.0, 2.0]
    vs["min_velocity"] = [-0.5, 0.0, -2.0]
    vs["max_accel"] = [2.0, 0.0, 4.0]
    vs["max_decel"] = [-2.0, 0.0, -4.0]

    # 6) bt_navigator 的坐标系（默认 base_link）
    bt = find_node(p, "bt_navigator")
    bt["global_frame"] = "map"
    bt["robot_base_frame"] = "base_footprint"

    # 7) 兜底：全文替换残留的 base_link
    replace_frame(p)

    os.makedirs(os.path.dirname(os.path.abspath(args.out)), exist_ok=True)
    with open(args.out, "w", encoding="utf-8") as f:
        yaml.safe_dump(p, f, allow_unicode=True, sort_keys=False)
    print(f"[patch_params] wrote {args.out}")


if __name__ == "__main__":
    main()
