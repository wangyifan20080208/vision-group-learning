#!/usr/bin/env bash
# 彻底清理所有仿真相关进程。
# 教训：之前漏掉 static_transform_publisher，导致 map->odom 被重复发布，
# TF 冲突 → lifecycle_manager 配置失败 → Nav2 全部卡在 unconfigured。
pkill -f "ros2 launch rm_nav_sim"            2>/dev/null
sleep 0.5
pkill -9 -f "lib/rm_nav_sim/"                    2>/dev/null
pkill -9 -f "rm_nav_sim.sim_node"                2>/dev/null
pkill -9 -f "rm_nav_sim.deskew_node"             2>/dev/null
pkill -9 -f "rm_nav_sim.referee_sim_node"        2>/dev/null
pkill -9 -f "rm_nav_sim.sentry_fsm_node"         2>/dev/null
pkill -9 -f "nav2_lifecycle_manager"             2>/dev/null
pkill -9 -f "nav2_controller"                2>/dev/null
pkill -9 -f "nav2_planner"                   2>/dev/null
pkill -9 -f "nav2_bt_navigator"              2>/dev/null
pkill -9 -f "nav2_behaviors"                 2>/dev/null
pkill -9 -f "nav2_waypoint_follower"         2>/dev/null
pkill -9 -f "nav2_smoother"                  2>/dev/null
pkill -9 -f "nav2_velocity_smoother"         2>/dev/null
pkill -9 -f "nav2_map_server"                2>/dev/null
pkill -9 -f "static_transform_publisher"     2>/dev/null
pkill -9 -f "rviz2"                          2>/dev/null
pkill -9 -f "pose_record.py"                 2>/dev/null
sleep 1.2
