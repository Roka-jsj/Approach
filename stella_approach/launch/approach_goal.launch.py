#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    detections_topic = LaunchConfiguration("detections_topic")
    cmd_vel_topic = LaunchConfiguration("cmd_vel_topic")
    service_name = LaunchConfiguration("service_name")
    target_frame = LaunchConfiguration("target_frame")
    desired_distance = LaunchConfiguration("desired_distance")
    y_tolerance = LaunchConfiguration("y_tolerance")
    target_timeout_sec = LaunchConfiguration("target_timeout_sec")
    drive_target_timeout_sec = LaunchConfiguration("drive_target_timeout_sec")
    max_rotation_step_deg = LaunchConfiguration("max_rotation_step_deg")
    settle_time_sec = LaunchConfiguration("settle_time_sec")
    rotate_speed = LaunchConfiguration("rotate_speed")
    max_forward_speed = LaunchConfiguration("max_forward_speed")
    drive_angular_kp = LaunchConfiguration("drive_angular_kp")
    max_drive_angular_speed = LaunchConfiguration("max_drive_angular_speed")
    max_drive_heading_error_deg = LaunchConfiguration("max_drive_heading_error_deg")

    return LaunchDescription(
        [
            DeclareLaunchArgument("detections_topic", default_value="/yolo/detections_3d"),
            DeclareLaunchArgument("cmd_vel_topic", default_value="/cmd_vel"),
            DeclareLaunchArgument("service_name", default_value="approach_object"),
            DeclareLaunchArgument("target_frame", default_value="base_link"),
            DeclareLaunchArgument("desired_distance", default_value="0.30"),
            DeclareLaunchArgument("y_tolerance", default_value="0.10"),
            DeclareLaunchArgument("target_timeout_sec", default_value="1.5"),
            DeclareLaunchArgument("drive_target_timeout_sec", default_value="0.35"),
            DeclareLaunchArgument("max_rotation_step_deg", default_value="10.0"),
            DeclareLaunchArgument("settle_time_sec", default_value="0.2"),
            DeclareLaunchArgument("rotate_speed", default_value="0.8"),
            DeclareLaunchArgument("max_forward_speed", default_value="0.2"),
            DeclareLaunchArgument("drive_angular_kp", default_value="1.2"),
            DeclareLaunchArgument("max_drive_angular_speed", default_value="0.35"),
            DeclareLaunchArgument("max_drive_heading_error_deg", default_value="12.0"),
            Node(
                package="stella_approach",
                executable="approach_controller.py",
                name="approach_controller",
                output="screen",
                parameters=[
                    {
                        "detections_topic": detections_topic,
                        "cmd_vel_topic": cmd_vel_topic,
                        "service_name": service_name,
                        "target_frame": target_frame,
                        "desired_distance": desired_distance,
                        "y_tolerance": y_tolerance,
                        "target_timeout_sec": target_timeout_sec,
                        "drive_target_timeout_sec": drive_target_timeout_sec,
                        "max_rotation_step_deg": max_rotation_step_deg,
                        "settle_time_sec": settle_time_sec,
                        "rotate_speed": rotate_speed,
                        "max_forward_speed": max_forward_speed,
                        "drive_angular_kp": drive_angular_kp,
                        "max_drive_angular_speed": max_drive_angular_speed,
                        "max_drive_heading_error_deg": max_drive_heading_error_deg,
                    }
                ],
            ),
        ]
    )
