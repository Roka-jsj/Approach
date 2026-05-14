#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    service_name = LaunchConfiguration('service_name')
    navigate_action_name = LaunchConfiguration('navigate_action_name')
    target_frame = LaunchConfiguration('target_frame')
    base_frame = LaunchConfiguration('base_frame')
    detections_topic = LaunchConfiguration('detections_topic')
    object_position_service_name = LaunchConfiguration(
        'object_position_service_name')
    target_class_name = LaunchConfiguration('target_class_name')
    auto_start_on_launch = LaunchConfiguration('auto_start_on_launch')

    return LaunchDescription([
        DeclareLaunchArgument('service_name', default_value='approach_target'),
        DeclareLaunchArgument(
            'navigate_action_name',
            default_value='navigate_to_pose'),
        DeclareLaunchArgument('target_frame', default_value='map'),
        DeclareLaunchArgument('base_frame', default_value='base_link'),
        DeclareLaunchArgument(
            'detections_topic',
            default_value='/yolo/detections_3d'),
        DeclareLaunchArgument(
            'object_position_service_name',
            default_value='/yolo/get_target_position'),
        DeclareLaunchArgument('target_class_name', default_value=''),
        DeclareLaunchArgument('auto_start_on_launch', default_value='false'),

        Node(
            package='stella_approach',
            executable='approach_goal_service.py',
            name='approach_goal_service',
            output='screen',
            parameters=[{
                'service_name': service_name,
                'navigate_action_name': navigate_action_name,
                'target_frame': target_frame,
                'base_frame': base_frame,
                'detections_topic': detections_topic,
                'object_position_service_name': object_position_service_name,
                'target_class_name': target_class_name,
                'auto_start_on_launch': auto_start_on_launch,
            }],
        ),
    ])
