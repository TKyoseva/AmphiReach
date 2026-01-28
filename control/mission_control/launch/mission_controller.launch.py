#!/usr/bin/env python3
"""
ROS2 Launch file for Mission Controller
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os

def generate_launch_description():
    # Get package path
    package_path = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    
    # Declare launch arguments
    max_speed_arg = DeclareLaunchArgument(
        'max_speed',
        default_value='25.0',
        description='Maximum speed for mission (m/s)'
    )
    
    auto_start_arg = DeclareLaunchArgument(
        'auto_start',
        default_value='false',
        description='Automatically start mission on launch'
    )
    
    world_file_arg = DeclareLaunchArgument(
        'world_file',
        default_value=os.path.join(package_path, 'ar_robot_water_world_hydrodynamics.sdf'),
        description='Path to Gazebo world file'
    )
    
    # Mission controller node
    mission_controller_node = Node(
        package='ar_robot_control',
        executable='mission_controller',
        name='mission_controller',
        output='screen',
        parameters=[{
            'max_speed': LaunchConfiguration('max_speed'),
            'auto_start': LaunchConfiguration('auto_start'),
        }],
        remappings=[
            ('odom', '/model/ar_robot/odometry'),  # Adjust based on your odometry topic
        ]
    )
    
    # Motor controller node (if using ROS2)
    motor_controller_node = Node(
        package='ar_robot_control',
        executable='motor_controller',
        name='motor_controller',
        output='screen',
    )
    
    return LaunchDescription([
        max_speed_arg,
        auto_start_arg,
        world_file_arg,
        LogInfo(msg=['Launching Mission Controller']),
        mission_controller_node,
        motor_controller_node,
    ])
