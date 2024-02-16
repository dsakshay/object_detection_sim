#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
import launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node



def generate_launch_description():
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        "use_sim_time",
        default_value="true",
        choices=["true", "false"],
        description="Use sim time",
    )

    use_sim_time = LaunchConfiguration("use_sim_time", default='true')


    detect_node = Node(
        package="object_detection_sim",
        executable="detection_node.py",
        name="detect_node",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
    )

    pose_pub_node = Node(
        package="object_detection_sim",
        executable="pose_pub.py",
        name="pose_pub_node",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
    )

    move_node = Node(
        package="object_detection_sim",
        executable="move_to_goal_node.py",
        name="move_node",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
    )

    ld = LaunchDescription()

    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(detect_node)
    ld.add_action(move_node)
    # ld.add_action(pose_pub_node)

    return ld