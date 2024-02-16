#!/usr/bin/env python3

import os 

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    pkg_ros_ign_gazebo = get_package_share_directory("ros_ign_gazebo")
    pkg_robot_sim = get_package_share_directory("object_detection_simulation")
    
    # Gazebo launch
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_ign_gazebo, "launch", "ign_gazebo.launch.py"),
        ),
    )

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        "use_sim_time",
        default_value="true",
        choices=["true", "false"],
        description="Use sim time",
    )

    use_sim_time = LaunchConfiguration("use_sim_time")
    
    declare_ignition_args = DeclareLaunchArgument(
        "ign_args",
        default_value=[
            os.path.join(pkg_robot_sim, "worlds", "station.sdf")
            + " -v 2 --gui-config "
            + os.path.join(pkg_robot_sim, "ign", "gui.config"),
            "",
        ],
        description="Ignition Gazebo arguments",
    )

    bridge = Node(
        package="ros_ign_bridge",
        executable="parameter_bridge",
        parameters=[{"use_sim_time": use_sim_time}],
        output="screen",
    )


    ld = LaunchDescription()

    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_ignition_args)
    ld.add_action(bridge)
    ld.add_action(gazebo)


    return ld 


