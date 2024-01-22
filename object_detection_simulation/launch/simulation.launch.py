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
    pkg_robot_sim = get_package_share_directory("object_detection_simulation")

    default_rviz = os.path.join(pkg_robot_sim, "rviz", "obj_detect.rviz")

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        "use_sim_time",
        default_value="true",
        choices=["true", "false"],
        description="Use sim time",
    )

    use_sim_time = LaunchConfiguration("use_sim_time", default='true')


    robot_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_robot_sim, "launch", "robot.launch.py")
        ),
        launch_arguments={"use_sim_time": use_sim_time}.items(),
    )

    spawn_object = Node(
        package="ros_ign_gazebo",
        executable="create",
        arguments=[
            "-name",
            "object",
            "-x",
            "5.0",
            "-y",
            "15.0",
            "-z",
            "0.6",
            "-Y",
            "1.57",
            "-file",
            os.path.join(pkg_robot_sim, "models", "object", "object.sdf"),
        ],
        output="screen",
    )


    map_2_odom_tf = Node(package="tf2_ros",
        executable="static_transform_publisher",
        name="map_2_odom",
        parameters=[{"use_sim_time": use_sim_time}],
        arguments=["--x",
                    "0.0",
                    "--y",
                    "0.0",
                    "--z",
                    "0.0",
                    "--roll",
                    "0.0",
                    "--yaw",
                    "0.0",
                    "--pitch",
                    "0.0",
                    "--frame-id",
                    "map",
                    "--child-frame-id",
                    "/robot/odom"]
                #    "10"]
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
        arguments=["--display-config", default_rviz],
    )


    ld = LaunchDescription()

    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(robot_launch)
    ld.add_action(spawn_object)
    ld.add_action(rviz_node)
    ld.add_action(map_2_odom_tf)

    return ld

