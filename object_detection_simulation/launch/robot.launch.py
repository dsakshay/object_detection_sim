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
    pkg_robot_ignition = get_package_share_directory("object_detection_simulation")

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

    # Spawn Robot
    spawn_robot = Node(
        package="ros_ign_gazebo",
        executable="create",
        arguments=[
            "-name",
            "robot",
            "-x",
            "5.0",
            "-z",
            "0.6",
            "-Y",
            "1.57",
            "-file",
            os.path.join(pkg_robot_ignition, "models", "robot", "robot.sdf"),
        ],
        output="screen",
    )

    # Ignition Bridge
    bridge = Node(
        package="ros_ign_bridge",
        executable="parameter_bridge",
        arguments=[
            "/world/station/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock",
            "/tf@tf2_msgs/msg/TFMessage[ignition.msgs.Pose_V",
            "/cmd_vel@geometry_msgs/msg/Twist@ignition.msgs.Twist",
            "/odom@nav_msgs/msg/Odometry[ignition.msgs.Odometry",
            "/laserscan@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan",
            "/model/robot/pose@geometry_msgs/msg/TransformStamped[ignition.msgs.Pose",
            "/camera/camera_info@sensor_msgs/msg/CameraInfo[ignition.msgs.CameraInfo",
            "/camera/color@sensor_msgs/msg/Image[ignition.msgs.Image",
            "/world/station/model/robot/link/camera_depth_frame/sensor/depth/camera_info@sensor_msgs/msg/CameraInfo[ignition.msgs.CameraInfo",
            "/world/station/model/robot/link/camera_depth_frame/sensor/depth/depth_image/points@sensor_msgs/msg/PointCloud2[ignition.msgs.PointCloudPacked",
        ],
        remappings=[
            (
                "/camera/camera_info",
                "/front_camera/rgb/camera_info",
            ),
            (
                "/camera/color",
                "/front_camera/rgb",
            ),
            (
                "/world/station/model/robot/link/camera_depth_frame/sensor/depth/camera_info",
                "/front_camera/stereo/camera_info",
            ),
            (
                "/world/station/model/robot/link/camera_depth_frame/sensor/depth/depth_image/points",
                "/front_camera/stereo/points",
            ),
            (
                "/world/station/clock",
                "/clock",
            ),
        ],
        parameters=[{"use_sim_time": use_sim_time}],
        output="screen",
    )

    ignition_args = DeclareLaunchArgument(
        "ign_args",
        default_value=[
            os.path.join(pkg_robot_ignition, "worlds", "station.sdf")
            + " -v 2 --gui-config "
            + os.path.join(pkg_robot_ignition, "ign", "gui.config"),
            "",
        ],
        description="Ignition Gazebo arguments",
    )

    open_rviz_arg = DeclareLaunchArgument(
        "rviz", default_value="true", description="Open RViz."
    )

    T_baselink_camera = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=[
            "--x",
            "2.012",
            "--y",
            "0",
            "--z",
            "-0.05",
            "--yaw",
            "0",
            "--pitch",
            "0",
            "--roll",
            "0",
            "--frame-id",
            "robot/base_link",
            "--child-frame-id",
            "robot/camera_front",
        ],
        parameters=[{"use_sim_time": use_sim_time}],
    )

    

    ld = LaunchDescription()
    ld.add_action(declare_use_sim_time_cmd)

    ld.add_action(ignition_args)
    ld.add_action(open_rviz_arg)
    ld.add_action(gazebo)

    ld.add_action(spawn_robot)
    ld.add_action(T_baselink_camera)

    ld.add_action(bridge)


    return ld