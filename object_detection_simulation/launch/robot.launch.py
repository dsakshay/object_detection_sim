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
            os.path.join(pkg_robot_ignition, "models", "robot", "model.sdf"),
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

    

    ld = LaunchDescription()
    ld.add_action(declare_use_sim_time_cmd)

    ld.add_action(ignition_args)
    ld.add_action(open_rviz_arg)
    ld.add_action(gazebo)

    ld.add_action(spawn_robot)

    ld.add_action(bridge)


    return ld