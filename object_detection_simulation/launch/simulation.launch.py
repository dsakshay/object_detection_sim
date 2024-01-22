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
            "20.0",
            "-z",
            "0.6",
            "-Y",
            "1.57",
            "-file",
            os.path.join(pkg_robot_sim, "models", "object", "object.sdf"),
        ],
        output="screen",
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

    return ld

