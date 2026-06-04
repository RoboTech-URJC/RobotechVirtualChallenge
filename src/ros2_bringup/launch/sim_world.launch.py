"""Launch a Robotech Virtual Challenge world with ROS 2 Jazzy and Gazebo."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.actions import SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.conditions import UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch.substitutions import TextSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    package_share = FindPackageShare("ros2_bringup")
    ros_gz_sim_share = FindPackageShare("ros_gz_sim")

    world = LaunchConfiguration("world")
    gui = LaunchConfiguration("gui")
    world_path = PathJoinSubstitution([package_share, "worlds", world])
    model_path = PathJoinSubstitution([package_share, "worlds", "models"])
    source_model_path = TextSubstitution(
        text="/home/manza/Desktop/hackaton/RobotechVirtualChallenge/src/ros2_bringup/worlds/models")

    return LaunchDescription([
        DeclareLaunchArgument(
            "world",
            default_value="robotech_virtual_challenge.world",
            description=(
                "World file from ros2_bringup/worlds. "
                "robotech_virtual_challenge.world is the static "
                "Gazebo Harmonic port of the challenge world."
            ),
        ),
        DeclareLaunchArgument(
            "gui",
            default_value="true",
            description="Start Gazebo with the graphical client.",
        ),
        SetEnvironmentVariable(
            "GZ_SIM_RESOURCE_PATH",
            [
                model_path,
                TextSubstitution(text=":"),
                source_model_path,
                TextSubstitution(text=":"),
                EnvironmentVariable("GZ_SIM_RESOURCE_PATH", default_value=""),
            ],
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    ros_gz_sim_share,
                    "launch",
                    "gz_sim.launch.py",
                ])
            ),
            launch_arguments={
                "gz_args": [TextSubstitution(text="-r "), world_path],
            }.items(),
            condition=IfCondition(gui),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    ros_gz_sim_share,
                    "launch",
                    "gz_sim.launch.py",
                ])
            ),
            launch_arguments={
                "gz_args": [TextSubstitution(text="-s -r --headless-rendering "), world_path],
            }.items(),
            condition=UnlessCondition(gui),
        ),
    ])
