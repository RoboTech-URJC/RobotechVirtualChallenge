
# Copyright (c) 2024 Intelligent Robotics Lab (URJC)
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.


import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    SetEnvironmentVariable
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    world_arg = DeclareLaunchArgument(
        'world', default_value=os.path.join(
            get_package_share_directory('ros2_bringup'),
            'worlds',
            'robotech_virtual_challenge.world'))

    gui_arg = DeclareLaunchArgument(
        'gui',
        default_value='true',
        description='Set to false to run gazebo headless',
    )

    gazebo_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ros_gz_sim'), 'launch',
                         'gz_sim.launch.py')),
        launch_arguments={'gz_args': ['-r -s --headless-rendering ', LaunchConfiguration('world')]}.items()
    )

    gazebo_client = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ros_gz_sim'),
                         'launch',
                         'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': [' -g ']}.items(),
        condition=IfCondition(LaunchConfiguration('gui')),
    )

    clock_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='clock_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        output='screen',
    )

    spawn_team1 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('kobuki_description'),
            'launch/'), 'spawn.launch.py']),
        launch_arguments={
            'namespace': 'team1',
            'name': 'team1_kobuki',
            'x': '-1.0',
            'y': '0.0',
            'z': '0.05',
            'do_tf_remapping': 'false',
            'top_plate_color': '0.10 0.35 1.0 1',
        }.items(),
    )

    spawn_team2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('kobuki_description'),
            'launch/'), 'spawn.launch.py']),
        launch_arguments={
            'namespace': 'team2',
            'name': 'team2_kobuki',
            'x': '1.0',
            'y': '0.0',
            'z': '0.05',
            'do_tf_remapping': 'false',
            'top_plate_color': '1.0 0.25 0.75 1',
        }.items(),
    )

    fake_bumper_team1 = Node(
        package='kobuki',
        executable='fake_bumer_node',
        namespace='team1',
        name='fake_bumer_node',
        output='screen',
    )

    fake_bumper_team2 = Node(
        package='kobuki',
        executable='fake_bumer_node',
        namespace='team2',
        name='fake_bumer_node',
        output='screen',
    )

    bringup_pkg_path = get_package_share_directory('ros2_bringup')
    kobuki_pkg_path = get_package_share_directory('kobuki')
    workspace_path = os.path.abspath(
        os.path.join(bringup_pkg_path, '..', '..', '..', '..'))
    source_model_path = os.path.join(
        workspace_path, 'src', 'ros2_bringup', 'worlds', 'models')

    model_paths = [
        os.path.join(bringup_pkg_path, 'worlds', 'models'),
        source_model_path,
        os.path.join(kobuki_pkg_path, 'models'),
    ]
    model_path = os.pathsep.join(
        path for path in model_paths if os.path.isdir(path))

    if 'GZ_SIM_RESOURCE_PATH' in os.environ:
        model_path += os.pathsep + os.environ['GZ_SIM_RESOURCE_PATH']

    ld = LaunchDescription()
    ld.add_action(SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH', model_path))
    ld.add_action(world_arg)
    ld.add_action(gui_arg)
    ld.add_action(gazebo_server)
    ld.add_action(gazebo_client)
    ld.add_action(clock_bridge)
    ld.add_action(spawn_team1)
    ld.add_action(spawn_team2)
    ld.add_action(fake_bumper_team1)
    ld.add_action(fake_bumper_team2)

    return ld
