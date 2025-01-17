# Copyright 2022 INRAE, French National Research Institute for Agriculture, Food and Environment
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

from launch import LaunchDescription

from launch.actions import (
    IncludeLaunchDescription,
    DeclareLaunchArgument,
    OpaqueFunction,
    GroupAction,
)

from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


def launch_setup(context, *args, **kwargs):

    mode = LaunchConfiguration("mode").perform(context)
    robot_namespace = LaunchConfiguration("robot_namespace").perform(context)
    demo_config_directory = LaunchConfiguration("demo_config_directory").perform(context)
    robot_config_directory = LaunchConfiguration("robot_config_directory").perform(context)

    if mode == "simulation":
        mode += "_gazebo_classic"

    actions = []

    if mode == "simulation_gazebo_classic":

        actions.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    get_package_share_directory("tirrex_demo")
                    + "/launch/simulator.launch.py"
                ),
                launch_arguments={
                    "simulator_type": "gazebo",
                    "demo_config_directory": demo_config_directory,
                }.items(),
            )
        )

    actions.append(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                get_package_share_directory("tirrex_demo") + "/launch/robot/robot.launch.py"
            ),
            launch_arguments={
                "mode": mode,
                "robot_namespace": robot_namespace,
                "robot_configuration_directory": robot_config_directory, 
            }.items(),
        )
    )

    actions.append(
        Node(
            package="rqt_runtime_monitor",
            executable="rqt_runtime_monitor",
            name="monitor",
            arguments=['--force-discover'],
        )
    )

    return [GroupAction(actions)]


def generate_launch_description():
    entities = [
        DeclareLaunchArgument(
            'demo_config_directory',
            description='directory containing the YAML file to configure the simulation',
        ),
        DeclareLaunchArgument(
            'robot_config_directory',
            default_value=PathJoinSubstitution(
                [LaunchConfiguration('demo_config_directory'), 'robot']
            ),
            description='directory containing the YAML file to configure the spawned robot',
        ),
        DeclareLaunchArgument(
            'robot_namespace',
            description='ROS namespace used for the robot',
            default_value="robot",
        ),
        DeclareLaunchArgument(
            'mode',
            default_value='simulation_gazebo_classic',
            description='used to select the context and nodes to start',
            choices=['simulation_gazebo_classic', 'simulation', 'live', 'replay'],
        ),
        OpaqueFunction(function=launch_setup),
    ]
    return LaunchDescription(entities)
