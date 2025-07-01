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
    OpaqueFunction,
    GroupAction,
)

from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

from tirrex_core import launch


def launch_setup(context, *args, **kwargs):

    mode = launch.get_mode(context)
    robot_namespace = launch.get_robot_namespace(context)
    demo_config_directory = launch.get_demo_configuration_directory(context)
    robot_config_directory = launch.get_robot_configuration_directory(context)

    actions = []

    if mode == "simulation_gazebo_classic":

        actions.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    get_package_share_directory("tirrex_core")
                    + "/launch/simulator.launch.py"
                ),
                launch_arguments={
                    "mode": mode,
                    "demo_configuration_directory": demo_config_directory,
                }.items(),
            )
        )

    actions.append(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                get_package_share_directory("tirrex_core")
                + "/launch/robot/robot.launch.py"
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
    return LaunchDescription(
        [
            launch.declare_mode(),
            launch.declare_robot_namespace(),
            launch.declare_demo_configuration_directory(),
            launch.declare_robot_configuration_directory(),
            OpaqueFunction(function=launch_setup),
        ]
    )
