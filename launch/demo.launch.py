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
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


def launch_setup(context, *args, **kwargs):

    demo = LaunchConfiguration("demo").perform(context)
    demo_timestamp = LaunchConfiguration("demo_timestamp").perform(context)
    demo_config_directory = LaunchConfiguration("demo_config_directory").perform(context)

    mode = LaunchConfiguration("mode").perform(context)
    record = LaunchConfiguration("record").perform(context)
    robot_namespace = LaunchConfiguration("robot_namespace").perform(context)
    localisation = LaunchConfiguration("localisation").perform(context)

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
                "robot_configuration_directory": demo_config_directory+"/robot",
            }.items(),
        )
    )

    if localisation == "true":
        actions.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    get_package_share_directory("tirrex_demo")
                    + "/launch/robot/robot_localisation.launch.py"
                ),
                launch_arguments={
                    "mode": mode,
                    "robot_namespace": robot_namespace,
                    "demo_config_directory": demo_config_directory,
                }.items(),
            )
        )

    if record == "true":

        actions.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    get_package_share_directory("tirrex_demo") + "/launch/record.launch.py"
                ),
                launch_arguments={
                    "demo": demo,
                    "demo_timestamp": demo_timestamp,
                    "demo_config_directory": demo_config_directory,
                    "mode": mode,
                    "robot_namespace": robot_namespace,
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

    declared_arguments = []

    declared_arguments.append(DeclareLaunchArgument("demo"))

    declared_arguments.append(DeclareLaunchArgument("demo_timestamp"))

    declared_arguments.append(DeclareLaunchArgument("demo_config_directory"))

    declared_arguments.append(DeclareLaunchArgument("mode"))

    declared_arguments.append(DeclareLaunchArgument("robot_namespace"))

    declared_arguments.append(DeclareLaunchArgument("localisation", default_value="true"))

    declared_arguments.append(DeclareLaunchArgument("record", default_value="false"))

    return LaunchDescription(
        declared_arguments + [OpaqueFunction(function=launch_setup)]
    )
