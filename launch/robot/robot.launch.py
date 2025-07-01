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
from launch.actions import IncludeLaunchDescription, OpaqueFunction
from launch.substitutions import Command, LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import ExecutableInPackage
from ament_index_python.packages import get_package_share_directory

from tirrex_core import launch


def launch_setup(context, *args, **kwargs):

    mode = launch.get_mode(context)
    robot_namespace = launch.get_robot_namespace(context)
    robot_urdf_description = launch.get_robot_urdf_description(context)
    configuration_directory = launch.get_robot_configuration_directory(context)
    base_meta_description_file_path = launch.get_mobile_base_meta_description_file_path(context)
    joystick_meta_description_file_path = launch.get_joystick_meta_description_file_path(context)

    robot = []

    # print(get_robot_urdf_description(context))
    # print("robot_namespace", robot_namespace)

    if mode.startswith("simulation"):
        robot.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    get_package_share_directory("romea_simulation_meta_bringup")
                    + "/launch/entity.launch.py"
                ),
                launch_arguments={
                    "simulator_type": launch.get_simulator_type(context),
                    "robot_namespace": robot_namespace,
                    "robot_urdf_description": robot_urdf_description,
                    "meta_description_file_path": base_meta_description_file_path,
                }.items(),
            )
        )

    if "replay" not in mode:

        robot.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    get_package_share_directory("romea_mobile_base_meta_bringup")
                    + "/launch/mobile_base.launch.py"
                ),
                launch_arguments={
                    "mode": mode,
                    "robot_namespace": robot_namespace,
                    "mobile_base_meta_description_file_path": base_meta_description_file_path,
                    "joystick_meta_description_file_path": joystick_meta_description_file_path,
                    "urdf_description": robot_urdf_description,
                }.items(),
            )
        )

        robot.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    get_package_share_directory("tirrex_core")
                    + "/launch/robot/robot_devices.launch.py"
                ),
                launch_arguments={
                    "mode": mode,
                    "robot_namespace": robot_namespace,
                    "robot_configuration_directory": configuration_directory,
                }.items(),
            )
        )

    robot.append(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                get_package_share_directory("tirrex_core")
                + "/launch/robot/robot_state_publisher.launch.py"
            ),
            launch_arguments={
                "mode": mode,
                "robot_namespace": robot_namespace,
                "robot_configuration_directory": configuration_directory,
                "robot_urdf_description": robot_urdf_description,
            }.items(),
        )
    )

    return robot


def generate_launch_description():

    declared_arguments = []
    declared_arguments.append(launch.declare_mode())
    declared_arguments.append(launch.declare_robot_namespace())
    declared_arguments.append(launch.declare_robot_configuration_directory())

    robot_urdf_description = Command(
        [
            ExecutableInPackage("robot_description.py", "tirrex_core"),
            " mode:",
            LaunchConfiguration("mode"),
            " robot_namespace:",
            LaunchConfiguration("robot_namespace"),
            " robot_configuration_directory:",
            LaunchConfiguration("robot_configuration_directory"),
        ],
        on_stderr="ignore"
    )

    declared_arguments.append(launch.declare_robot_urdf_description(robot_urdf_description))

    return LaunchDescription(
        declared_arguments + [OpaqueFunction(function=launch_setup)]
    )
