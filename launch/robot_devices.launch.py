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
)
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource

from tirrex_demo import (
    get_available_devices,
    get_devices_meta_description,
    get_device_meta_description_file_path,
)

from ament_index_python.packages import get_package_share_directory


def get_mode(context):
    return LaunchConfiguration("mode").perform(context)


def get_robot_namespace(context):
    return LaunchConfiguration("robot_namespace").perform(context)


def get_robot_configuration_directory(context):
    return LaunchConfiguration("robot_configuration_directory").perform(context)


def launch_setup(context, *args, **kwargs):

    mode = get_mode(context)
    robot_namespace = get_robot_namespace(context)
    configuration_directory = get_robot_configuration_directory(context)
    devices = get_devices_meta_description(configuration_directory)

    actions = []

    for device_name in get_available_devices(devices, mode):
        device_type = devices[device_name]["type"]

        if device_type != "joystick":

            meta_description_file_path = get_device_meta_description_file_path(
                configuration_directory, devices, device_name
            )

            actions.append(
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        get_package_share_directory("romea_" + device_type + "_bringup")
                        + "/launch/"
                        + device_type
                        + ".launch.py"
                    ),
                    launch_arguments={
                        "mode": mode,
                        "robot_namespace": robot_namespace,
                        "meta_description_file_path": meta_description_file_path,
                    }.items(),
                )
            )

    return actions


def generate_launch_description():

    declared_arguments = []

    declared_arguments.append(DeclareLaunchArgument("mode", default_value="live"))

    declared_arguments.append(DeclareLaunchArgument("robot_namespace"))

    declared_arguments.append(DeclareLaunchArgument("robot_configuration_directory"))

    return LaunchDescription(
        declared_arguments + [OpaqueFunction(function=launch_setup)]
    )
