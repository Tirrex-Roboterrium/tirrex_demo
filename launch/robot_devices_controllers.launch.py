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


def get_robot_urdf_description(context):
    return LaunchConfiguration("robot_urdf_description").perform(context)


def launch_setup(context, *args, **kwargs):

    devices_controllers = []

    configuration_directory = get_robot_configuration_directory(context)
    devices = get_devices_meta_description(configuration_directory)

    for device_name in get_available_devices(devices, get_mode(context), "arm"):
        print("coucou **************************************************************************")

        print(device_name)
        meta_description_file_path = get_device_meta_description_file_path(
            configuration_directory, devices, device_name
        )

        device_type = devices[device_name]["type"]

        devices_controllers.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    get_package_share_directory("romea_" + device_type + "_bringup")
                    + "/launch/"
                    + device_type
                    + "_controllers.launch.py"
                ),
                launch_arguments={
                    "mode": get_mode(context),
                    "robot_namespace": get_robot_namespace(context),
                    "robot_urdf_description": get_robot_urdf_description(context),
                    "meta_description_file_path": meta_description_file_path,
                }.items(),
            )
        )

    return devices_controllers


def generate_launch_description():

    declared_arguments = []

    declared_arguments.append(DeclareLaunchArgument("mode"))

    declared_arguments.append(DeclareLaunchArgument("robot_namespace"))

    declared_arguments.append(DeclareLaunchArgument("robot_configuration_directory"))

    declared_arguments.append(DeclareLaunchArgument("robot_urdf_description"))

    return LaunchDescription(
        declared_arguments + [OpaqueFunction(function=launch_setup)]
    )
