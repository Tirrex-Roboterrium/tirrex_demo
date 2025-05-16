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
    DeclareLaunchArgument,
    OpaqueFunction,
    GroupAction
)
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetParameter, PushRosNamespace


from ament_index_python.packages import get_package_share_directory

from tirrex_demo import (
    get_available_devices,
    get_base_meta_description,
    get_devices_configuration,
    get_device_meta_description_file_path,
)

import yaml


def get_mode(context):
    return LaunchConfiguration("mode").perform(context)


def get_robot_namespace(context):
    return LaunchConfiguration("robot_namespace").perform(context)


def get_robot_configuration_directory(context):
    return LaunchConfiguration("robot_configuration_directory").perform(context)


def get_robot_urdf_description(context):
    return LaunchConfiguration("robot_urdf_description").perform(context)


def launch_setup(context, *args, **kwargs):

    mode = get_mode(context)
    robot_namespace = get_robot_namespace(context)
    configuration_directory = get_robot_configuration_directory(context)
    robot_urdf_description = get_robot_urdf_description(context)

    base = get_base_meta_description(configuration_directory)
    joint_states_source_list = [base["name"]+"/joint_states"]

    devices = get_devices_configuration(configuration_directory)
    for device_name in get_available_devices(devices, mode, "arm") + \
                        get_available_devices(devices, mode, "implement"):

        meta_description_file_path = get_device_meta_description_file_path(
            configuration_directory, devices, device_name
        )

        with open(meta_description_file_path) as f:
            joint_states_source_list.append(yaml.safe_load(f)["name"]+"/joint_states")

    robot_description = {"robot_description": robot_urdf_description}

    joint_state_publisher = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        parameters=[
            {"source_list": joint_states_source_list},
            {"rate": 100},
            robot_description
        ],
        output={
            'stdout': 'log',
            'stderr': 'log',
        },
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[robot_description],
        output={
            'stdout': 'log',
            'stderr': 'log',
        },
    )

    return [
        GroupAction(
            actions=[
                SetParameter(name="use_sim_time", value=(mode != "live")),
                PushRosNamespace(robot_namespace),
                joint_state_publisher,
                robot_state_publisher,
            ]
        )
    ]


def generate_launch_description():

    declared_arguments = []

    declared_arguments.append(DeclareLaunchArgument("mode"))

    declared_arguments.append(DeclareLaunchArgument("robot_namespace"))

    declared_arguments.append(DeclareLaunchArgument("robot_configuration_directory"))

    declared_arguments.append(DeclareLaunchArgument("robot_urdf_description"))

    return LaunchDescription(
        declared_arguments + [OpaqueFunction(function=launch_setup)]
    )
