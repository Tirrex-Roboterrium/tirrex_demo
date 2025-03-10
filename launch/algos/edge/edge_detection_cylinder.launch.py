# Copyright 2024 INRAE, French National Research Institute for Agriculture, Food and Environment
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
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

from tirrex_demo import (
    get_joystick_meta_description_file_path,
)


def get_mode(context):
    return LaunchConfiguration("mode").perform(context)


def get_robot_namespace(context):
    return LaunchConfiguration("robot_namespace").perform(context)


def get_demo_config_directory(context):
    return LaunchConfiguration("demo_config_directory").perform(context)


def get_robot_config_directory(context):
    return LaunchConfiguration("robot_config_directory").perform(context)


def get_algo_configuration_file_path(context):
    return get_robot_config_directory(context) + "/edge_detection_cylinder.yaml"


def launch_setup(context, *args, **kwargs):
    mode = get_mode(context)
    robot_namespace = get_robot_namespace(context)
    robot_config_directory = get_robot_config_directory(context)
    configuration_file_path = get_algo_configuration_file_path(context)

    joystick_meta_description_file_path = get_joystick_meta_description_file_path(
        robot_config_directory, mode
    )

    edge_detection = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            get_package_share_directory("romea_edge_detection_cylinder")
            + "/launch/edge_detection_cylinder.launch.py"
        ),
        launch_arguments={
            "mode": mode,
            "robot_namespace": robot_namespace,
            "algo_configuration_file_path": configuration_file_path,
            "joystick_meta_description_file_path": joystick_meta_description_file_path,
        }.items(),
    )

    return [edge_detection]


def generate_launch_description():

    declared_arguments = []

    declared_arguments.append(DeclareLaunchArgument("mode"))

    declared_arguments.append(DeclareLaunchArgument("robot_namespace"))

    declared_arguments.append(DeclareLaunchArgument("demo_config_directory"))

    declared_arguments.append(DeclareLaunchArgument("robot_config_directory"))

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])
