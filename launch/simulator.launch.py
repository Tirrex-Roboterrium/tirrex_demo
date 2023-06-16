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
    ExecuteProcess,
)

from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

from tirrex_demo import (
    get_wgs84_anchor_file_path,
    get_simulation_configuration_file_path,
    get_record_configuration,
    get_record_directory,
    get_bag_topics,
)

from shutil import copytree
from os import getcwd
import subprocess


def launch_setup(context, *args, **kwargs):

    simulator_type = LaunchConfiguration("simulator_type").perform(context)
    demo_config_directory = LaunchConfiguration("demo_config_directory").perform(context)

    wgs84_anchor_file_path = get_wgs84_anchor_file_path(
        demo_config_directory
    )

    simulation_configuration_file_path = get_simulation_configuration_file_path(
        demo_config_directory
    )

    simulator = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            get_package_share_directory("romea_simulation_bringup")
            + "/launch/simulator.launch.py"
        ),
        launch_arguments={
            "simulator_type": simulator_type,
            "wgs84_anchor_file_path": wgs84_anchor_file_path,
            "simulation_configuration_file_path": simulation_configuration_file_path,
        }.items(),
    )

    return [simulator]


def generate_launch_description():

    declared_arguments = []

    declared_arguments.append(DeclareLaunchArgument("demo_config_directory"))

    declared_arguments.append(DeclareLaunchArgument("simulator_type", default_value="gazebo"))

    return LaunchDescription(
        declared_arguments + [OpaqueFunction(function=launch_setup)]
    )
