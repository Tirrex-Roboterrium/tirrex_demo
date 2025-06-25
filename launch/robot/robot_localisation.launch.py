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
    GroupAction
)
from launch_ros.actions import SetParameter, PushRosNamespace
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


def get_mode(context):
    return LaunchConfiguration("mode").perform(context)


def get_robot_namespace(context):
    return LaunchConfiguration("robot_namespace").perform(context)


def get_robot_config_directory(context):
    return LaunchConfiguration("robot_config_directory").perform(context)


def get_localisation_configuration_file_path(context):
    return get_robot_config_directory(context) + "/localisation.yaml"


def launch_setup(context, *args, **kwargs):

    mode = get_mode(context)
    mode = get_mode(context)

    robot_namespace = get_robot_namespace(context)
    localisation_configuration_file_path = get_localisation_configuration_file_path(context)

    localisation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            get_package_share_directory("romea_robot_to_world_localisation_core")
            + "/launch/robot_to_world_localisation.launch.py"
        ),
        launch_arguments={
            "filter_configuration_file_path": localisation_configuration_file_path,
        }.items(),
    )

    actions = [
        GroupAction(
            [
                SetParameter(name="use_sim_time", value=(mode != "live")),
                PushRosNamespace(robot_namespace),
                PushRosNamespace("localisation"),
                localisation
            ]
        )
    ]

    return actions


def generate_launch_description():

    return LaunchDescription(
        [
            DeclareLaunchArgument("mode", default_value="simulation_gazebo_classic"),
            DeclareLaunchArgument("robot_namespace", default_value="robot"),
            DeclareLaunchArgument("robot_config_directory"),
            OpaqueFunction(function=launch_setup)
        ]
    )
