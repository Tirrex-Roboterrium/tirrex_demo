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
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


def get_robot_namespace(context):
    return LaunchConfiguration("robot_namespace").perform(context)


def get_demo_config_directory(context):
    return LaunchConfiguration("demo_config_directory").perform(context)


def get_wgs84_anchor_configuration_file_path(context):
    return get_demo_config_directory(context) + "/wgs84_anchor.yaml"


def get_trajectorty_file_path(context):
    return LaunchConfiguration("trajectory_filename").perform(context)


def launch_setup(context, *args, **kwargs):

    robot_namespace = get_robot_namespace(context)
    trajectorty_file_path = get_trajectorty_file_path(context)
    wgs84_anchor_file_path = get_wgs84_anchor_configuration_file_path(context)

    path_recorder = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            get_package_share_directory("romea_path_following")
            + "/launch/path_recorder.launch.py"
        ),
        launch_arguments={
            "robot_namespace": robot_namespace,
            "trajectory_file_path": trajectorty_file_path,
            "wgs84_anchor_file_path": wgs84_anchor_file_path,
        }.items(),
    )

    return [path_recorder]


def generate_launch_description():

    declared_arguments = []

    declared_arguments.append(DeclareLaunchArgument("robot_namespace"))

    declared_arguments.append(DeclareLaunchArgument("trajectory_filename"))

    declared_arguments.append(DeclareLaunchArgument("demo_config_directory"))

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])
