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

from tirrex_demo import get_replay_configuration
from os import getenv


def get_replay_directory(context):
    return LaunchConfiguration("replay_directory").perform(context)


def get_use_recorded_config(context):
    return LaunchConfiguration("use_recorded_config").perform(context)


def launch_setup(context, *args, **kwargs):

    replay_directory = get_replay_directory(context)
    replay_configuration = get_replay_configuration(replay_directory)
    use_recorded_config = get_use_recorded_config(context)

    actions = []

    launch_arguments = replay_configuration["launch_arguments"]
    if use_recorded_config == "true":
        launch_arguments["demo_config_directory"] = replay_directory+"/config"

    actions.append(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                get_package_share_directory(replay_configuration["pkg"])
                + "/launch/"
                + replay_configuration["launch_file"]
            ),
            launch_arguments=replay_configuration["launch_arguments"].items(),
        )
    )

    bag_play_cmd = ["ros2", "bag", "play", replay_directory + "/bag"]
    if launch_arguments["mode"] == "simulation" and getenv("ROS_DISTRO") != "galactic":
        bag_play_cmd.append("--clock")

    actions.append(ExecuteProcess(cmd=bag_play_cmd))

    return [GroupAction(actions)]


def generate_launch_description():

    declared_arguments = []

    declared_arguments.append(DeclareLaunchArgument("replay_directory"))

    declared_arguments.append(DeclareLaunchArgument("use_recorded_config", default_value="false"))

    return LaunchDescription(
        declared_arguments + [OpaqueFunction(function=launch_setup)]
    )
