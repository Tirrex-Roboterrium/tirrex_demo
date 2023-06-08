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
    get_simulation_configuration_file_path,
    get_record_configuration,
    get_record_directory,
    get_bag_topics,
)

from shutil import copytree
from os import getcwd
import subprocess


def launch_setup(context, *args, **kwargs):

    demo = LaunchConfiguration("demo").perform(context)
    demo_timestamp = LaunchConfiguration("demo_timestamp").perform(context)
    demo_config_directory = LaunchConfiguration("demo_config_directory").perform(context)

    mode = LaunchConfiguration("mode").perform(context)
    record = LaunchConfiguration("record").perform(context)
    robot_namespace = LaunchConfiguration("robot_namespace").perform(context)

    actions = []

    if mode == "simulation":

        simulation_configuration_file_path = get_simulation_configuration_file_path(
            demo_config_directory
        )

        actions.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    get_package_share_directory("romea_simulation_bringup")
                    + "/launch/simulator.launch.py"
                ),
                launch_arguments={
                    "simulator_type": "gazebo",
                    "simulation_configuration_file_path": simulation_configuration_file_path,
                }.items(),
            )
        )

    actions.append(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                get_package_share_directory("tirrex_demo") + "/launch/robot.launch.py"
            ),
            launch_arguments={
                "mode": mode,
                "robot_namespace": robot_namespace,
                "robot_configuration_directory": demo_config_directory+"/robot",
            }.items(),
        )
    )

    if record == "true":

        bag_record_cmd = ["ros2", "bag", "record", "/tf", "/tf_static"]
        bag_record_cmd.extend(get_bag_topics(
            robot_namespace, demo_config_directory+"/robot", mode))

        record_configuration = get_record_configuration(demo_config_directory)

        record_directory = get_record_directory(
            record_configuration, demo, demo_timestamp
        )

        if record_configuration["config"] is True:
            copytree(demo_config_directory, record_directory + "/config")

        if record_configuration["vcs"] is True:
            repos_file = open(record_directory + "/demo.repos", "w")
            subprocess.call(["vcs", "export", "--exact", getcwd()], stdout=repos_file)

            diff_file = open(record_directory + "/demo.diff", "w")
            subprocess.call(["vcs", "diff", getcwd()], stdout=diff_file)

        bag_record_cmd.extend(["-o", record_directory + "/bag"])
        actions.append(ExecuteProcess(cmd=bag_record_cmd))

    return [GroupAction(actions)]


def generate_launch_description():

    declared_arguments = []

    declared_arguments.append(DeclareLaunchArgument("demo"))

    declared_arguments.append(DeclareLaunchArgument("demo_timestamp"))

    declared_arguments.append(DeclareLaunchArgument("demo_config_directory"))

    declared_arguments.append(DeclareLaunchArgument("mode"))

    declared_arguments.append(DeclareLaunchArgument("record", default_value="false"))

    return LaunchDescription(
        declared_arguments + [OpaqueFunction(function=launch_setup)]
    )
