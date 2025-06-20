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
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.actions import (
    DeclareLaunchArgument,
    OpaqueFunction,
    ExecuteProcess,
)

from tirrex_demo import (
    get_demo_timestamp,
    get_record_configuration,
    get_record_directory,
    get_bag_topics,
)

from shutil import copytree
import os
import subprocess


def launch_setup(context, *args, **kwargs):
    demo = LaunchConfiguration("demo").perform(context)
    demo_config_directory = LaunchConfiguration("demo_config_directory").perform(context)
    robot_config_directory = LaunchConfiguration("robot_config_directory").perform(context)
    demo_timestamp = LaunchConfiguration("demo_timestamp").perform(context)
    robot_namespace = LaunchConfiguration("robot_namespace").perform(context)
    mode = LaunchConfiguration("mode").perform(context)

    record_configuration = get_record_configuration(demo_config_directory)
    record_directory = get_record_directory(record_configuration, demo, demo_timestamp)

    print(f"record_directory: {record_directory}")

    bag_record_cmd = [
        "ros2",
        "bag",
        "record",
        "-s",
        "mcap",
        "--storage-preset-profile",
        "zstd_fast",
    ]

    topics = ["/tf", "/tf_static"]
    topics.extend(get_bag_topics(robot_namespace, robot_config_directory, mode))

    additional_topics = record_configuration["additional_topics"]
    if additional_topics:
        topics.extend(additional_topics)

    # allow using regular expression in topic names
    topics_expr = "|".join(topics)
    bag_record_cmd.extend(["-e", f"^({topics_expr})$"])

    if mode.startswith("simulation"):
        if os.getenv("ROS_DISTRO") == "galactic":
            bag_record_cmd.append("/clock")
        else:
            bag_record_cmd.append("--use-sim-time")

    record_configuration = get_record_configuration(demo_config_directory)

    record_directory = get_record_directory(record_configuration, demo, demo_timestamp)
    bag_record_cmd.extend(["-o", record_directory + "/bag"])
    print(bag_record_cmd)

    if record_configuration["config"] is True:
        copytree(demo_config_directory, record_directory + "/config")

    if record_configuration["vcs"] is True:
        repos_file = open(record_directory + "/demo.repos", "w")
        subprocess.call(["vcs", "-n", "export", "--exact", "src"], stdout=repos_file)

        diff_file = open(record_directory + "/demo.diff", "w")
        subprocess.call(["vcs", "-ns", "diff"], stdout=diff_file)

    recorder = ExecuteProcess(cmd=bag_record_cmd)

    # return [
    #     TimerAction(actions=[recorder], period=10.0),
    # ]

    return [recorder]


def generate_launch_description():
    entities = [
        DeclareLaunchArgument(
            "demo_config_directory",
            description="directory containing the YAML file to configure the simulation",
        ),
        DeclareLaunchArgument(
            "robot_config_directory",
            default_value=PathJoinSubstitution(
                [LaunchConfiguration("demo_config_directory"), "robot"]
            ),
            description="directory containing the YAML file to configure the spawned robot",
        ),
        DeclareLaunchArgument(
            "robot_namespace",
            description="ROS namespace used for the robot",
            default_value="robot",
        ),
        DeclareLaunchArgument(
            "mode",
            default_value="simulation_gazebo_classic",
            description="used to select the context and nodes to start",
            choices=["simulation_gazebo_classic", "simulation", "live", "replay"],
        ),
        DeclareLaunchArgument(
            "demo",
            description="name used for the directory containing logs or record files",
        ),
        DeclareLaunchArgument(
            "demo_timestamp",
            description="timestamp string used as name for the created subdirectory",
            default_value=get_demo_timestamp(),
        ),
        OpaqueFunction(function=launch_setup),
    ]
    return LaunchDescription(entities)
