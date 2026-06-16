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

from os import getcwd, getenv
from shutil import copytree
import subprocess

from launch import LaunchDescription
from launch.actions import ExecuteProcess, OpaqueFunction, TimerAction

from tirrex_core import config
from tirrex_core import launch


def launch_setup(context, *args, **kwargs):

    mode = launch.get_mode(context)
    demo = launch.get_demo(context)
    demo_timestamp = launch.get_demo_start_timestamp(context)
    demo_config_directory = launch.get_demo_configuration_directory(context)

    bag_record_cmd = [
        "ros2", "bag", "record",
        "-s", "mcap",
        "--storage-preset-profile", "zstd_fast",
        "-b", "2000000000",  # split: 2.0 GB
    ]
    bag_record_cmd.extend(launch.get_bag_topic(context))

    record_configuration = launch.get_record_configuration(context)
    topics = ["/tf", "/tf_static"]
    additional_topics = record_configuration["additional_topics"]
    if additional_topics:
        topics.extend(additional_topics)

    # allow using regular expression in topic names
    topics_expr = "|".join(topics)
    bag_record_cmd.extend(["-e", f"^({topics_expr})$"])

    if "simulation" in mode:
        if getenv("ROS_DISTRO") == "galactic":
            bag_record_cmd.append("/clock")
        else:
            bag_record_cmd.append("--use-sim-time")

    record_directory = config.get_record_directory(
        record_configuration, demo, demo_timestamp
    )
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

    return LaunchDescription(
        [
            launch.declare_demo(),
            launch.declare_demo_start_timestamp(),
            launch.declare_demo_configuration_directory(),
            launch.declare_robot_namespace(),
            launch.declare_robot_configuration_directory(),
            launch.declare_mode(),
            OpaqueFunction(function=launch_setup)
        ]
    )
