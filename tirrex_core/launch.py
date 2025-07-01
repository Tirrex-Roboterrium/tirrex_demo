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

import os
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

import romea_mobile_base_meta_bringup
import romea_joystick_meta_bringup
from tirrex_core import config


def declare_argument(description, default_value):
    if not default_value:
        return DeclareLaunchArgument(**description)
    else:
        return DeclareLaunchArgument(**description, default_value=default_value)


def declare_robot_namespace(default_value=None):
    return declare_argument(
        {
            "name": "robot_namespace",
            "description": "ROS namespace used for the robot without the '/' prefix",
        },
        default_value
    )


def declare_mode(default_value=None):    
    return declare_argument(
        {
            "name": "mode",
            "description": "used to select the context and nodes to start",
            "choices": [
                "live",
                "simulation",
                "simulation_gazebo",
                "simulation_gazebo_classic",
                "replay",
            ],
        },
        default_value
    )


def declare_demo(default_value=None):
    return declare_argument(
        {
            "name": "demo",
            "description": "name of the demonstration",
        },
        default_value
    )


def declare_demo_configuration_directory(default_value=None):
    return declare_argument(
        {
            "name": "demo_configuration_directory",
            "description": "directory containing the YAML files used to describe the demonstration",
        },
        default_value
    )


def declare_demo_start_timestamp(default_value=None):
    return declare_argument(
        {
            "name": "demo_start_timestamp",
            "description": "string representation of the demo start timestamp",
        },
        default_value
    )


def declare_robot_configuration_directory():
    return DeclareLaunchArgument(
        name="robot_configuration_directory",
        default_value=PathJoinSubstitution(
            [LaunchConfiguration("demo_configuration_directory"), "robot"]
        ),
        description="directory containing the YAML files used to configure the robot",
    )


def declare_robot_urdf_description(default_value=None):
    return declare_argument(
        {
            "name": "robot_urdf_description",
            "description": "robot URDF description",
        },
        default_value
    )


def declare_record(default_value="false"):
    return declare_argument(
        {
            "name": "record",
            "description": "enable or not recording",
            "choices": ["true", "false"]
        },
        default_value
    )


def declare_robot(default_value=None):
    return declare_argument(
        {
            "name": "robot",
            "description": "name of the selected robot",
        },
        default_value
    )


def declare_launch_robot(default_value="false"):
    return DeclareLaunchArgument(
        "launch_robot",
        default_value=default_value,
        description="start robot nodes or not.",
        choices=["true", "false"],
    )


def declare_path(default_value=None):
    return declare_argument(
        {
            "name": "path",
            "description": "path name for the robot's navigation or for recording purposes",
        },
        default_value
    )


def get_mode(context):
    mode = LaunchConfiguration("mode").perform(context)
    if mode == "simulation":
        mode += "_gazebo_classic"
    return mode


def get_robot_namespace(context):
    return LaunchConfiguration("robot_namespace").perform(context)


def get_demo(context):
    return LaunchConfiguration("demo").perform(context)


def get_demo_configuration_directory(context):
    return LaunchConfiguration("demo_configuration_directory").perform(context)


def get_demo_start_timestamp(context):
    return LaunchConfiguration("demo_start_timestamp").perform(context)


def get_robot_configuration_directory(context):
    return LaunchConfiguration("robot_configuration_directory").perform(context)


def get_robot_urdf_description(context):
    return LaunchConfiguration("robot_urdf_description").perform(context)


def get_record(context):
    return LaunchConfiguration("record").perform(context)


def get_robot(context):
    return LaunchConfiguration("robot").perform(context)


def get_launch_robot(context):
    return LaunchConfiguration("launch_robot").perform(context)


def get_path(context):
    path = LaunchConfiguration("path").perform(context)
    if not os.path.isabs(path):
        path = config.get_trajectory_file_path(get_demo_configuration_directory(context), path)

    return path


def get_simulation_configuration_file_path(context):
    return config.get_simulation_configuration_file_path(
        get_demo_configuration_directory(context))


def get_simulation_configuration(context):
    return config.get_simulation_configuration(
        get_demo_configuration_directory(context))


def get_simulator_type(context):
    mode = get_mode(context)
    assert mode.startswith("simulation_"), "Cannot deduce simulator type from mode."
    return mode.replace("simulation_", "")


def get_wgs84_anchor_file_path(context):
    return config.get_wgs84_anchor_file_path(get_demo_configuration_directory(context))


def get_wgs84_anchor(context):
    return config.get_wgs84_anchor(get_demo_configuration_directory(context))


def get_mobile_base_meta_description_file_path(context):
    return config.get_base_meta_description_file_path(
        get_robot_configuration_directory(context)
    )


def get_mobile_base_meta_description(context):
    return romea_mobile_base_meta_bringup.load_meta_description(
        get_mobile_base_meta_description_file_path(context),
        get_robot_namespace(context),
    )


def get_joystick_meta_description_file_path(context):
    return config.get_joystick_meta_description_file_path(
            get_robot_configuration_directory(context),
            get_mode(context)
    )


def get_joystick_meta_description(context):
    print(get_joystick_meta_description_file_path(context))
    return romea_joystick_meta_bringup.load_meta_description(
        get_joystick_meta_description_file_path(context),
        get_robot_namespace(context),
    )


def get_localisation_configuration(context):
    return config.get_localisation_configuration(
        get_robot_configuration_directory(context)
    )


def get_path_following_configuration(context):
    return config.get_path_following_configuration(
        get_robot_configuration_directory(context)
    )


def get_record_configuration(context):
    return config.get_record_configuration(
        get_robot_configuration_directory(context)
    )


def get_bag_topic(context):
    return config.get_bag_topics(
        get_robot_namespace(context),
        get_robot_configuration_directory(context),
        get_mode(context)
    )
