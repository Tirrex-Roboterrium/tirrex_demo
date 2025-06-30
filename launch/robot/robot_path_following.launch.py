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
import yaml

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, GroupAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node, PushRosNamespace, SetParameter


import romea_joystick_utils
import romea_joystick_meta_bringup
import romea_mobile_base_meta_bringup
import tirrex_demo

from tirrex_demo import (
    get_base_meta_description_file_path,
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


def get_wgs84_anchor(context):
    return tirrex_demo.get_wgs84_anchor(get_demo_config_directory(context))


def get_path_following_configuration(context):
    return tirrex_demo.get_path_following_configuration(get_robot_config_directory(context))


def get_mobile_base_meta_description(context):
    return romea_mobile_base_meta_bringup.load_meta_description(
        get_base_meta_description_file_path(get_robot_config_directory(context)),
        get_robot_namespace(context),
    )


def get_joystick_meta_description(context):
    return romea_joystick_meta_bringup.load_meta_description(
        get_joystick_meta_description_file_path(
            get_robot_config_directory(context), get_mode(context)
        ),
        get_robot_namespace(context),
    )


def get_trajectory_file_path(context):
    trajectory_filename = LaunchConfiguration("trajectory_filename").perform(context)
    return os.path.join(get_demo_config_directory(context), "paths", trajectory_filename)


def get_joystick_configuration(joystick_meta_description):
    return romea_joystick_meta_bringup.get_complete_configuration(joystick_meta_description)


def get_joystick_mapping(joystick_meta_description):
    joystick_configuration = get_joystick_configuration(joystick_meta_description)
    joystick_remapping_file_path = (
        get_package_share_directory("romea_path_following")
        + f'/config/joystick/{joystick_configuration["type"]}.yaml'
    )

    with open(joystick_remapping_file_path) as f:
        joystick_remapping = yaml.safe_load(f)

    return romea_joystick_utils.apply_joystick_buttons_remapping(
        joystick_configuration, joystick_remapping
    )


def get_mobile_base_configuration(mobile_base_meta_description):
    return romea_mobile_base_meta_bringup.get_configuration(mobile_base_meta_description)


def launch_setup(context, *args, **kwargs):
    mode = get_mode(context)
    robot_namespace = get_robot_namespace(context)
    joystick_meta_description = get_joystick_meta_description(context)
    mobile_base_meta_description = get_mobile_base_meta_description(context)

    actions = []

    actions.append(PushRosNamespace(robot_namespace))

    actions.append(SetParameter(name="use_sim_time", value=(mode != "live")))

    actions.append(
        Node(
            package="romea_path_matching",
            executable="path_matching_node",
            name="path_matching",
            output="screen",
            parameters=[
                {
                    "wgs84_anchor": get_wgs84_anchor(context),
                    "path": get_trajectory_file_path(context),
                    "prediction_time_horizon": 1.0,
                    "path_frame_id": "map",
                    "autoconfigure": True,
                    "autostart": True,
                }
            ],
            remappings=[("odom", "localisation/filtered_odom")],
        )
    )

    joystick_name = joystick_meta_description.get_name()
    mobile_base_name = mobile_base_meta_description.get_name()

    actions.append(
        Node(
            package="romea_path_following",
            executable="path_following_node",
            name="path_following",
            output="screen",
            parameters=[
                {
                    "joystick": get_joystick_mapping(joystick_meta_description),
                    "base": get_mobile_base_configuration(mobile_base_meta_description),
                    "autoconfigure": True,
                    "autostart": True,
                },
                get_path_following_configuration(context),  # may redefine previous parameters
            ],
            remappings=[
                ("cmd_mux/subscribe", mobile_base_name + "/cmd_mux/subscribe"),
                ("cmd_mux/unsubscribe", mobile_base_name + "/cmd_mux/unsubscribe"),
                ("odometry", mobile_base_name + "/controller/odometry"),
                ("joystick/joy", joystick_name + "/joy"),
            ],
        )
    )

    return [GroupAction(actions)]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("mode"),
            DeclareLaunchArgument("demo_config_directory"),
            DeclareLaunchArgument("robot_namespace", default_value="robot"),
            DeclareLaunchArgument(
                "robot_config_directory",
                default_value=PathJoinSubstitution(
                    [LaunchConfiguration("demo_config_directory"), "robot"]
                ),
            ),
            DeclareLaunchArgument("trajectory_filename"),
            OpaqueFunction(function=launch_setup),
        ]
    )
