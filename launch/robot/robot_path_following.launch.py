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

import yaml

from launch import LaunchDescription
from launch.actions import OpaqueFunction, GroupAction
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node, PushRosNamespace, SetParameter

import romea_joystick_utils
import romea_joystick_meta_bringup
import romea_mobile_base_meta_bringup
from tirrex_core import launch


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
    mode = launch.get_mode(context)
    robot_namespace = launch.get_robot_namespace(context)
    joystick_meta_description = launch.get_joystick_meta_description(context)
    mobile_base_meta_description = launch.get_mobile_base_meta_description(context)

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
                    "wgs84_anchor": launch.get_wgs84_anchor(context),
                    "path": launch.get_path(context),
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
                launch.get_path_following_configuration(context),  # may redefine previous parameters
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
            launch.declare_mode(),
            launch.declare_robot_namespace("robot"),
            launch.declare_demo_configuration_directory(),
            launch.declare_robot_configuration_directory(),
            launch.declare_path(),
            OpaqueFunction(function=launch_setup),
        ]
    )
