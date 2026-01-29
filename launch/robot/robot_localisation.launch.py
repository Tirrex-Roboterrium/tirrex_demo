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
from launch.actions import GroupAction, OpaqueFunction
from launch_ros.actions import Node, PushRosNamespace, SetParameter

from tirrex_core import launch


def launch_setup(context, *args, **kwargs):

    mode = launch.get_mode(context)
    robot_namespace = launch.get_robot_namespace(context)
    localisation_configuration = launch.get_localisation_configuration(context)

    localisation = Node(
        package="romea_robot_to_world_localisation_core",
        executable="robot_to_world_kalman_localisation_node",
        name="kalman",
        parameters=[localisation_configuration],
        output="screen",
    )

    actions = [
        GroupAction(
            [
                SetParameter(name="use_sim_time", value=(mode != "live")),
                PushRosNamespace(robot_namespace),
                PushRosNamespace("localisation"),
                localisation,
            ]
        )
    ]

    return actions


def generate_launch_description():

    return LaunchDescription(
        [
            launch.declare_mode(),
            launch.declare_robot_namespace("robot"),
            launch.declare_robot_configuration_directory(),
            OpaqueFunction(function=launch_setup),
        ]
    )
