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
from launch.actions import OpaqueFunction, GroupAction
from launch_ros.actions import Node, PushRosNamespace

from tirrex_core import launch


def launch_setup(context, *args, **kwargs):
    robot_namespace = launch.get_robot_namespace(context)
    wgs84_anchor = launch.get_wgs84_anchor(context)
    latitude = wgs84_anchor["latitude"]
    longitude = wgs84_anchor["longitude"]
    altitude = wgs84_anchor["altitude"]

    actions = []

    actions.append(PushRosNamespace(robot_namespace))

    actions.append(
        Node(
            package="romea_path_tools",
            executable="record",
            name="path_recorder",
            output="screen",
            parameters=[
                {"anchor": [latitude, longitude, altitude]},
                {"output": launch.get_path(context)},
            ],
            remappings=[("odom", "localisation/filtered_odom")],
        )
    )

    return [GroupAction(actions)]
    

def generate_launch_description():
    return LaunchDescription(
        [
            launch.declare_robot_namespace(),
            launch.declare_demo_configuration_directory(),
            launch.declare_path("output_path.traj"),
            OpaqueFunction(function=launch_setup)
        ]
    )
