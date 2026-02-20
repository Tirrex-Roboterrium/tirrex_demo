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

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource

from tirrex_core import config, launch


def launch_setup(context, *args, **kwargs):
    
    mode = launch.get_mode(context)
    robot_namespace = launch.get_robot_namespace(context)
    configuration_directory = launch.get_robot_configuration_directory(context)
    wgs84_anchor_file_path =  launch.get_wgs84_anchor_file_path(context)

    robot_meta_description_file_path = config.generate_robot_meta_description_file(
        robot_namespace, configuration_directory, mode
    )

    robot=[]
    if mode.startswith("simulation"):
        robot.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    get_package_share_directory("romea_simulation_meta_bringup")
                    + "/launch/entity.launch.py"
                ),
                launch_arguments={
                    "entity_type": "robot",
                    "simulator_type": launch.get_simulator_type(context),
                    "robot_namespace": robot_namespace,
                    "meta_description_file_path": robot_meta_description_file_path,
                }.items(),
            )
        )

    robot.append(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                get_package_share_directory("romea_robot_meta_bringup")
                + "/launch/robot.launch.py"
            ),
            launch_arguments={
                "mode": mode,
                "robot_namespace": robot_namespace,
                "meta_description_file_path": robot_meta_description_file_path,
                "wgs84_anchor_file_path": wgs84_anchor_file_path,
            }.items(),
        )
    )

    robot.append(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                get_package_share_directory("romea_robot_meta_bringup")
                + "/launch/robot_state_publisher.launch.py"
            ),
            launch_arguments={
                "mode": mode,
                "robot_namespace": robot_namespace,
                "meta_description_file_path": robot_meta_description_file_path,
            }.items(),
        )
    )

    return robot


def generate_launch_description():

    declared_arguments = []
    declared_arguments.append(launch.declare_mode())
    declared_arguments.append(launch.declare_robot_namespace())
    declared_arguments.append(launch.declare_robot_configuration_directory())

    return LaunchDescription(
        declared_arguments + [OpaqueFunction(function=launch_setup)]
    )
