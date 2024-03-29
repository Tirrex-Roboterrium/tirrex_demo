#!/usr/bin/env python3

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

import sys
import importlib
import xml.etree.ElementTree as ET

from tirrex_demo import (
    get_base_meta_description_file_path,
    get_available_devices,
    get_devices_configuration,
    get_device_meta_description_file_path,
)

import romea_mobile_base_bringup

if __name__ == "__main__":

    argv = sys.argv

    parameters = {}
    for argument in argv[1:]:
        name, value = argument.split(":")
        parameters[name] = value

    mode = parameters["mode"]

    if "replay" in mode:
        mode.replace("replay_", " ")

    robot_namespace = parameters["robot_namespace"]
    configuration_directory = parameters["robot_configuration_directory"]

    base_meta_description_file_path = get_base_meta_description_file_path(
        configuration_directory
    )

    urdf = ET.fromstring(
        romea_mobile_base_bringup.urdf_description(
            robot_namespace, mode, base_meta_description_file_path
        )
    )

    devices = get_devices_configuration(configuration_directory)
    for device_name in get_available_devices(devices, mode):

        device_type = devices[device_name]["type"]
        device_meta_description_file_path = get_device_meta_description_file_path(
            configuration_directory, devices, device_name
        )

        if device_type != "joystick":

            device_bringup = importlib.import_module(
                "romea_" + device_type + "_bringup"
            )

            urdf.extend(
                ET.fromstring(
                    device_bringup.urdf_description(
                        robot_namespace, mode, device_meta_description_file_path
                    )
                )
            )

    print(ET.tostring(urdf, encoding="unicode"))
