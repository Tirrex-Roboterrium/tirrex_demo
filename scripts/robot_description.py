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

import importlib
import sys
import xml.etree.ElementTree as ET

import romea_mobile_base_meta_bringup.meta_description as mobile_base

from tirrex_core.config import (
    get_available_devices,
    get_base_meta_description_file_path,
    get_device_meta_description_file_path,
    get_devices_configuration,
)


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

    mobile_base_meta_description = mobile_base.load_meta_description(
        get_base_meta_description_file_path(configuration_directory), robot_namespace
    )

    urdf = ET.fromstring(
        mobile_base.generate_urdf_description(mode, mobile_base_meta_description)
    )

    devices = get_devices_configuration(configuration_directory)
    for device_name in get_available_devices(devices, mode):

        device_type = devices[device_name]["type"]

        if device_type != "joystick":

            device = importlib.import_module(
                "romea_" + device_type + "_meta_bringup.meta_description"
            )

            device_meta_description_file_path = get_device_meta_description_file_path(
                configuration_directory, devices, device_name
            )

            device_meta_description = device.load_meta_description(
                device_meta_description_file_path, robot_namespace
            )

            urdf.extend(
                ET.fromstring(
                    device.generate_urdf_description(mode, device_meta_description)
                )
            )

    print(ET.tostring(urdf, encoding="unicode"))
