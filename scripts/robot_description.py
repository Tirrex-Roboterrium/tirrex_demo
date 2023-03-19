#!/usr/bin/env python3
import sys
import importlib
import xml.etree.ElementTree as ET

from tirrex_demo import (
    get_base_meta_description_filename,
    get_available_devices,
    get_devices_meta_description,
    get_device_meta_description_filename,
)

import romea_mobile_base_bringup

if __name__ == "__main__":

    argv = sys.argv

    parameters = {}
    for argument in argv[1:]:
        name, value = argument.split(":")
        parameters[name] = value

    mode = parameters["mode"]
    if "live" in mode:
        mode = "live"
    if "simulation" in mode:
        mode = "simulation"

    robot_namespace = parameters["robot_namespace"]
    configuration_directory = parameters["robot_configuration_directory"]

    base_meta_description_filename = get_base_meta_description_filename(
        configuration_directory
    )

    urdf = ET.fromstring(
        romea_mobile_base_bringup.urdf_description(
            robot_namespace, mode, base_meta_description_filename
        )
    )

    devices = get_devices_meta_description(configuration_directory)
    for device_name in get_available_devices(devices, mode):

        device_type = devices[device_name]["type"]
        device_meta_description_filename = get_device_meta_description_filename(
            configuration_directory, devices, device_name
        )

        if device_type != "joystick":

            device_bringup = importlib.import_module(
                "romea_" + device_type + "_bringup"
            )

            urdf.extend(
                ET.fromstring(
                    device_bringup.urdf_description(
                        robot_namespace, device_meta_description_filename
                    )
                )
            )

    # set gazebo control at the end of the file because there's problem
    # with plugin namespace if gazebo_ros2_control is first launched (i don't understand)
    index = list(urdf).index(urdf.find("ros2_control"))
    urdf.append(urdf[index])
    urdf.remove(urdf[index])
    urdf.append(urdf[index])
    urdf.remove(urdf[index])

    print(ET.tostring(urdf, encoding="unicode"))
