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

import datetime
import yaml
import os

from romea_common_bringup import robot_prefix
from romea_common_bringup import device_prefix
from ament_index_python.packages import get_package_share_directory
from os.path import join


def get_demo_config_directory(demo_name):
    return get_package_share_directory(demo_name) + "/config"


def get_base_meta_description_file_path(robot_configuration_directory):
    return robot_configuration_directory + "/base.yaml"


def get_base_meta_description(robot_configuration_directory):
    with open(get_base_meta_description_file_path(robot_configuration_directory)) as f:
        return yaml.safe_load(f)


def get_devices_meta_description_file_path(robot_configuration_directory):
    return robot_configuration_directory + "/devices.yaml"


def get_devices_meta_description(robot_configuration_directory):
    with open(get_devices_meta_description_file_path(robot_configuration_directory)) as f:
        return yaml.safe_load(f)


def get_teleop_configuration_file_path(robot_configuration_directory):
    return robot_configuration_directory + "/teleop.yaml"


def get_teleop_configuration(robot_configuration_directory):
    with open(get_teleop_configuration_file_path(robot_configuration_directory)) as f:
        return yaml.safe_load(f)


def get_simulation_configuration_file_path(demo_configuration_directory):
    return demo_configuration_directory + "/simulation.yaml"


def get_simulation_configuration(robot_configuration_directory):
    with open(get_simulation_configuration_file_path(robot_configuration_directory)) as f:
        return yaml.safe_load(f)


def get_record_configuration_file_path(robot_configuration_directory):
    return robot_configuration_directory + "/records.yaml"


def get_record_configuration(robot_configuration_directory):
    with open(get_record_configuration_file_path(robot_configuration_directory)) as f:
        return yaml.safe_load(f)


def get_demo_timestamp():
    return datetime.datetime.now().strftime("%y-%m-%d-%H-%M")


def get_ros_home_directory(demo_name, demo_timestamp):
    return join("~/.ros/demos/", join(demo_name, demo_timestamp))


def get_ros_home_log_directory(demo_name, demo_timestamp):
    return join(get_ros_home_directory(demo_name, demo_timestamp), "log")


def get_ros_home_debug_directory(demo_name, demo_timestamp):
    return join(get_ros_home_directory(demo_name, demo_timestamp), "debug")


def get_record_directory(records_configuration, demo_name, demo_timestamp):
    return join(records_configuration["directory"], join(demo_name, demo_timestamp))


def get_record_log_directory(records_configuration, demo_name, demo_timestamp):
    return join(
        get_record_directory(records_configuration, demo_name, demo_timestamp), "log"
    )


def get_record_debug_directory(records_configuration, demo_name, demo_timestamp):
    return join(
        get_record_directory(records_configuration, demo_name, demo_timestamp), "debug"
    )


def get_log_directory(demo_name, demo_timestamp, record):
    if record == "true":
        demo_configuration_directory = get_demo_config_directory(demo_name)
        record_configuration = get_record_configuration(demo_configuration_directory)
        if record_configuration["log"]:
            return get_record_log_directory(
                record_configuration, demo_name, demo_timestamp
            )

    return get_ros_home_log_directory(demo_name, demo_timestamp)


def get_debug_directory(demo_name, demo_timestamp, record):
    if record == "true":
        demo_configuration_directory = get_demo_config_directory(demo_name)
        record_configuration = get_record_configuration(demo_configuration_directory)
        if record_configuration["debug"]:
            return get_record_debug_directory(
                record_configuration, demo_name, demo_timestamp
            )

    return get_ros_home_debug_directory(demo_name, demo_timestamp)


def save_replay_configuration(demo_name, demo_timestamp, launch_file, launch_arguments):

    replay_configuration = {}
    replay_configuration["pkg"] = demo_name
    replay_configuration["launch_file"] = launch_file
    replay_configuration["launch_arguments"] = launch_arguments

    demo_configuration_directory = get_demo_config_directory(demo_name)
    record_configuration = get_record_configuration(demo_configuration_directory)
    records_directory = get_record_directory(
        record_configuration, demo_name, demo_timestamp
    )

    filename = records_directory + "/replay.yaml"
    os.makedirs(os.path.dirname(filename), exist_ok=True)
    with open(filename, "w") as f:
        yaml.dump(replay_configuration, f, default_flow_style=False)


def get_replay_configuration_filename(replay_directory):
    return replay_directory + "/replay.yaml"


def get_replay_configuration(replay_directory):
    with open(get_replay_configuration_filename(replay_directory)) as f:
        return yaml.safe_load(f)


def get_device_meta_description_file_path(robot_configuration_directory, devices, device_name):

    device_type = devices[device_name]["type"]

    return (
        robot_configuration_directory
        + "/devices/"
        + device_name
        + "."
        + device_type
        + ".yaml"
    )


def get_device_meta_description(robot_configuration_directory, devices, device_name):

    device_config_yaml_file = get_device_meta_description_file_path(
        robot_configuration_directory, devices, device_name
    )

    with open(device_config_yaml_file) as f:
        return yaml.safe_load(f)


def is_available_device(devices, mode, device_name, type):
    if type == "all" or devices[device_name]["type"] == type:
        return (
            mode in devices[device_name]["available_mode"]
            or devices[device_name]["available_mode"] == "all"
        )
    else:
        return False


def get_available_devices(devices, mode, type="all"):

    available_devices = []
    for device_name in devices.keys():
        if is_available_device(devices, mode, device_name, type):
            available_devices.append(device_name)

    return available_devices


def get_topics(prefix, meta_description):

    configuration = meta_description.get("records", {})
    bridge = meta_description.get("bridge", {})

    topics = []
    for topic in configuration.keys():
        if configuration[topic] is True:
            topics.append(bridge.get(topic, prefix+topic))

    return topics


def get_bag_topics(robot_configuration_directory, mode):

    base = get_base_meta_description(robot_configuration_directory)
    devices = get_devices_meta_description(robot_configuration_directory)
    topics = get_topics(robot_prefix(base["name"]), base)

    for device_name in get_available_devices(devices, mode):

        device = get_device_meta_description(
            robot_configuration_directory, devices, device_name
        )

        topics_prefix = device_prefix(
            base.get("name"),
            device.get("namespace"),
            device.get("name")
        )

        topics.extend(get_topics(topics_prefix, device))

    return topics


def get_available_joystick(robot_configuration_directory, devices, mode):

    available_joysticks = get_available_devices(devices, mode, "joystick")

    if len(available_joysticks) == 0:
        raise LookupError("A joystick must be added in devices configuration")

    if len(available_joysticks) >= 2:
        raise LookupError("Only one joystick must be available in devices configuration")

    return available_joysticks[0]


def get_joystick_meta_description_file_path(robot_configuration_directory, mode):
    devices = get_devices_meta_description(robot_configuration_directory)
    joystick_name = get_available_joystick(robot_configuration_directory, devices, mode)
    return get_device_meta_description_file_path(
        robot_configuration_directory, devices, joystick_name
    )


# def robot_full_name(type, model=""):
#     if model != "":
#         return type + "_" + model
#     else:
#         return type
