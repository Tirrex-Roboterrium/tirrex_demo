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
from launch.actions import DeclareLaunchArgument, OpaqueFunction, GroupAction
from launch.substitutions import LaunchConfiguration  # Command
from launch_ros.actions import Node, SetParameter, PushRosNamespace


from tirrex_demo import (
    get_devices_meta_description,
    get_device_meta_description_file_path,
    get_available_devices,
    get_wgs84_anchor,
    get_localisation_configuration,
)

import yaml

# import importlib
from romea_common_bringup import device_namespace, robot_prefix
from romea_imu_bringup import IMUMetaDescription, get_imu_specifications
from romea_gps_bringup import GPSMetaDescription, get_gps_specifications


def get_mode(context):
    return LaunchConfiguration("mode").perform(context)


def get_robot_namespace(context):
    return LaunchConfiguration("robot_namespace").perform(context)


def get_demo_config_directory(context):
    return LaunchConfiguration("demo_config_directory").perform(context)


def get_sensor_meta_description_file_path(
    mode, configuration_directory, sensor_ros_name
):
    devices = get_devices_meta_description(configuration_directory)
    for device_name in get_available_devices(devices, mode):

        # module = importlib.import_module("romea_"+devices[device_name]["type"])
        # my_class = getattr(module, 'MyClass')

        meta_description_file_path = get_device_meta_description_file_path(
            configuration_directory, devices, device_name
        )

        with open(meta_description_file_path) as f:
            meta_description = yaml.safe_load(f)

        if meta_description["name"] == sensor_ros_name:
            return meta_description_file_path

    return None


def get_device_namespace(robot_namespace, plugin_configuration):
    device_name = plugin_configuration["sensor_name"]
    return device_namespace(robot_namespace, None, device_name)


def get_controller_namespace(robot_namespace, odo_plugin_configuration):
    base_name = odo_plugin_configuration["base_name"]
    base_namespace = device_namespace(robot_namespace, None, base_name)
    return base_namespace + "/" + odo_plugin_configuration["controller_name"]


def get_filter_parameters(robot_namespace, localisation_configuration):
    base_footprint_frame_id = robot_prefix(robot_namespace) + "base_link"

    return [
        localisation_configuration["core"],
        {"base_footprint_frame_id": base_footprint_frame_id},
    ]
    # log_directory


def get_odo_plugin_parameters(mode, robot_config_directory, localisation_configuration):
    return [localisation_configuration["plugins"]["odo"]]


def get_odo_plugin_remappings(robot_namespace, localisation_configuration):

    odo_plugin_configuration = localisation_configuration["plugins"]["odo"]
    controller_topic = odo_plugin_configuration["controller_topic"]
    controller_namespace = get_controller_namespace(
        robot_namespace, odo_plugin_configuration
    )

    return [
        (
            "vehicle_controller/" + controller_topic,
            controller_namespace + "/" + controller_topic,
        )
    ]


def get_imu_plugin_parameters(mode, robot_config_directory, localisation_configuration):

    imu_plugin_configuration = localisation_configuration["plugins"]["imu"]

    imu_meta_description_file_path = get_sensor_meta_description_file_path(
        mode, robot_config_directory, imu_plugin_configuration["sensor_name"]
    )

    imu_meta_description = IMUMetaDescription(imu_meta_description_file_path)

    imu_specifications = get_imu_specifications(imu_meta_description)

    # imu_plugin_configuration.pop("sensor_name")

    return [
        imu_plugin_configuration,
        {"enable_accelerations": ("live" in mode)},
        {"imu": imu_specifications},
        {"imu.rate": float(imu_meta_description.get_rate())},
        {"imu.xyz": imu_meta_description.get_xyz()},
        {"imu.rpy": imu_meta_description.get_rpy_rad()},
    ]


def get_imu_plugin_remappings(robot_namepsace, localisation_configuration):

    imu_plugin_configuration = localisation_configuration["plugins"]["imu"]
    imu_namespace = get_device_namespace(robot_namepsace, imu_plugin_configuration)

    odo_plugin_configuration = localisation_configuration["plugins"]["odo"]
    controller_namespace = get_controller_namespace(
        robot_namepsace, odo_plugin_configuration
    )

    return [
        ("imu/data", imu_namespace + "/data"),
        ("vehicle_controller/odom", controller_namespace + "/odom"),
    ]


def get_gps_plugin_parameters(
    mode, robot_config_directory, localisation_configuration, wgs84_anchor
):

    gps_plugin_configuration = localisation_configuration["plugins"]["gps"]

    gps_meta_description_file_path = get_sensor_meta_description_file_path(
        mode, robot_config_directory, gps_plugin_configuration["sensor_name"]
    )

    gps_meta_description = GPSMetaDescription(gps_meta_description_file_path)

    gps_specifications = get_gps_specifications(gps_meta_description)

    # gps_plugin_configuration.pop("sensor_name")

    return [
        gps_plugin_configuration,
        {"wgs84_anchor": wgs84_anchor},
        {"gps": gps_specifications},
        {"gps.rate": float(gps_meta_description.get_rate())},
        {"gps.xyz": gps_meta_description.get_xyz()},
    ]


def get_gps_plugin_remappings(robot_namespace, localisation_configuration):

    gps_plugin_configuration = localisation_configuration["plugins"]["gps"]
    gps_namespace = get_device_namespace(robot_namespace, gps_plugin_configuration)

    odo_plugin_configuration = localisation_configuration["plugins"]["odo"]
    controller_namespace = get_controller_namespace(
        robot_namespace, odo_plugin_configuration
    )

    return [
        ("gps/nmea_sentence", gps_namespace + "/nmea_sentence"),
        ("vehicle_controller/odom", controller_namespace + "/odom"),
    ]


def launch_setup(context, *args, **kwargs):

    mode = get_mode(context)
    robot_namespace = get_robot_namespace(context)
    demo_config_directory = get_demo_config_directory(context)
    robot_config_directory = demo_config_directory + "/robot"

    wgs84_anchor = get_wgs84_anchor(demo_config_directory)
    localisation_configuration = get_localisation_configuration(demo_config_directory)

    filter = Node(
        package="romea_robot_to_world_localisation",
        executable="robot_to_world_kalman_localisation_node",
        name="robot_to_world_kalman_localisation",
        parameters=get_filter_parameters(robot_namespace, localisation_configuration),
        output="screen",
    )

    odo_plugin_parameters = get_odo_plugin_parameters(
        mode, robot_config_directory, localisation_configuration
    )

    odo_plugin_remappings = get_odo_plugin_remappings(
        robot_namespace, localisation_configuration
    )

    odo_plugin = Node(
        package="romea_odo_localisation_plugin",
        executable="odo_localisation_plugin_node",
        name="odo_localisation_plugin",
        parameters=odo_plugin_parameters,
        remappings=odo_plugin_remappings,
        output="screen",
    )

    imu_plugin_parameters = get_imu_plugin_parameters(
        mode, robot_config_directory, localisation_configuration
    )

    imu_plugin_remappings = get_imu_plugin_remappings(
        robot_namespace, localisation_configuration
    )

    imu_plugin = Node(
        package="romea_imu_localisation_plugin",
        executable="imu_localisation_plugin_node",
        name="imu_localisation_plugin",
        parameters=imu_plugin_parameters,
        remappings=imu_plugin_remappings,
        output="screen",
    )

    gps_plugin_parameters = get_gps_plugin_parameters(
        mode, robot_config_directory, localisation_configuration, wgs84_anchor
    )

    gps_plugin_remappings = get_gps_plugin_remappings(
        robot_namespace, localisation_configuration
    )

    gps_plugin = Node(
        package="romea_gps_localisation_plugin",
        executable="gps_localisation_plugin_node",
        name="gps_localisation_plugin",
        parameters=gps_plugin_parameters,
        remappings=gps_plugin_remappings,
        output="screen",
    )

    return [
        GroupAction(
            actions=[
                SetParameter(name="use_sim_time", value=(mode != "live")),
                PushRosNamespace(robot_namespace),
                PushRosNamespace("localisation"),
                filter,
                odo_plugin,
                imu_plugin,
                gps_plugin,
            ]
        )
    ]


def generate_launch_description():

    declared_arguments = []

    declared_arguments.append(DeclareLaunchArgument("mode"))

    declared_arguments.append(DeclareLaunchArgument("robot_namespace"))

    declared_arguments.append(DeclareLaunchArgument("demo_config_directory"))

    return LaunchDescription(
        declared_arguments + [OpaqueFunction(function=launch_setup)]
    )
