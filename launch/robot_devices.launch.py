from launch import LaunchDescription

from launch.actions import (
    IncludeLaunchDescription,
    DeclareLaunchArgument,
    OpaqueFunction,
)
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource

from tirrex_demo import (
    get_available_devices,
    get_devices_meta_description,
    get_device_meta_description_filename,
)

from ament_index_python.packages import get_package_share_directory


def get_robot_namespace(context):
    return LaunchConfiguration("robot_namespace").perform(context)


def get_robot_configuration_directory(context):
    return LaunchConfiguration("robot_configuration_directory").perform(context)


def launch_setup(context, *args, **kwargs):

    robot_namespace = get_robot_namespace(context)
    configuration_directory = get_robot_configuration_directory(context)
    devices = get_devices_meta_description(configuration_directory)

    robot_devices = []
    for device_name in get_available_devices(devices, "live"):
        device_type = devices[device_name]["type"]

        if device_type != "joystick":

            meta_description_filename = get_device_meta_description_filename(
                configuration_directory, devices, device_name
            )

            robot_devices.append(
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        get_package_share_directory("romea_" + device_type + "_bringup")
                        + "/launch/"
                        + device_type
                        + "_driver.launch.py"
                    ),
                    launch_arguments={
                        "robot_namespace": robot_namespace,
                        "meta_description_filename": meta_description_filename,
                    }.items(),
                )
            )

    return robot_devices


def generate_launch_description():

    declared_arguments = []

    declared_arguments.append(DeclareLaunchArgument("robot_namespace"))

    declared_arguments.append(DeclareLaunchArgument("robot_configuration_directory"))

    return LaunchDescription(
        declared_arguments + [OpaqueFunction(function=launch_setup)]
    )
