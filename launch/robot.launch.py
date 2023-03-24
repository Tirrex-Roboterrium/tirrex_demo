from launch import LaunchDescription

from launch.actions import (
    IncludeLaunchDescription,
    DeclareLaunchArgument,
    OpaqueFunction,
)
from launch.substitutions import Command, LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import ExecutableInPackage

from tirrex_demo import (
    get_base_meta_description_file_path,
    get_joystick_meta_description_file_path,
    get_teleop_configuration_file_path,
)

from ament_index_python.packages import get_package_share_directory


def get_mode(context):
    return LaunchConfiguration("mode").perform(context)


def get_robot_namespace(context):
    return LaunchConfiguration("robot_namespace").perform(context)


def get_robot_configuration_directory(context):
    return LaunchConfiguration("robot_configuration_directory").perform(context)


def get_robot_urdf_description(context):
    return LaunchConfiguration("robot_urdf_description").perform(context)


def launch_setup(context, *args, **kwargs):

    mode = get_mode(context)
    robot_namespace = get_robot_namespace(context)
    configuration_directory = get_robot_configuration_directory(context)

    base_meta_description_file_path = get_base_meta_description_file_path(
        configuration_directory
    )

    joystick_meta_description_file_path = get_joystick_meta_description_file_path(
        configuration_directory, mode
    )

    teleop_configuration_file_path = get_teleop_configuration_file_path(
        configuration_directory
    )

    robot = []

    # print(get_robot_urdf_description(context))
    if "replay" not in mode:

        robot.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    get_package_share_directory("romea_mobile_base_bringup")
                    + "/launch/mobile_base.launch.py"
                ),
                launch_arguments={
                    "mode": mode,
                    "robot_namespace": robot_namespace,
                    "meta_description_file_path": base_meta_description_file_path,
                    "urdf_description": get_robot_urdf_description(context),
                }.items(),
            )
        )

        robot.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    get_package_share_directory("romea_joystick_bringup")
                    + "/launch/joystick_driver.launch.py"
                ),
                launch_arguments={
                    "robot_namespace": robot_namespace,
                    "meta_description_file_path": joystick_meta_description_file_path,
                }.items(),
            )
        )

    robot.append(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                get_package_share_directory("romea_teleop_bringup")
                + "/launch/mobile_base_teleop.launch.py"
            ),
            launch_arguments={
                "robot_namespace": robot_namespace,
                "base_meta_description_file_path": base_meta_description_file_path,
                "joystick_meta_description_file_path": joystick_meta_description_file_path,
                "teleop_configuration_file_path": teleop_configuration_file_path,
            }.items(),
        )
    )

    if mode == "live":
        robot.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    get_package_share_directory("tirrex_demo")
                    + "/launch/robot_devices.launch.py"
                ),
                launch_arguments={
                    "robot_namespace": robot_namespace,
                    "robot_configuration_directory": configuration_directory,
                }.items(),
            )
        )

    return robot


def generate_launch_description():

    declared_arguments = []

    declared_arguments.append(DeclareLaunchArgument("mode"))

    declared_arguments.append(DeclareLaunchArgument("robot_namespace"))

    declared_arguments.append(DeclareLaunchArgument("robot_configuration_directory"))

    robot_urdf_description = Command(
        [
            ExecutableInPackage("robot_description.py", "tirrex_demo"),
            " mode:",
            LaunchConfiguration("mode"),
            " robot_namespace:",
            LaunchConfiguration("robot_namespace"),
            " robot_configuration_directory:",
            LaunchConfiguration("robot_configuration_directory"),
        ]
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "robot_urdf_description", default_value=robot_urdf_description
        )
    )

    return LaunchDescription(
        declared_arguments + [OpaqueFunction(function=launch_setup)]
    )
