# Create a New Demo Configuration

A demo configuration assembles components that already exist in the ROMEA /
TIRREX workspace. Start from an existing configuration directory such as
`tirrex_adap2e/config`, then adapt the robot, devices, localisation,
navigation, simulation and recording files to the new use case.

If the robot, sensor, joystick, implement, arm or simulator is not supported
yet, add or extend the corresponding packages first, as described in the
previous chapter. Once the component support exists, the demo configuration only
selects it through YAML files and launch arguments.

A ROS2 package can be added around the configuration directory when the demo
must be distributed, versioned or launched with a short package-specific
command.

## Minimal Configuration Skeleton

```text
config/
  robot/
    base.yaml
    devices.yaml
    devices/
    localisation.yaml
    path_following.yaml
    path_matching.yaml
    teleop.yaml
  paths/
  simulation.yaml
  wgs84_anchor.yaml
  records.yaml
```

## Minimal Steps

1. Create the configuration directory.
2. Select the mobile base model in `config/robot/base.yaml`.
3. Create `config/robot/devices.yaml`.
4. Add one meta-description file per device in `config/robot/devices/`.
5. Add exactly one available joystick for each launch mode.
6. Add `wgs84_anchor.yaml` if localisation or simulated GPS is needed.
7. Add localisation configuration when the demo estimates the robot pose.
8. Add teleoperation configuration when the demo accepts joystick commands.
9. Add path following and path matching configuration when navigation demos
    are needed.
10. Add `simulation.yaml` if simulation is needed.
11. Add `records.yaml` if recording is needed.
12. Optionally create a ROS2 package and a wrapper launch file for convenience.

## Validation Checklist

Before launching, check:

* `demo_configuration_directory` points to the configuration directory;
* the `demo` argument identifies the demo consistently in logs and records;
* `robot_namespace` is consistent with topic remappings;
* `config/robot/base.yaml` selects a mobile base supported by its description
  and bringup packages;
* the corresponding hardware package exists when the demo is launched on a
  real robot;
* every device in `devices.yaml` has a matching file;
* exactly one joystick is available for the selected mode;
* the selected joystick mapping exists in `romea_joystick_utils/config/`;
* the selected joystick launch profile exists in
  `romea_joystick_meta_bringup/profile/`;
* each selected sensor has matching specifications and geometry files in the
  corresponding `<sensor_type>_description/config/` directory;
* each selected sensor driver profile exists in the corresponding
  `<sensor_type>_meta_bringup/profile/` directory;
* the selected simulator backend has a world entry in `simulation.yaml`;
* record directory paths exist or can be created.

## Configuration Extension Points

The previous chapter explains what must be added to the workspace when a
component is not supported yet. Once the component exists, a demo configuration
usually only changes the files listed below.

| Need | Component support checked in | Demo configuration change |
| --- | --- | --- |
| Use another mobile base model | `<robot_name>_description`, `<robot_name>_bringup` and, for real robots, `<robot_name>_hardware` | Update `config/robot/base.yaml` with the selected manufacturer, model and optional version fields expected by the robot description package. |
| Use another joystick model | `romea_joystick_utils` and `romea_joystick_meta_bringup` | Update the joystick meta-description in `config/robot/devices/` with the selected manufacturer, and model. |
| Add a sensor already supported by ROMEA | `<sensor_type>_description` and `<sensor_type>_meta_bringup` | Add one entry in `devices.yaml` and one device meta-description file with the selected manufacturer, model, optional version and device location. |
| Add an implement or arm already supported by ROMEA | `romea_implement_meta_bringup` or `romea_arm_meta_bringup` | Add one entry in `devices.yaml` and one device meta-description file with the selected manufacturer, model, optional version and device location. |
| Use another simulator backend | `romea_simulation_meta_bringup` and simulator-specific packages | Add or select a simulator entry in `simulation.yaml` and launch with the corresponding `mode`. |
| Change localisation, navigation, teleoperation or recording behaviour | The corresponding algorithm packages and `tirrex_core` launch files | Update the dedicated YAML files in `config/robot/`, add trajectories in `config/paths/` or update `records.yaml`. |
