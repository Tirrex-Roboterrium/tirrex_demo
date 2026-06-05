# Add a New Component

Before creating a new demo configuration, check whether every component used by
the demo already exists in the ROMEA / TIRREX workspace. A demo configuration
only selects and assembles existing components. If the current ROMEA ecosystem
does not yet support a robot, sensor, joystick, arm or implement, the
corresponding packages must be added or extended first.

This chapter gives the user-level checklist for that work. The detailed
implementation remains in the README files of each package family.

## General Rule

A new component is usually added in three steps:

1. add the description data or Python API that represents the component model, geometry and configuration;
2. expose the scripts, launch files, drivers or controllers needed to generate and run the component;
3. add or update the corresponding `meta_bringup` package so that a demo meta-description can select the component and its launch profile.

Once these steps are done, the demo only needs a new meta-description file and
an entry in `config/robot/devices.yaml` or `config/robot/base.yaml`.

## New Mobile Base

A new mobile base requires the packages that describe, control and optionally
interface with the platform.

| Package family | Purpose |
| --- | --- |
| `<robot_name>_description` | URDF, geometry, kinematic parameters and base configuration files. |
| `<robot_name>_hardware` | Hardware interface when the real robot is used. |
| `<robot_name>_bringup` | Launch files, controller configuration and test launch files. |

The description package must expose a small Python API used by the bringup and
meta-bringup layers. For a mobile base with several variants, this API receives
a `robot_model` argument; for a single-model robot, this argument can be omitted
or kept with a fixed value.

---

Expected description API:

```python
def get_specifications_path_file(robot_model): ...
def get_specifications_configuration(robot_model): ...
def get_configuration(robot_model): ...
def generate_configuration_file(configuration, extended): ...
def generate_ros2_control_description(prefix, mode, base_name, robot_model): ...
def generate_urdf_description(
    prefix,
    mode,
    base_name,
    robot_model,
    controller_manager_config_yaml_file,
    ros_prefix,
): ...
```

The bringup package usually wraps this description API and injects the
controller manager configuration used by the robot launch files.

Expected bringup API:

```python
def get_configuration(robot_model): ...
def generate_configuration_file(robot_model, extended): ...
def generate_ros2_control_description(prefix, mode, base_name, robot_model): ...
def generate_urdf_description(prefix, mode, base_name, robot_model, ros_prefix): ...
```

The bringup package should also install command-line scripts that expose these
generators, typically:

```text
generate_configuration_file.py
generate_urdf_description.py
generate_ros2_control_description.py
```

---

Typical commands are:

```bash
ros2 run adap2e_bringup generate_configuration_file.py \
  robot_model:two \
  extended:true

ros2 run adap2e_bringup generate_urdf_description.py \
  mode:simulation \
  base_name:base \
  robot_model:two \
  robot_namespace:adap2e

ros2 run adap2e_bringup generate_ros2_control_description.py \
  mode:simulation \
  base_name:base \
  robot_model:two \
  robot_namespace:adap2e
```

In the bringup package, the minimum useful launch set is a base launch file and
a teleoperation launch file. The base launch file must start the low-level
controllers needed by the mobile base and the command mux associated with these
controllers. The command mux is used to select the active command among several
sources, for example teleoperation, autonomous path following or another command
producer. Additional simulation, test or implement launch files are useful, but
they are usually added once the base runtime is already working.

For a real robot, the hardware package usually provides a hardware interface
class derived from the generic interfaces available for each mobile base family
in `romea_mobile_base_hardware`. The user implementation then only has to write
the robot-specific communication code: send joint commands to the low-level
robot controller and read the odometry or joint feedback returned by the robot.
These joints can be wheel joints, but also joints belonging to devices that move
an implement, for example a three-point hitch.

## New Sensor

A new sensor must first be supported by its sensor family: GPS, IMU, lidar,
camera or another future modality.

| Sensor family | Typical packages to update |
| --- | --- |
| GPS | `romea_gps_description`, `romea_gps_meta_bringup`, driver packages. |
| IMU | `romea_imu_description`, `romea_imu_meta_bringup`, driver packages. |
| Lidar | `romea_lidar_description`, `romea_lidar_meta_bringup`, driver packages. |
| Camera | `romea_camera_description`, `romea_camera_meta_bringup`, driver packages. |

For most sensor families, the description package must provide two files: one
specifications file that describes the sensor model, and one geometry file that
describes how this model is represented in the robot description. The exact
content of these files depends on the sensor family, so the corresponding
`<sensor_name>_description` README remains the reference. The
`romea_common_description` documentation also explains the common rules used to
write specifications files.

The usual file naming convention is:

```text
<manufacturer>_<model>_<version>_specifications.yaml
<manufacturer>_<model>_<version>_geometry.yaml
```

Examples:

```text
xsens_mti_6xx_specifications.yaml
xsens_mti_6xx_geometry.yaml
sick_tim_5xx_specifications.yaml
sick_tim_5xx_geometry.yaml
```

When the version is not specified, the empty version field is kept, which
produces two consecutive underscores:

```text
xsens_mti__specifications.yaml
xsens_mti__geometry.yaml
```

The model or version field can also describe a family of sensors rather than a
single product, for example `sick_tim_5xx` or `xsens_mti_6xx`.

Example of specifications file:

```yaml
version:
  list: ["610", "620", "630"]
type:
  depend: version
  dict:
    "610": IMU
    "620": VRU
    "630": AHRS
rate:
  default: 100
  list: [1, 2, 4, 5, 10, 20, 40, 50, 80, 100, 200, 400]
acceleration_noise_density: 588.6e-06
acceleration_bias_stability_std: 98.1e-06
acceleration_range: 98.1
angular_speed_noise_density: 0.007
angular_speed_bias_stability_std: 8
angular_speed_range: 2000
```

Example of geometry file:

```yaml
mass: 0.075
aabb:
  length: 0.057
  width: 0.041
  height: 0.025
  center: [0.0, 0.0, 0.0]
mesh:
  xyz: [0.0, 0.0, 0.0]
  rpy: [0.0, 0.0, 0.0]
  scale: [0.001, 0.001, 0.001]
  filename: package://my_imu_description/meshes/imu.stl
```

GPS descriptions are slightly different because the receiver and the antenna
are described separately: receiver specifications are stored in the receiver
configuration directory, while antenna geometry is stored in the antenna
configuration directory.

In the device meta-description, these files are selected from the
`manufacturer`, `model` and `version` fields. Additional parameters are then
chosen from the available specifications. Typical examples are the acquisition
rate for an IMU, the image size for a camera or the horizontal resolution for
a lidar.

From this meta-description, the meta-bringup package can generate the URDF
fragment, the complete sensor configuration file and the launch parameters
needed by the selected mode.

Example IMU meta-description:

```yaml
name: imu
launch:
  - include:
      file: "$(find-pkg-share romea_imu_meta_bringup)/profile/xsens_driver.launch.py"
      arg:
        - name: device
          value: /dev/ttyUSB0
        - name: baudrate
          value: "115200"
configuration:
  manufacturer: xsens
  model: mti
  rate: 100
location:
  parent_link: base_link
  xyz: [0.0, 0.0, 0.7]
  rpy: [0.0, 0.0, 0.0]
records:
  data: true
```

The meta-bringup package must provide at least one launch profile, stored in its
`profile/` directory, able to start the sensor driver used by the selected mode.

## New Sensor Type

Adding a new sensor type, for example radar, is a larger task than adding a new
model to an existing sensor family. A new family must define its own description
package, meta-bringup package and driver profiles before a demo configuration
can reference it.

Typical packages to create are:

| Package family | Purpose |
| --- | --- |
| `romea_<sensor_type>_description` | Sensor specifications, geometry files, units and Python helpers used to generate configuration and descriptions. |
| `romea_<sensor_type>_meta_bringup` | Meta-description parser, URDF generation, configuration generation and launch profiles. |
| Driver packages | Real sensor drivers, vendor bridges or simulation bridges for this modality. In many cases the driver already exists because it is provided by the manufacturer. If no suitable driver exists yet, it can be added to the ROMEA ecosystem. |

The description package must first define the data model of the new modality:
which specifications are required, which geometry fields are supported, which
URDF or mesh resources are needed, and how these files are named. The
meta-bringup package then reads a device meta-description, selects the matching
specifications and geometry files, and generates the URDF fragment, runtime
configuration and launch files.

The description package should expose a Python API similar to the existing
sensor description packages:

---

```python
def get_specifications_file_path(sensor_description): ...
def get_specifications(sensor_description): ...
def get_geometry_file_path(sensor_description): ...
def get_geometry(sensor_description): ...
def get_specification_units_file_path(): ...
def get_specification_units(): ...
def get_complete_configuration(sensor_name, sensor_description, sensor_location): ...
def generate_configuration_file(configuration, extended): ...
def generate_urdf_description(
    prefix,
    mode,
    sensor_name,
    sensor_description,
    sensor_location,
    ros_namespace,
    standalone=False,
): ...
```

In this API, `sensor_description` is the configuration dictionary extracted from
the device meta-description, typically containing `manufacturer`, `model`,
`version` and other sensor-specific options. `sensor_location` is the location
dictionary extracted from the same meta-description, typically containing
`parent_link`, `xyz` and `rpy`.

The meta-bringup package should expose the helpers used by launch files to read
the device meta-description, resolve the matching description files and generate
the final runtime artifacts:

```python
class SensorTypeMetaDescription(SensorMetaDescription): ...
def load_meta_description(meta_description_file_path, robot_name=None): ...
def get_specifications(meta_description): ...
def get_geometry(meta_description): ...
def get_complete_configuration(meta_description): ...
def generate_yaml_configuration_file_str(meta_description, extended): ...
def generate_yaml_launch_file_str(meta_description): ...
def generate_xml_urdf_description_str(mode, meta_description, standalone=False): ...
```

---

It should also install command-line scripts that expose these generators,
typically:

```text
generate_configuration_file.py
generate_urdf_description.py
```

Typical commands for a radar family would look like:

```bash
ros2 run romea_radar_meta_bringup generate_configuration_file.py \
  manufacturer:my_manufacturer \
  model:my_radar \
  version:one \
  extended:true

ros2 run romea_radar_meta_bringup generate_urdf_description.py \
  name:radar \
  manufacturer:my_manufacturer \
  model:my_radar \
  version:one \
  robot_namespace:adap2e
```

Example radar meta-description:

```yaml
name: radar
launch:
  - include:
      file: "$(find-pkg-share romea_radar_meta_bringup)/profile/radar_driver.launch.py"
      arg:
        - name: ip
          value: 192.168.1.120
configuration:
  manufacturer: my_manufacturer
  model: my_radar
  rate: 20
location:
  parent_link: base_link
  xyz: [1.2, 0.0, 0.8]
  rpy: [0.0, 0.0, 0.0]
records:
  detections: true
```

If the new sensor type is used by localisation, navigation or perception, the
corresponding algorithm packages must also be extended to consume the data
produced by the new driver.

## New Joystick

A new joystick must be described in the joystick tooling before it can be
selected by a demo.

| Package family | Purpose |
| --- | --- |
| `romea_joystick_utils` | Joystick mapping, axes and button conventions. |
| `romea_joystick_meta_bringup` | Meta-description parsing and launch profile generation. |

The joystick mapping is added to the `romea_joystick_utils` configuration
directory. The file name follows:

```text
<manufacturer>_<model>.yaml
```

Example joystick mapping:

```yaml
type: microsoft_xbox
joy_msg_layout:
  buttons:
    mapping:
      A: 0
      B: 1
      X: 2
      Y: 3
    values:
      unpressed: 0
      pressed: 1
  axes:
    sticks:
      mapping:
        Horizontal_Left_Stick: 0
        Vertical_Left_Stick: 1
      values:
        range: [-1.0, 1.0]
```

This mapping defines how the ROS2 joystick message indexes correspond to the
physical buttons, sticks, triggers and directional pads. It is then reused by
teleoperation and control algorithms.

The joystick meta-bringup package must also provide a launch profile dans le répertoire profiles for the
joystick driver used in the selected mode.

Example joystick meta-description:

```yaml
name: joystick
launch:
  - include:
      file: "$(find-pkg-share romea_joystick_meta_bringup)/profile/joy.launch.py"
configuration:
  manufacturer: microsoft
  model: xbox
  rate: 10
records:
  joy: true
```

For a given launch mode, the configuration must expose exactly one available
joystick.

## New Arm

This documentation will be completed soon.

## New Implement

This documentation will be completed soon.

## New Simulator

This documentation will be completed soon.
