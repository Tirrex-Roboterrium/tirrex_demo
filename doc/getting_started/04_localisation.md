# Localisation Configuration

## Principle

Localisation estimates the robot pose in a local metric frame. In TIRREX, this
frame is an ENU frame attached to the WGS84 anchor defined by the demo. The
anchor latitude, longitude and altitude define the geographic position of the
local origin, which corresponds to `(0, 0, 0)` in the ENU frame.

The localisation pipeline is organised in three steps. First, sensors and
controllers publish ROS2 data such as mobile base odometry, NMEA sentences or
IMU measurements. Then localisation plugins convert these data streams into
typed localisation observations. Finally, the robot-to-world Kalman filter fuses
these observations and publishes the filtered robot pose, odometry, TF
transform, status and diagnostics.

Figure 5 - Localisation data flow:

```mermaid
flowchart LR
  subgraph sources["Robot data sources"]
    base["Mobile base controller"]
    gps_sensor["GPS receiver"]
    imu_sensor["IMU"]
  end

  subgraph ros2_topics["ROS2 data topics"]
    odom_topic["controller odometry<br/>nav_msgs/msg/Odometry"]
    nmea_topic["gps/nmea_sentence<br/>nmea_msgs/msg/Sentence"]
    imu_topic["imu/data<br/>sensor_msgs/msg/Imu"]
  end

  subgraph plugins["Localisation plugin nodes"]
    odo_plugin["romea_localisation_odo_plugin<br/>controller odometry"]
    gps_plugin["romea_localisation_gps_plugin<br/>GPS NMEA data"]
    imu_plugin["romea_localisation_imu_plugin<br/>IMU data"]
  end

  subgraph observation_inputs["Localisation observations"]
    twist["twist<br/>ObservationTwist2DStamped"]
    position["position<br/>ObservationPosition2DStamped"]
    course["course<br/>ObservationCourseStamped"]
    angular_speed["angular_speed<br/>ObservationAngularSpeedStamped"]
    attitude["attitude<br/>ObservationAttitudeStamped"]
  end

  subgraph localisation_core["Robot-to-world localisation core"]
    filter["localisation filter<br/>romea_core_localisation"]
  end

  subgraph outputs["ROS2 outputs"]
    filtered_odom["filtered_odom<br/>Odometry"]
    tf["map -> base_footprint<br/>TF transform"]
    status["status<br/>LocalisationStatus"]
    diagnostics["diagnostics<br/>diagnostic report"]
  end

  base --> odom_topic
  gps_sensor --> nmea_topic
  imu_sensor --> imu_topic

  odom_topic --> odo_plugin
  odom_topic --> gps_plugin
  odom_topic --> imu_plugin
  nmea_topic --> gps_plugin
  imu_topic --> imu_plugin

  odo_plugin --> twist
  gps_plugin --> position
  gps_plugin --> course
  imu_plugin --> angular_speed
  imu_plugin --> attitude

  twist --> filter
  position --> filter
  course --> filter
  angular_speed --> filter
  attitude --> filter

  filter --> filtered_odom
  filter --> tf
  filter --> status
  filter --> diagnostics

  classDef pluginStyle fill:#eaf7ea,stroke:#5c9f5c,color:#111,rx:6,ry:6
  classDef obs fill:#fff6d8,stroke:#c9a227,color:#111,rx:6,ry:6
  classDef coreStyle fill:#f1eaff,stroke:#8b6fc6,color:#111,rx:6,ry:6
  classDef ros2 fill:#e8f2ff,stroke:#5b8ec7,color:#111,rx:6,ry:6

  class base,gps_sensor,imu_sensor ros2
  class odom_topic,nmea_topic,imu_topic ros2
  class odo_plugin,gps_plugin,imu_plugin pluginStyle
  class twist,position,course,angular_speed,attitude obs
  class filter coreStyle
  class filtered_odom,tf,status,diagnostics ros2

  style sources fill:#f6faff,stroke:#9abbe3,rx:6,ry:6
  style ros2_topics fill:#f6faff,stroke:#9abbe3,rx:6,ry:6
  style plugins fill:#f7fff7,stroke:#9ecf9e,rx:6,ry:6
  style observation_inputs fill:#fffaf0,stroke:#dec86b,rx:6,ry:6
  style localisation_core fill:#faf7ff,stroke:#b8a4dd,rx:6,ry:6
  style outputs fill:#f6faff,stroke:#9abbe3,rx:6,ry:6
```



## Configuration

Configuring localisation therefore involves several files. The WGS84 anchor is
defined once for the demo, as described in the previous chapter. The Kalman
filter is configured in `robot/localisation.yaml`, and the localisation plugin
launch profiles are declared in the meta-descriptions of the mobile base, GPS
receivers and IMUs:

```text
config/
  wgs84_anchor.yaml            geographic reference defined before localisation
  robot/
    localisation.yaml          edited in this chapter
    base.yaml                  mobile base launch profiles
    devices/
      *.gps.yaml               GPS launch profiles
      *.imu.yaml               IMU launch profiles
```

The TIRREX localisation launch file starts `romea_robot_to_world_localisation_core/robot_to_world_kalman_localisation_node` under the robot namespace, in the `localisation` namespace. It also sets the robot base frame from the selected robot namespace.

This node does not read sensors directly. It consumes typed observations from `romea_localisation_msgs` and feeds them to the robot-to-world Kalman filter.

The geographic anchor must be defined before localisation because GPS
observations are converted from WGS84 coordinates into the local ENU frame used
by the filter.

Example `localisation.yaml`:

```yaml
filter:
  state_pool_size: 1000
predictor:
  maximal_dead_recknoning_travelled_distance: 2.
  maximal_dead_recknoning_elapsed_time: 10.
angular_speed_updater:
  minimal_rate: 10
twist_updater:
  minimal_rate: 0
linear_speeds_updater:
  minimal_rate: 10
position_updater:
  minimal_rate: 1
  trigger: always
course_updater:
  minimal_rate: 1
  trigger: once
pose_updater:
  minimal_rate: 0
  trigger: always
attitude_updater:
  minimal_rate: 10
publish_rate: 10
debug: true
```

This file configures the filter itself and declares which observation updaters
are used for data fusion. In the example above, the filter uses angular speed,
left/right linear speeds, GPS position, GPS course and IMU attitude. The full
twist updater is disabled because its `minimal_rate` is set to `0`.

In this configuration, the filter reconstructs the robot motion from
`linear_speeds_updater` and `angular_speed_updater`, using observations produced
by two different plugins: the mobile base odometry plugin and the IMU
localisation plugin. If no IMU is available, a simpler configuration can use
`twist_updater` instead, which consumes the complete twist observation produced
by the odometry plugin. The same idea applies to absolute pose information: this
example uses `position_updater` and `course_updater` because position and course
are produced separately from NMEA data. If a sensor or another algorithm
publishes a complete pose observation, `pose_updater` can be used directly.

| Parameter | Meaning |
| --------- | ------- |
| `filter.state_pool_size` | Number of timestamped states kept by the asynchronous Kalman filter. |
| `predictor.maximal_dead_recknoning_travelled_distance` | Maximum travelled distance allowed while the filter is only dead-reckoning. |
| `predictor.maximal_dead_recknoning_elapsed_time` | Maximum elapsed time allowed while the filter is only dead-reckoning. |
| `<updater>.minimal_rate` | Minimal expected observation rate. A value of `0` disables the updater. |
| `<updater>.trigger` | Update policy for exteroceptive observations. `always` uses every valid observation; `once` uses the first valid observation, typically for initialisation. |
| `<updater>.mahalanobis_distance_rejection_threshold` | Optional outlier rejection threshold for pose-related observations. |
| `publish_rate` | Rate used to publish filtered odometry, status and diagnostics. |
| `debug` | Enables localisation debug outputs when supported by the node. |

The dead-reckoning limits protect the localisation from running indefinitely on
motion prediction only. If the travelled distance or elapsed time limits are
exceeded while no valid exteroceptive observation is available, the localisation
is reset. It can start again only when the observations required for
initialisation become available again.

The `minimal_rate` of each updater is also part of the filter integrity checks.
When an updater does not receive data at the expected rate, its observations are
not used. Depending on the role of this updater, this can prevent the filter
from starting or force a reset while the filter is already running.

Exteroceptive updaters, such as position, course or pose, also use a
Mahalanobis distance test. This rejects observations that are inconsistent with
the current filter state. Rejected observations are not used to update the
filter, so the localisation continues in dead-reckoning until a valid
exteroceptive observation is accepted.

For absolute pose information, use either `pose_updater` when a single source
provides a complete pose observation, or a combination of `position_updater` and
`course_updater` when position and course come from different sources.

The most common updaters are:

| Updater | Observation topic | Typical producer |
| ------- | ----------------- | ---------------- |
| `twist_updater` | `twist` | Mobile base odometry plugin. |
| `linear_speed_updater` | `twist` | Mobile base odometry plugin, when a single longitudinal speed is used. |
| `linear_speeds_updater` | `twist` | Mobile base odometry plugin, when left/right speeds are used. |
| `angular_speed_updater` | `angular_speed` | IMU localisation plugin. |
| `attitude_updater` | `attitude` | IMU localisation plugin. |
| `position_updater` | `position` | GPS localisation plugin. |
| `course_updater` | `course` | GPS localisation plugin. |
| `range_updater` | `range` | Optional RTLS localisation plugin. |
| `pose_updater` | `pose` | Optional pose-producing localisation plugin. |

## Launch Flow

The localisation runtime is assembled by two TIRREX launch files.
`robot/robot.launch.py` starts the mobile base, GPS and IMU localisation plugins
through the corresponding meta-description launch profiles. These plugins
convert raw ROS2 topics into typed localisation observations. In parallel,
`robot/robot_localisation.launch.py` starts the robot-to-world Kalman
localisation node from `robot/localisation.yaml`. The Kalman node then consumes
the observations produced by the plugins.

Figure 6 - Localisation launch flow:

```mermaid
flowchart LR
  subgraph tirrex["TIRREX core"]
    direction LR
    subgraph config["Configuration"]
      direction TB
      localisation_yaml["localisation.yaml"]
      base_yaml["base.yaml"]
      gps_yaml["*.gps.yaml"]
      imu_yaml["*.imu.yaml"]
      anchor["wgs84_anchor.yaml"]
    end

    subgraph robot_launch["robot/robot.launch.py"]
      direction TB
      plugin_profiles["localisation plugin profiles"]
    end

    subgraph localisation_launch["robot/robot_localisation.launch.py"]
      direction TB
      node_args["localisation node parameters"]
    end
  end

  subgraph runtime["Localisation runtime"]
    direction LR
    subgraph plugins["Localisation plugins"]
      direction TB
      odo_plugin["odometry plugin"]
      gps_plugin["GPS plugin"]
      imu_plugin["IMU plugin"]
    end

    subgraph core["Robot-to-world localisation core"]
      direction TB
      filter["robot_to_world_kalman_localisation_node"]
    end
  end

  localisation_yaml --> localisation_launch
  base_yaml --> robot_launch
  gps_yaml --> robot_launch
  imu_yaml --> robot_launch
  anchor --> robot_launch
  robot_launch --> plugins
  localisation_launch --> filter
  odo_plugin --> filter
  gps_plugin --> filter
  imu_plugin --> filter

  classDef configStyle fill:#e8f2ff,stroke:#5b8ec7,color:#111,rx:6,ry:6
  classDef launch fill:#eaf7ea,stroke:#5c9f5c,color:#111,rx:6,ry:6
  classDef coreStyle fill:#f1eaff,stroke:#8b6fc6,color:#111,rx:6,ry:6

  class localisation_yaml,base_yaml,gps_yaml,imu_yaml,anchor configStyle
  class plugin_profiles,node_args,odo_plugin,gps_plugin,imu_plugin launch
  class filter coreStyle

  style config fill:#f6faff,stroke:#9abbe3,rx:6,ry:6
  style robot_launch fill:#eaf7ea,stroke:#5c9f5c,rx:6,ry:6
  style localisation_launch fill:#eaf7ea,stroke:#5c9f5c,rx:6,ry:6
  style tirrex fill:#fff3e6,stroke:#d89b55,rx:6,ry:6
  style plugins fill:#f7fff7,stroke:#9ecf9e,rx:6,ry:6
  style core fill:#faf7ff,stroke:#b8a4dd,rx:6,ry:6
  style runtime fill:#fffaf0,stroke:#d4b24f,rx:6,ry:6
```

## Localisation Plugin Profiles

The Kalman filter consumes observations, not raw sensor messages. The
localisation plugins that produce these observations must therefore be launched
from the corresponding mobile base, GPS and IMU meta-descriptions.

Typical localisation plugin launch entries:

```yaml
# Mobile base meta-description
launch:
  - include:
      file: "$(find-pkg-share romea_mobile_base_meta_bringup)/profile/localisation_plugin.launch.py"
      arg:
        - name: controller_topic
          value: odom
```

The mobile base profile starts the odometry localisation plugin. It subscribes
to the selected controller topic and publishes the motion observations used by
the `twist_updater`, `linear_speed_updater` or `linear_speeds_updater`.

```yaml
# GPS device meta-description
launch:
  - include:
      file: "$(find-pkg-share romea_gps_meta_bringup)/profile/localisation_plugin.launch.py"
      arg:
        - name: wgs84_anchor_file_path
          value: $(var wgs84_anchor_file_path)
        - name: odom_topic
          value: /$(var robot_namespace)/base/controller/odom
```

The GPS profile starts either the single-antenna or dual-antenna GPS
localisation plugin, depending on the GPS device configuration. The plugin uses
`wgs84_anchor.yaml` to express GPS fixes in the local ENU frame. A
single-antenna GPS also uses vehicle odometry to determine whether the robot is
moving forward or backward before converting GPS track angle into robot course.

```yaml
# IMU device meta-description
launch:
  - include:
      file: "$(find-pkg-share romea_imu_meta_bringup)/profile/localisation_plugin.launch.py"
      arg:
        - name: odom_topic
          value: /$(var robot_namespace)/base/controller/odom
```

The IMU profile starts the IMU localisation plugin. It publishes angular speed
and attitude observations. The odometry input is used to detect stationary
phases and support angular speed bias estimation.
