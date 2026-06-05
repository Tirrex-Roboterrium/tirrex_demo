# TIRREX Demo Getting Started

This document is the entry point of the TIRREX demo user guide.

A TIRREX demo is primarily a configuration directory. This directory can be
stored in a ROS2 package, but `tirrex_core` only needs the path passed through
`demo_configuration_directory`.

The guide is split into focused chapters so that each configuration family can
be updated independently when the corresponding meta-bringup package evolves.

## Chapters

1. [Overview](getting_started/01_overview.md)
2. [Robot configuration](getting_started/02_robot.md)
3. [Geographic anchor configuration](getting_started/03_geographic_anchor.md)
4. [Localisation configuration](getting_started/04_localisation.md)
5. [Path following configuration](getting_started/05_path_following.md)
6. [Teleoperation configuration](getting_started/06_teleoperation.md)
7. [Simulation configuration](getting_started/07_simulation.md)
8. [Recording and replay](getting_started/08_recording_replay.md)
9. [Complete Adap2e walkthrough](getting_started/09_adap2e_walkthrough.md)
10. [Add a new component to the workspace](getting_started/10_add_new_component.md)
11. [Create a new demo configuration](getting_started/11_create_new_demo.md)
12. [PDF generation](getting_started/12_pdf_generation.md)

## PDF Generation

The full guide can be converted to PDF by assembling this file and all chapters:

```bash
cd src/tirrex/tirrex_demo
MERMAID_FILTER_FORMAT=pdf pandoc \
  doc/getting_started.md \
  doc/getting_started/01_overview.md \
  doc/getting_started/02_robot.md \
  doc/getting_started/03_geographic_anchor.md \
  doc/getting_started/04_localisation.md \
  doc/getting_started/05_path_following.md \
  doc/getting_started/06_teleoperation.md \
  doc/getting_started/07_simulation.md \
  doc/getting_started/08_recording_replay.md \
  doc/getting_started/09_adap2e_walkthrough.md \
  doc/getting_started/10_add_new_component.md \
  doc/getting_started/11_create_new_demo.md \
  doc/getting_started/12_pdf_generation.md \
  -F mermaid-filter \
  -o doc/getting_started.pdf \
  --toc \
  --number-sections \
  --top-level-division=chapter \
  -V documentclass=report \
  -V geometry:margin=2cm \
  --resource-path=doc/getting_started
```

## Quick Launch Example

```bash
ros2 launch tirrex_core demo.launch.py \
  demo:=my_demo \
  demo_start_timestamp:=manual \
  demo_configuration_directory:=/path/to/config \
  robot_namespace:=robot \
  mode:=simulation \
  record:=false
```
