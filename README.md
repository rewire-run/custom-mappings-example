# rewire-custom-mappings-example

[![Pixi Badge](https://img.shields.io/endpoint?url=https://raw.githubusercontent.com/prefix-dev/pixi/main/assets/badge/v0.json)](https://pixi.sh)

Custom message publishers for testing [Rewire](https://github.com/rewire-run/rewire) dynamic mappings.

Demonstrates how Rewire can visualize arbitrary ROS 2 message types through JSON5 mapping configuration,
without requiring built-in converter support.

## Message Types

| Type | Rerun Archetypes | Description |
|---|---|---|
| `MotorStatus` | `Scalars`, `TextLog` | Temperature, RPM, status text |
| `Detections` | `Points3D`, `Scalars` | 3D detections with labels |
| `RigidBodies` | `InstancePoses3D`, `Points3D`, `Scalars` | Named rigid bodies with markers |
| `GpsReading` | `GeoPoints` | GPS coordinates |
| `ForceVector` | `Arrows3D`, `Scalars` | Force arrow with magnitude |
| `Detections2D` | `Points2D`, `Scalars` | 2D detections with labels |
| `BoundingBoxes2D` | `Boxes2D` | 2D bounding boxes with labels |
| `BoundingBoxes3D` | `Boxes3D` | 3D bounding boxes with labels |
| `Trajectory` | `LineStrips3D` | Waypoint path |

## Setup

Requires [pixi](https://pixi.sh).

```bash
pixi install
```

This project uses [pixi-build-ros](https://github.com/nicross/pixi-build-ros) to build the custom_mappings ROS package as a conda dependency. The `ros-humble-custom-mappings` (or `ros-jazzy-custom-mappings`) package is built from source and installed into the pixi environment automatically.

## Environments

| Environment | ROS Distro |
|---|---|
| `default` / `jazzy` | Jazzy |
| `Humble` | Humble |

## Usage

```bash
pixi run ros2 run custom_mappings publisher
```

In a separate terminal, run `rewire` with the mapping configuration:

```bash
pixi run rewire record --all --config mappings.json5
```

## Development

After modifying the C++ package in `custom_mappings/`, reinstall the local package:

```bash
pixi reinstall ros-humble-custom-mappings
```
