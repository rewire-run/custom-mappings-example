# rewire-custom-mappings-example

[![Pixi Badge](https://img.shields.io/endpoint?url=https://raw.githubusercontent.com/prefix-dev/pixi/main/assets/badge/v0.json)](https://pixi.sh)

Custom message publishers for testing [rewire](https://github.com/rewire-run/rewire) dynamic mappings.

Demonstrates how rewire can visualize arbitrary ROS 2 message types through JSON5 mapping configuration,
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

## Usage

```bash
pixi run launch
```

In a separate terminal, run rewire with the mapping configuration:

```bash
pixi run rewire record --all --config mappings.json5
```

## Project Structure

- **mappings.json5** — rewire dynamic mapping rules
- **custom_mappings/msg/** — ROS 2 message definitions (`.msg` files)
- **custom_mappings/src/publisher.cpp** — C++ publisher node
