# MotionLinker

## Description
MotionLinker is a RobotStudio Smart Component that synchronizes the motion of a source ABB controller with a target virtual controller. It supports both Joint and Cartesian synchronization modes.

## Features

- Joint motion synchronization
- Cartesian motion synchronization
- Automatic switching between Joint and Cartesian modes depending on controller state
- Automatic controller discovery
- Support for online and offline controllers
- Synchronization of global and local `tooldata` and `wobjdata` between Source and Target
- Automatic update and visualization of active `tooldata` and `wobjdata` on the Target controller
- Support for coordinated WorkObjects

## Requirements

- RobotStudio 2025
- RobotStudio SDK 2025
- PC SDK 2025
- .NET Framework 4.8
- A station with the same mechanical unit and a *dummy* controller
- The `T_ROB1` task must contain a main program that remains running and does not execute motion instructions that interfere with synchronization

## Configuration / Properties

| Property | Type | Description |
|-----------|------:|-------------|
| SourceController | String | Source controller |
| TargetController | String | Target controller |
| OnlineSource | Bool | Search only online controllers |
| Cartesian | Bool | Enable Cartesian synchronization |
| CoordinatedWObjs | Bool | Enable coordinated WorkObject support |

## Quick Start

1. Add MotionLinker to the station
2. Select `SourceController`
3. Select `TargetController`
4. Choose `Joint` or `Cartesian` mode
5. Start the simulation

![MotionLinker demo](./assets/MotionLinker.gif)

## Known Limitations

- Coordinated WorkObjects require a more expensive inverse kinematics strategy. Standard matrix-based calculations are faster but do not support coordinated WorkObjects.
- Cartesian synchronization may not be reliable in Manual mode. MotionLinker automatically switches to Joint synchronization in this case.
- Only the `T_ROB1` task is supported
- RobotStudio SDK and PC SDK are only compatible with `.NET Framework 4.8`
- Source and Target controllers must use the same mechanical unit

## Documentation

- [Architecture and workflow](architecture.md)