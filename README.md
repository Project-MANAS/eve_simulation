# Eve Simulation

This repository contains the code for the simulation and plugins built for Eve, the driverless car project of Project MANAS, Manipal.

[Eve in simulation](images/logo.gif)    [Eve driving](images/road.gif)

## Packages

The packages included in this repository are as follows:

#### Simulation

- **eve_description**: Provides a URDF for Eve along with sensor plugins
- **eve_plugin**: Provides a Gazebo plugin for Eve implementing the Ackermann Steering Geometry along with PIDs to allow acceleration, brake, and steering capabilities.
- **eve_sim**: Provides a Gazebo environment with models for road, speed bump, stop sign, speed limit signs, trees, etc.


#### Interfacing

- **eve_control**: Provides keyboard and joystick controllers that can be used to control Eve in the environment
- **eve_msgs**: Control message to send commands to Eve


## Instructions

To launch the simulation:
```bash
roslaunch eve_sim world.launch controller:=keyboard (or joystick)
```
