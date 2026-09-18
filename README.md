# usv_controller

ROS 2 C++ controller for **Selene waypoint following and joystick manual override**. It reads the boat pose through TF, computes position/heading guidance, allocates thrust across four pods, and publishes direct motor and servo commands to PX4.

## Role in the system

- Waypoint commands and status use [waypoint_msgs](https://github.com/Sub-Horizon-NTNU/waypoint_msgs); mission programs live in [usv_mission_package](https://github.com/Sub-Horizon-NTNU/usv_mission_package).
- Boat transforms are supplied by [transform_broadcaster](https://github.com/Sub-Horizon-NTNU/transform_broadcaster).
- Manual commands, override, stop and arm requests come from [usv_teleop](https://github.com/Sub-Horizon-NTNU/usv_teleop).
- PX4 communication uses `px4_msgs` and the `/fmu/in/offboard_control_mode`, `/fmu/in/actuator_motors`, `/fmu/in/actuator_servos` and `/fmu/in/vehicle_command` topics.
- [usv_bringup](https://github.com/Sub-Horizon-NTNU/usv_bringup) assembles the companion-side stack and DDS bridge.

This version uses PX4 direct actuator control. The separate ArduPilot-based simulator is not automatically compatible with these output interfaces.

## Where to look

- [src/usv_controller/PositionController.cpp](src/usv_controller/PositionController.cpp): guidance, manual override, PX4 publishing and controller parameters.
- [include/usv_controller/OmniXAllocator.hpp](include/usv_controller/OmniXAllocator.hpp): thrust allocation.
- [src/usv_controller/WaypointManager.cpp](src/usv_controller/WaypointManager.cpp): waypoint handling.
- [config/tuning.yaml](config/tuning.yaml): tuning values loaded by the launch file.
- [launch/usv_controller.launch.py](launch/usv_controller.launch.py): `config` and `initial_heading` arguments.

## Build and operation

Place this package, `waypoint_msgs` and a firmware-compatible `px4_msgs` in a ROS 2 workspace. After sourcing ROS 2, build from the workspace root:

```sh
colcon build --packages-up-to usv_controller
source install/setup.bash
```

The launch entry point is `ros2 launch usv_controller usv_controller.launch.py`. It is an actuator-control process: hardware channel assignment, thrust geometry, servo limits and firmware/message compatibility must match the vehicle before operation. This README describes the code; it does not establish that the current tuning has been validated on the boat.
