# Dual-robot arm controller

## Description

Robot arm type: FR5
Arm1: Ram,  Arm2: Rem
Maximum weight of end-effector: 5kg

## Structure

The project is structured as follows:
- **apriltag**: Used for visual fiducial marker detection and localization.
  - **apriltag_ros**
- **MoveIt!**: Used for motion planning and execution.
  - **fr5v6_dual_moveit_config**: Used for motion planning and configuration of dual robot arms.
  - **fr5v6_single_moveit_config**: Used for motion planning and configuration of a single robot arm.
  - **frcobot_description**:Contains URDF and other description files for the FR5v6 robot arms.
  - **moveit_servo**:Used for real-time servoing of the robot arms. *Note: This feature is not fully implemented.*
- **ROS Controller**: Manages the control loops and interfaces with the hardware.
  - **ros_control_boilerplate**



