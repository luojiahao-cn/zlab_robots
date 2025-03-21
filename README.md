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
  - **ros_control_boilerplate**：A ROS controller based on ServoJ commands.

## Installation

To install and set up the project, follow these steps:

1. Clone the repository:
    ```bash
    git clone https://github.com/luojiahao-cn/zlab_robots.git
    cd zlab_robots
    ```

2. Initialize and update submodules:
    ```bash
    git submodule update --init --recursive
    ```

3. Build the workspace:
    ```bash
    cd catkin_ws
    catkin_make
    ```

4. Source the workspace:
    ```bash
    source devel/setup.bash
    ```

## Usage

- To start the single arm controller, use the following command:

    ```bash
    roslaunch frcobot_examples connect_single_arm.launch
    ```
    ![connect_single_arm Image](images/connect_single_arm.png)

- To start the dual-robot arm controller, use the following command:

    ```bash
    roslaunch frcobot_examples connect_dual_arms.launch
    ```
    ![connect_dual_arms Image](images/connect_dual_arms.png)
