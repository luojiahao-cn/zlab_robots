# Xbox Controller Node

This package provides a ROS node for interfacing with an Xbox controller. It allows for reading the controller's inputs and publishing them as ROS messages.


## Usage

1. Source the workspace:
    ```sh
    source ~/catkin_ws/devel/setup.bash
    ```

2. Run the node:
    ```sh
    roslaunch xbox_controller_node xbox_controller.launch
    ```

## Topics

### Published Topics

- `/joy` (`sensor_msgs/Joy`): Publishes the state of the Xbox controller's buttons and axes.

## Nodes

### joy_node

Reads inputs from an Xbox controller and publishes them as ROS messages.

#### Parameters

- `~device` (string, default: "/dev/input/js0"): The device file for the Xbox controller.

