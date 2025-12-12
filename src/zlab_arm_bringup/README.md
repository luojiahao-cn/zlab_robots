# ZLab Arm Bringup

这个包提供了启动 ZLab 机械臂硬件和 MoveIt 的便捷启动文件。

## 启动文件

### 真实硬件启动

#### 单臂启动
```bash
# 启动 arm1（默认 IP: 192.168.31.202）
roslaunch zlab_arm_bringup single_arm_bringup.launch arm_id:=arm1

# 指定 IP 地址
roslaunch zlab_arm_bringup single_arm_bringup.launch arm_id:=arm1 robot_ip:=192.168.31.202

# 指定工具
roslaunch zlab_arm_bringup single_arm_bringup.launch arm_id:=arm1 tool_name:=permanent_magnet

# 不启动 RViz
roslaunch zlab_arm_bringup single_arm_bringup.launch arm_id:=arm1 use_rviz:=false
```

#### 双臂启动
```bash
# 使用默认 IP 地址
roslaunch zlab_arm_bringup dual_arms_bringup.launch

# 指定 IP 地址
roslaunch zlab_arm_bringup dual_arms_bringup.launch \
    arm1_ip:=192.168.31.202 \
    arm2_ip:=192.168.31.203

# 指定工具
roslaunch zlab_arm_bringup dual_arms_bringup.launch \
    arm1_tool:=permanent_magnet \
    arm2_tool:=electronic_magnet
```

### Gazebo 仿真启动

#### 单臂仿真
```bash
# 启动单臂 Gazebo 仿真
roslaunch zlab_arm_bringup single_arm_gazebo.launch arm_id:=arm1

# 指定工具
roslaunch zlab_arm_bringup single_arm_gazebo.launch arm_id:=arm1 tool_name:=permanent_magnet

# 无 GUI 模式
roslaunch zlab_arm_bringup single_arm_gazebo.launch arm_id:=arm1 gazebo_gui:=false
```

#### 双臂仿真
```bash
# 启动双臂 Gazebo 仿真
roslaunch zlab_arm_bringup dual_arms_gazebo.launch

# 指定工具
roslaunch zlab_arm_bringup dual_arms_gazebo.launch \
    arm1_tool:=permanent_magnet \
    arm2_tool:=electronic_magnet

# 指定机械臂位置
roslaunch zlab_arm_bringup dual_arms_gazebo.launch \
    arm1_xyz:="0 0 0" \
    arm1_rpy:="0 0 0" \
    arm2_xyz:="0.5 0 0" \
    arm2_rpy:="0 0 0"
```

## 参数说明

### 通用参数

- `arm_id`: 机械臂 ID（arm1 或 arm2），默认 arm1
- `tool_name`: 工具名称（如 permanent_magnet, electronic_magnet 等），默认 none
- `pipeline`: MoveIt 规划管道，默认 ompl
- `use_rviz`: 是否启动 RViz，默认 true

### 硬件启动参数

- `robot_ip`: 机械臂 IP 地址（单臂）
- `arm1_ip`: arm1 的 IP 地址，默认 192.168.31.202
- `arm2_ip`: arm2 的 IP 地址，默认 192.168.31.203
- `loop_hz`: 控制循环频率，默认 60.0 Hz

### Gazebo 仿真参数

- `gazebo_gui`: 是否启动 Gazebo GUI，默认 true
- `paused`: 是否暂停启动，默认 false
- `world_name`: Gazebo 世界文件，默认 worlds/empty.world
- `world_pose`: 机器人初始位置，默认 "-x 0 -y 0 -z 0 -R 0 -P 0 -Y 0"
- `arm1_xyz`: arm1 位置（x y z），默认 "0 0 0"
- `arm1_rpy`: arm1 姿态（roll pitch yaw），默认 "0 0 0"
- `arm2_xyz`: arm2 位置（x y z），默认空（使用默认值）
- `arm2_rpy`: arm2 姿态（roll pitch yaw），默认空（使用默认值）

## 依赖

- `zlab_arm_hardware`: 硬件接口包
- `zlab_arm_single_moveit_config`: 单臂 MoveIt 配置
- `zlab_arm_dual_moveit_config`: 双臂 MoveIt 配置
- `gazebo_ros`: Gazebo ROS 接口

