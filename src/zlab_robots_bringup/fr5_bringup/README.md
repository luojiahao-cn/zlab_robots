# ZLab Arm Bringup

这个包提供了启动 ZLab 机械臂硬件和 MoveIt 的便捷启动文件。

## 启动文件

### 真实硬件启动

#### 单臂启动
```bash
# 启动 arm1（默认 IP: 192.168.31.201）
roslaunch fr5_bringup single_arm_bringup.launch arm_id:=arm1

# 指定 IP 地址
roslaunch fr5_bringup single_arm_bringup.launch arm_id:=arm1 robot_ip:=192.168.31.201

# 指定工具
roslaunch fr5_bringup single_arm_bringup.launch arm_id:=arm1 tool_name:=permanent_magnet

# 不启动 RViz
roslaunch fr5_bringup single_arm_bringup.launch arm_id:=arm1 use_rviz:=false

# 不加载环境（用于测试）
roslaunch fr5_bringup single_arm_bringup.launch arm_id:=arm1 load_environment:=false

# 使用自定义环境配置
roslaunch fr5_bringup single_arm_bringup.launch arm_id:=arm1 \
    environment_config:=$(rospack find fr5_bringup)/config/default_environment.yaml
```

#### 双臂启动
```bash
# 使用默认 IP 地址
roslaunch fr5_bringup dual_arms_bringup.launch

# 指定 IP 地址
roslaunch fr5_bringup dual_arms_bringup.launch \
    arm1_ip:=192.168.31.201 \
    arm2_ip:=192.168.31.202

# 指定工具
roslaunch fr5_bringup dual_arms_bringup.launch \
    arm1_tool:=permanent_magnet \
    arm2_tool:=electronic_magnet
```

### Gazebo 仿真启动

#### 单臂仿真
```bash
# 启动单臂 Gazebo 仿真
roslaunch fr5_bringup single_arm_gazebo.launch arm_id:=arm1

# 指定工具
roslaunch fr5_bringup single_arm_gazebo.launch arm_id:=arm1 tool_name:=permanent_magnet

# 无 GUI 模式
roslaunch fr5_bringup single_arm_gazebo.launch arm_id:=arm1 gazebo_gui:=false
```

#### 双臂仿真
```bash
# 启动双臂 Gazebo 仿真
roslaunch fr5_bringup dual_arms_gazebo.launch

# 指定工具
roslaunch fr5_bringup dual_arms_gazebo.launch \
    arm1_tool:=permanent_magnet \
    arm2_tool:=electronic_magnet

# 指定机械臂位置
roslaunch fr5_bringup dual_arms_gazebo.launch \
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

### 环境加载参数

- `load_environment`: 是否加载环境障碍物（墙壁等），默认 true
- `environment_config`: 环境配置文件路径，默认 `$(find fr5_bringup)/config/default_environment.yaml`

### 硬件启动参数

- `robot_ip`: 机械臂 IP 地址（单臂）
- `arm1_ip`: arm1 的 IP 地址，默认 192.168.31.201
- `arm2_ip`: arm2 的 IP 地址，默认 192.168.31.202
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

## 环境加载功能

环境加载功能用于防止机械臂碰撞周边的墙壁和障碍物。环境障碍物会被添加到 MoveIt 的 planning scene 中，规划器会自动避开这些区域。

### 环境配置文件

环境配置文件位于 `config/default_environment.yaml`，支持以下类型的障碍物：

- **box**: 长方体（需要 3 个维度：长、宽、高）
- **cylinder**: 圆柱体（需要 2 个维度：高度、半径）
- **sphere**: 球体（需要 1 个维度：半径）

### 配置示例

```yaml
collision_objects:
  # 后墙
  - id: back_wall
    type: box
    frame_id: "{arm_id}_base_link"  # 会自动替换为 arm1_base_link 或 arm2_base_link
    position: [0.0, -0.5, 0.5]     # x, y, z (米)
    orientation: [0, 0, 0, 1]      # quaternion (x, y, z, w)
    dimensions: [2.0, 0.1, 2.0]     # 长、宽、高 (米)
```

### 使用说明

1. **默认环境**：启动时会自动加载默认环境配置
2. **自定义环境**：可以通过 `environment_config` 参数指定自定义配置文件
3. **禁用环境**：设置 `load_environment:=false` 可以禁用环境加载（用于测试）

### 在 RViz 中查看

环境障碍物会在 RViz 的 Planning Scene 中显示为半透明的几何体，可以通过 MoveIt Motion Planning 插件查看。

## 依赖

- `fr5_hardware`: 硬件接口包
- `fr5_single_moveit_config`: 单臂 MoveIt 配置
- `fr5_dual_moveit_config`: 双臂 MoveIt 配置
- `gazebo_ros`: Gazebo ROS 接口
- `moveit_ros_planning_interface`: MoveIt 规划接口
- `yaml-cpp`: YAML 配置文件解析库

