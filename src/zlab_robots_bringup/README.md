# ZLab Robots Bringup

这个包提供了启动 ZLab 机械臂集群（Diana7, FR5, 三臂协作）硬件和 MoveIt 的统一启动文件。

## 启动文件组织结构

- `launch/diana7/`: Diana7 机械臂相关启动项
- `launch/fr5/`: FR5 机械臂相关启动项（单臂/双臂）
- `launch/triple_arm/`: 三臂协作系统启动项

## 快速开始

### 1. Diana7 机械臂

#### 硬件启动
```bash
roslaunch zlab_robots_bringup diana7_bringup.launch robot_ip:=192.168.31.200
```

#### 仿真/MoveIt 预览 (Fake Execution)
```bash
roslaunch zlab_robots_bringup diana7_moveit.launch
```

### 2. FR5 机械臂

#### 单臂硬件启动
```bash
# 启动 arm1
roslaunch zlab_robots_bringup fr5_single_bringup.launch arm_id:=arm1
```

#### 单臂 MoveIt 预览
```bash
roslaunch zlab_robots_bringup fr5_single_moveit.launch arm_id:=arm1
```

#### 双臂硬件启动
```bash
roslaunch zlab_robots_bringup fr5_dual_bringup.launch
```

#### 双臂 MoveIt 预览
```bash
roslaunch zlab_robots_bringup fr5_dual_moveit.launch
```

#### Gazebo 仿真
```bash
# 单臂
roslaunch zlab_robots_bringup fr5_single_gazebo.launch
# 双臂
roslaunch zlab_robots_bringup fr5_dual_gazebo.launch
```

### 3. 三臂协作系统 (Diana7 + 2*FR5)

#### 硬件启动
```bash
roslaunch zlab_robots_bringup triple_arm_bringup.launch
```

#### MoveIt 预览
```bash
roslaunch zlab_robots_bringup triple_arm_moveit.launch
```

## 参数说明

- `arm_id`: 机械臂 ID（arm1 或 arm2），默认 arm1
- `tool_name`: 工具名称（如 permanent_magnet, electronic_magnet 等），默认 none
- `pipeline`: MoveIt 规划管道，默认 ompl
- `use_rviz`: 是否启动 RViz，默认 true

### 环境加载参数

- `load_environment`: 是否加载环境障碍物（墙壁等），默认 true
- `environment_config`: 环境配置文件路径，默认 `$(find zlab_robots_bringup)/config/default_environment.yaml`

### 硬件启动参数

- `robot_ip`: 机械臂 IP 地址（单臂）
- `arm1_ip`: arm1 的 IP 地址，默认 192.168.31.201
- `arm2_ip`: arm2 的 IP 地址，默认 192.168.31.202
- `loop_hz`: 控制循环频率，默认 30.0 Hz (FR5) / 50.0 Hz (Diana7)

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

