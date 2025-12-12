# ZLab 机械臂控制系统

## 项目描述

基于 ROS Noetic 的 ZLab 机械臂控制系统，支持单臂和双臂控制，集成了 MoveIt 运动规划功能。

### 机械臂信息

- **Arm1 (Ram)**: IP 地址 192.168.31.202
- **Arm2 (Rem)**: IP 地址 192.168.31.203
- **末端执行器最大负载**: 5kg

### 系统要求

- **操作系统**: Ubuntu 20.04 LTS
- **ROS 版本**: ROS Noetic

## 项目结构

项目包含以下主要包：

- **zlab_arm_bringup**: 启动文件包，提供便捷的启动脚本
  - 单臂/双臂硬件启动
  - Gazebo 仿真启动
  
- **zlab_arm_hardware**: 硬件接口包
  - 基于 `ros_control` 的硬件接口
  - TCP 通信协议（端口 8080）
  - 支持 ServoJ 关节空间控制指令
  
- **zlab_arm_description**: 机器人描述文件
  - URDF 模型
  - 支持多种工具配置
  
- **zlab_arm_single_moveit_config**: 单臂 MoveIt 配置
  - 运动规划配置
  - 碰撞检测配置
  
- **zlab_arm_dual_moveit_config**: 双臂 MoveIt 配置
  - 双臂协调运动规划
  - 双臂碰撞检测

## 安装

### 1. 克隆仓库

```bash
git clone https://github.com/luojiahao-cn/zlab_robots.git
cd zlab_robots
```

### 2. 安装依赖

在 `src` 目录下运行：

```bash
cd src
rosdep install --from-paths . --ignore-src -r -y
```

安装额外的 ROS 包：

```bash
sudo apt-get install ros-noetic-moveit
sudo apt-get install ros-noetic-trac-ik
sudo apt-get install ros-noetic-ros-controllers
sudo apt-get install ros-noetic-gazebo-ros-control
sudo apt-get install ros-noetic-joint-trajectory-controller
```

### 3. 编译工作空间

```bash
cd ..
catkin_make
```

### 4. 配置环境

```bash
source devel/setup.bash
```

建议将以下命令添加到 `~/.bashrc`：

```bash
echo "source ~/workshop/zlab_robots/devel/setup.bash" >> ~/.bashrc
```

## 使用方法

### 真实硬件启动

#### 单臂启动

```bash
# 启动 arm1（默认 IP: 192.168.31.202）
roslaunch zlab_arm_bringup single_arm_bringup.launch arm_id:=arm1

# 启动 arm2（默认 IP: 192.168.31.203）
roslaunch zlab_arm_bringup single_arm_bringup.launch arm_id:=arm2

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

# 无 GUI 模式（用于无头服务器）
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

### 仅启动硬件接口（不使用 MoveIt）

如果需要单独启动硬件接口，可以使用：

```bash
# 单臂
roslaunch zlab_arm_hardware arm_hardware.launch arm_id:=arm1

# 双臂
roslaunch zlab_arm_hardware dual_arms_hardware.launch
```

## 参数说明

### 通用参数

- `arm_id`: 机械臂 ID（arm1 或 arm2），默认 arm1
- `tool_name`: 工具名称（如 permanent_magnet, electronic_magnet 等），默认 none
- `pipeline`: MoveIt 规划管道，默认 ompl
- `use_rviz`: 是否启动 RViz，默认 true

### 硬件启动参数

- `robot_ip`: 机械臂 IP 地址（单臂启动时使用）
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

## 常见问题

### 机械臂运动卡顿

如果遇到机械臂运动卡顿，可以尝试以下方法：

1. **检查控制循环频率**：
   - 降低 `loop_hz` 参数（例如从 60Hz 降到 50Hz）
   - 检查系统 CPU 使用率，关闭不必要的进程

2. **检查网络连接**：
   - 确保机械臂 IP 地址正确
   - 检查网络延迟和丢包率
   - 确保防火墙允许端口 8080 通信

3. **检查控制循环周期**：
   - 查看日志中的 "Control loop missed desired period" 警告
   - 如果频繁出现，考虑降低控制频率

### 连接问题

如果遇到连接错误：

- **验证 IP 地址**：确保机械臂 IP 地址配置正确
- **检查网络连接**：使用 `ping` 命令测试网络连通性
- **检查端口**：确保端口 8080 未被占用
- **防火墙设置**：确保防火墙允许 TCP 端口 8080 通信

### 运动规划失败

如果 MoveIt 运动规划失败：

- **检查工作空间**：确保目标位置在机械臂工作空间内
- **检查碰撞**：确保没有碰撞检测错误
- **调整规划参数**：尝试不同的规划算法（ompl, chomp 等）
- **检查关节限制**：确保目标位置在关节限制范围内

### 错误消息

常见错误消息及解决方案：

- **"Connection lost with robot"**: 检查网络连接和 IP 地址配置
- **"Failed to receive joint states"**: 验证机械臂状态和连接
- **"Control loop cycle time exceeded"**: 降低控制频率或优化系统资源
- **"Failed to get param 'robot_ip'"**: 检查 launch 文件参数配置

## 开发说明

### 包依赖关系

```
zlab_arm_bringup
├── zlab_arm_hardware
│   └── zlab_arm_description
├── zlab_arm_single_moveit_config
│   └── zlab_arm_description
└── zlab_arm_dual_moveit_config
    └── zlab_arm_description
```

### 协议格式

硬件接口使用 TCP 协议通信（端口 8080）：

**ServoJ 指令（376）**：
```
/f/bIII123III376III{length}III{ServoJ(j1,j2,j3,j4,j5,j6,acc,vel,cmdT,filterT,gain)}III/b/f
```

**GetActualJointPosRadian 指令（375）**：
```
/f/bIII123III375III25IIIGetActualJointPosRadian()III/b/f
```

## 许可证

TODO

## 贡献

欢迎提交 Issue 和 Pull Request。

## 联系方式

如有问题或建议，请通过 GitHub Issues 联系。
