# ZLab 机器人控制系统

## 项目描述

基于 ROS Noetic 的 ZLab 机器人控制系统，支持 FR5（单臂和双臂）以及 Diana7 机械臂的控制，集成了 MoveIt 运动规划功能。

### 机械臂信息

- **FR5 Arm1 (Ram)**: 6自由度机械臂，负载5kg
- **FR5 Arm2 (Rem)**: 6自由度机械臂，负载5kg
- **Diana7**: 7自由度机械臂，负载7kg
- **末端执行器最大负载**: 5kg (FR5)

### 系统要求

- **操作系统**: Ubuntu 20.04 LTS
- **ROS 版本**: ROS Noetic
- **依赖库**: DianaApi (仅 Diana7 硬件接口需要)

## 项目结构

项目包含以下主要包：

### 1. 启动包 (Bringup)
- **fr5_bringup**: FR5 机械臂的启动文件包，提供单臂/双臂硬件及仿真启动脚本。

### 2. 描述包 (Descriptions)
- **fr5_description**: FR5 机器人描述文件（URDF/Xacro）。
- **diana7_description**: Diana7 机器人描述文件。

### 3. 硬件接口包 (Hardware)
- **fr5_hardware**: FR5 硬件接口，支持 TCP 通信协议。
- **diana7_hardware**: Diana7 硬件接口，使用 **DianaApi** 进行通信。

### 4. MoveIt 配置包 (MoveIt Config)
- **fr5_single_moveit_config**: FR5 单臂 MoveIt 配置。
- **fr5_dual_moveit_config**: FR5 双臂 MoveIt 配置。
- **diana7_moveit_config**: Diana7 MoveIt 配置。

## 特别说明：DianaApi

`diana7_hardware` 包依赖于 **DianaApi** 库。
- 该库默认应安装在 `/opt/DianaApi` 目录下。
- **如果您的系统中缺少该文件，请联系管理员获取安装包。**
- 编译前请确保 `/opt/DianaApi/include` 和 `/opt/DianaApi/lib` 路径存在并添加进系统路径。

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
echo "source ~/zlab_robots/devel/setup.bash" >> ~/.bashrc
```

## 使用方法

### FR5 真实硬件启动

#### 单臂启动

```bash
# 启动 arm1（默认 IP: 192.168.31.202）
roslaunch fr5_bringup single_arm_bringup.launch arm_id:=arm1

# 启动 arm2（默认 IP: 192.168.31.203）
roslaunch fr5_bringup single_arm_bringup.launch arm_id:=arm2
```

#### 双臂启动

```bash
# 使用默认 IP 地址
roslaunch fr5_bringup dual_arms_bringup.launch
```

### Diana7 真实硬件启动

```bash
# 启动 Diana7 硬件接口
roslaunch diana7_hardware diana7_hardware.launch

# 启动 Diana7 MoveIt 规划
roslaunch diana7_moveit_config demo.launch
```

### Gazebo 仿真启动 (FR5)

#### 单臂仿真

```bash
# 启动单臂 Gazebo 仿真
roslaunch fr5_bringup single_arm_gazebo.launch arm_id:=arm1
```

#### 双臂仿真

```bash
# 启动双臂 Gazebo 仿真
roslaunch fr5_bringup dual_arms_gazebo.launch
```

### 仅启动硬件接口（不使用 MoveIt）

```bash
# FR5 单臂
roslaunch fr5_hardware arm_hardware.launch arm_id:=arm1

# FR5 双臂
roslaunch fr5_hardware dual_arms_hardware.launch

# Diana7
roslaunch diana7_hardware diana7_hardware.launch
```

## 参数说明

### 通用参数 (FR5)

- `arm_id`: 机械臂 ID（arm1 或 arm2），默认 arm1
- `tool_name`: 工具名称，默认 none
- `pipeline`: MoveIt 规划管道，默认 ompl
- `use_rviz`: 是否启动 RViz，默认 true

### 硬件启动参数 (FR5)

- `robot_ip`: 机械臂 IP 地址（单臂启动时使用）
- `arm1_ip`: arm1 的 IP 地址，默认 192.168.31.202
- `arm2_ip`: arm2 的 IP 地址，默认 192.168.31.203
- `loop_hz`: 控制循环频率，默认 60.0 Hz

## 常见问题

### 1. 缺少 DianaApi
如果编译 `diana7_hardware` 失败，提示找不到 `DianaAPI.h`，请检查 `/opt/DianaApi` 是否存在。如果不存在，请向管理员索取。

### 2. 机械臂运动卡顿
- 检查网络延迟。
- 尝试降低 `loop_hz`。

## 许可证

TODO

## 贡献

欢迎提交 Issue 和 Pull Request。

## 联系方式

如有问题或建议，请通过 GitHub Issues 联系。
