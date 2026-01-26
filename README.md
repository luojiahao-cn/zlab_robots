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
- **zlab_robots_bringup**: 统一的启动文件包，提供单臂/双臂/三臂硬件及仿真启动脚本。

### 2. 描述包 (Descriptions)
- **zlab_robots_descriptions**: 包含所有机器人的描述文件。
  - **fr5_description**: FR5 机器人描述文件（URDF/Xacro）。
  - **diana7_description**: Diana7 机器人描述文件。
  - **tools_description**: 各种末端工具描述文件。

### 3. 硬件接口包 (Hardware)
- **zlab_robots_hardware**: 包含所有机器人的硬件接口。
  - **fr5_hardware**: FR5 硬件接口，支持 TCP 通信协议。
  - **diana7_hardware**: Diana7 硬件接口，使用 **DianaApi** 进行通信。

### 4. MoveIt 配置包 (MoveIt Config)
- **zlab_robots_moveit_config**: 包含所有机器人的 MoveIt 配置。
  - **fr5_single_moveit_config**: FR5 单臂 MoveIt 配置。
  - **fr5_dual_moveit_config**: FR5 双臂 MoveIt 配置。
  - **diana7_moveit_config**: Diana7 MoveIt 配置。
  - **triple_arm_moveit_config**: 三臂协作系统 MoveIt 配置。

## 命名规范

为了保证系统的统一性和可扩展性，所有机械臂遵循以下命名规范：

- **关节命名**：`[arm_id]_joint_1` 到 `[arm_id]_joint_N`（例如 `arm1_joint_1`, `diana7_joint_1`）。
- **连杆命名**：`[arm_id]_link_1` 到 `[arm_id]_link_N`。
- **底座结构**：`world` -> `[arm_id]_pedestal_link` (固定底座) -> `[arm_id]_base_link` (机械臂基点)。
- **末端结构**：`[arm_id]_link_N` -> `[arm_id]_ee_link` (工具挂载点)。

## 末端工具加载

系统支持静态加载末端工具，通过 `tool_name` 参数指定。

- **工具库**：位于 `tools_description/urdf/zlab_tools.xacro`。
- **支持的工具**：`electronic_magnet`, `permanent_magnet`, `soft_finger`, `vacuum_gripper` 等。
- **加载方式**：在启动文件或 Xacro 中设置 `tool_name` 参数即可。

### 修改工具安装位姿

若需调整工具相对于机械臂末端的安装角度（例如绕 Z 轴旋转 180 度），请修改对应工具目录下的配置文件：

`src/zlab_robots_descriptions/tools_description/tools/[tool_name]/config/tool.yaml`

找到 `mount_offset` 参数并修改 `rpy` 值：

```yaml
urdf_package:
  # ...
  mount_offset:
    xyz: [0, 0, 0]
    rpy: [0, 0, 3.1415926] # 修改此处 (Roll, Pitch, Yaw)
```

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
# 启动 arm1（默认 IP: 192.168.31.201）
roslaunch zlab_robots_bringup fr5_single_bringup.launch arm_id:=arm1

# 启动 arm2（默认 IP: 192.168.31.202）
roslaunch zlab_robots_bringup fr5_single_bringup.launch arm_id:=arm2
```

#### 双臂启动

```bash
# 使用默认 IP 地址 (arm1: 201, arm2: 202)
roslaunch zlab_robots_bringup fr5_dual_bringup.launch
```

### Diana7 真实硬件启动

```bash
# 启动 Diana7 硬件及 MoveIt
roslaunch zlab_robots_bringup diana7_bringup.launch robot_ip:=192.168.31.200
```

### 三臂协作系统启动 (Diana7 + 2*FR5)

```bash
# 启动三臂硬件及 MoveIt
roslaunch zlab_robots_bringup triple_arm_bringup.launch
```

### Gazebo 仿真启动 (FR5)

#### 单臂仿真

```bash
# 启动单臂 Gazebo 仿真
roslaunch zlab_robots_bringup fr5_single_gazebo.launch arm_id:=arm1
```

#### 双臂仿真

```bash
# 启动双臂 Gazebo 仿真
roslaunch zlab_robots_bringup fr5_dual_gazebo.launch
```

### 仅启动硬件接口（不使用 MoveIt）

```bash
# FR5 单臂
roslaunch zlab_robots_hardware fr5_hardware.launch arm_id:=arm1

# FR5 双臂 (需要分别启动两个硬件节点)
roslaunch zlab_robots_hardware fr5_hardware.launch arm_id:=arm1
roslaunch zlab_robots_hardware fr5_hardware.launch arm_id:=arm2

# Diana7
roslaunch zlab_robots_hardware diana7_hardware.launch
```

## 参数说明

### 通用参数 (FR5)

- `arm_id`: 机械臂 ID（arm1 或 arm2），默认 arm1
- `tool_name`: 工具名称，默认 none
- `pipeline`: MoveIt 规划管道，默认 ompl
- `use_rviz`: 是否启动 RViz，默认 true

### 硬件启动参数 (FR5)

- `robot_ip`: 机械臂 IP 地址（单臂启动时使用）
- `arm1_ip`: arm1 的 IP 地址，默认 192.168.31.201
- `arm2_ip`: arm2 的 IP 地址，默认 192.168.31.202
- `loop_hz`: 控制循环频率，默认 30.0 Hz (FR5) / 50.0 Hz (Diana7)

## 标定后位姿修改

在完成多臂系统的标定后，可以通过修改启动参数来调整各机械臂在世界坐标系下的相对位姿，无需修改 URDF 文件。

### 方法一：命令行参数（临时调试）

在启动 `demo.launch` 时，直接通过命令行参数覆盖默认位姿：

```bash
roslaunch triple_arm_moveit_config demo.launch \
    diana7_xyz:="0 0 0" diana7_rpy:="0 0 0" \
    arm1_xyz:="1.5 -0.2 0" arm1_rpy:="0 0 0" \
    arm2_xyz:="1.5 0.4 0" arm2_rpy:="0 0 0"
```

### 方法二：修改 Launch 文件（永久生效）

若需永久保存标定结果，请编辑 `src/zlab_robots_moveit_config/triple_arm_moveit_config/launch/demo.launch` 文件，修改对应的 `<arg>` 默认值：

```xml
<!-- Arm position parameters (optional) -->
<arg name="diana7_xyz" default="0 0 0" doc="Position of diana7 (x y z)"/>
<arg name="diana7_rpy" default="0 0 0" doc="Orientation of diana7 (roll pitch yaw)"/>
<arg name="arm1_xyz" default="1.5 -0.2 0" doc="Position of arm1 (x y z)"/>
<arg name="arm1_rpy" default="0 0 0" doc="Orientation of arm1 (roll pitch yaw)"/>
<arg name="arm2_xyz" default="1.5 0.4 0" doc="Position of arm2 (x y z)"/>
<arg name="arm2_rpy" default="0 0 0" doc="Orientation of arm2 (roll pitch yaw)"/>
```

## 常见问题

### 1. 缺少 DianaApi
如果编译 `diana7_hardware` 失败，提示找不到 `DianaAPI.h`，请检查 `/opt/DianaApi` 是否存在。如果不存在，请向管理员索取。

### 2. 机械臂运动卡顿
- 检查网络延迟。
- 尝试降低 `loop_hz`。

## 更新日志

### 2025-12-25
1. **系统架构大重构**：
   - 统一了所有机械臂的命名规范（`[arm_id]_joint_N` 和 `[arm_id]_link_N`）。
   - 简化了 URDF 树结构，统一使用 `pedestal_link`、`base_link` 和 `ee_link`。
   - 整合了所有启动文件到 `zlab_robots_bringup` 包中。
   - 重新组织了 FR5 启动文件，分为 `single` 和 `dual` 子目录。
   - 更新了所有 MoveIt 配置以匹配新的命名规范。
2. **工具加载机制简化**：
   - 移除了动态工具加载逻辑（YAML/JSON），改为基于 `zlab_tools.xacro` 的静态加载。
   - 删除了全局的 `tool_config` 参数，统一通过 `tool_name` 进行工具切换。
3. **文件与资源清理**：
   - 将 `fr.xacro` 重命名为 `fr5.xacro`。
   - 将 `tools_library.xacro` 重命名为 `zlab_tools.xacro`。
   - 重命名了 FR5 的原始网格文件（Mesh），使其符合 `link_N` 命名规范。
   - 修复了标定节点（Calibration Nodes）和 RViz 配置中的硬编码链接名称。

### 2025-12-21
1. **Diana7 描述包重构**：
   - 将 `diana7_base` 包完全整合进 `diana7_description`，移除了独立的 `diana7_base` 包。
   - 修复了底座模型在 MoveIt/RViz 中显示位置错误（陷入地下）的问题。

2. **启动文件优化**：
   - `diana7_bringup` 中的启动文件（`diana7_robot.launch`, `diana7_moveit_bringup.launch`）现在支持 `tool_name` 参数，可正确加载末端工具。
   - 移除了 `diana7_moveit_config` 中冗余的 `diana7_moveit_planning_execution.launch`，统一使用 `diana7_bringup` 进行实机启动。

3. **MoveIt 配置修复**：
   - 修正了 `diana7.srdf` 中的 `virtual_joint` 定义，将参考系修正为 `world`。
   - 修正了 `moveit.rviz` 的固定帧设置。

## 许可证

TODO

## 贡献

欢迎提交 Issue 和 Pull Request。

## 联系方式

如有问题或建议，请通过 GitHub Issues 联系。
