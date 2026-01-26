# ZLab 机器人控制系统

ZLab 实验室的 ROS Noetic 多机器人协作控制系统，支持 FR5、Diana7 机械臂以及三臂协作系统，集成了 MoveIt 运动规划、Gazebo 仿真和多种末端工具。

## 项目概述

ZLab 机器人控制系统是 ZLab 实验室专为多机器人协作任务开发的完整 ROS 机器人控制框架。系统支持多种机械臂配置，包括单臂、双臂和三臂协作模式，提供从仿真到实机部署的完整解决方案。

### 支持的机器人配置

| 机器人 | 自由度 | 负载能力 | 主要应用场景 |
|--------|--------|----------|--------------|
| **FR5** | 6 DOF | 5kg | 精密操作、拾放任务 |
| **Diana7** | 7 DOF | 7kg | 复杂姿态、高精度定位 |
| **双臂 FR5** | 12 DOF | 5kg×2 | 对称操作、协作任务 |
| **三臂系统** | 19 DOF | 混合负载 | 多机器人协作、复杂装配 |

### 核心特性

- **多机器人协作**：支持单臂、双臂、三臂等多种配置
- **统一接口**：标准化 ROS 接口，便于集成和扩展
- **工具生态**：丰富的末端工具库，支持快速切换
- **标定系统**：完整的多臂系统标定工具链
- **仿真支持**：Gazebo 物理仿真环境
- **MoveIt集成**：先进的运动规划和控制

## 项目架构

```
zlab_robots/
├── zlab_environment/           # 环境配置和障碍物管理
├── zlab_robots_bringup/        # 统一启动包
├── zlab_robots_calibration/    # 多臂系统标定工具
├── zlab_robots_descriptions/   # 机器人描述文件
│   ├── fr5_description/        # FR5 机器人模型
│   ├── diana7_description/     # Diana7 机器人模型
│   ├── multi_arm_description/  # 多臂系统集成
│   └── tools_description/      # 末端工具库
├── zlab_robots_hardware/       # 硬件接口层
│   ├── fr5_hardware/          # FR5 硬件驱动
│   └── diana7_hardware/       # Diana7 硬件驱动
└── zlab_robots_moveit_config/  # MoveIt 运动规划配置
    ├── fr5_single_moveit_config/    # 单臂 FR5
    ├── fr5_dual_moveit_config/      # 双臂 FR5
    ├── diana7_moveit_config/        # Diana7
    └── triple_arm_moveit_config/    # 三臂协作系统
```

## 系统要求

- **操作系统**: Ubuntu 20.04 LTS
- **ROS 版本**: ROS Noetic Ninjemys
- **依赖库**:
  - MoveIt 1.1.x
  - Gazebo 11.x
  - DianaApi (仅 Diana7 硬件模式需要)

## 快速开始

### 1. 环境配置

```bash
# 克隆 ZLab 机器人控制系统项目
git clone https://github.com/luojiahao-cn/zlab_robots.git
cd zlab_robots

# 安装 ROS 依赖
cd src
rosdep install --from-paths . --ignore-src -r -y

# 安装额外依赖
sudo apt-get install ros-noetic-moveit ros-noetic-trac-ik \
    ros-noetic-ros-controllers ros-noetic-gazebo-ros-control \
    ros-noetic-joint-trajectory-controller

# 编译工作空间
cd ..
catkin_make

# 配置环境
source devel/setup.bash
echo "source ~/zlab_robots/devel/setup.bash" >> ~/.bashrc
```

### 2. 启动测试

```bash
# 测试单臂 FR5 仿真
roslaunch zlab_robots_bringup fr5_single_moveit.launch arm_id:=arm1

# 测试双臂 FR5 仿真
roslaunch zlab_robots_bringup fr5_dual_moveit.launch

# 测试三臂协作系统仿真
roslaunch zlab_robots_bringup triple_arm_moveit.launch
```

## 使用指南

### FR5 系列启动

#### 单臂模式

```bash
# 仿真模式
roslaunch zlab_robots_bringup fr5_single_moveit.launch arm_id:=arm1

# 带工具的仿真模式
roslaunch zlab_robots_bringup fr5_single_moveit.launch \
    arm_id:=arm1 \
    tool_name:=electronic_magnet

# 硬件模式
roslaunch zlab_robots_bringup fr5_single_bringup.launch arm_id:=arm1 robot_ip:=192.168.31.201
```

#### 双臂模式

```bash
# 仿真模式（默认无工具）
roslaunch zlab_robots_bringup fr5_dual_moveit.launch

# 自定义工具配置
roslaunch zlab_robots_bringup fr5_dual_moveit.launch \
    arm1_tool:=electronic_magnet \
    arm2_tool:=permanent_magnet

# 硬件模式
roslaunch zlab_robots_bringup fr5_dual_bringup.launch
```

### Diana7 启动

```bash
# 仿真模式
roslaunch zlab_robots_bringup diana7_moveit.launch

# 带工具的仿真模式
roslaunch zlab_robots_bringup diana7_moveit.launch tool_name:=calibration_tool

# 硬件模式 (需要 DianaApi)
roslaunch zlab_robots_bringup diana7_bringup.launch robot_ip:=192.168.31.200
```

### 三臂协作系统

```bash
# 仿真模式（默认工具配置）
roslaunch zlab_robots_bringup triple_arm_moveit.launch

# 自定义工具配置
roslaunch zlab_robots_bringup triple_arm_moveit.launch \
    diana7_tool:=magnetometer_array \
    arm1_tool:=electronic_magnet \
    arm2_tool:=permanent_magnet

# 硬件模式
roslaunch zlab_robots_bringup triple_arm_bringup.launch
```

### Gazebo 仿真

```bash
# 单臂 Gazebo 仿真
roslaunch zlab_robots_bringup fr5_single_gazebo.launch arm_id:=arm1

# 双臂 Gazebo 仿真
roslaunch zlab_robots_bringup fr5_dual_gazebo.launch
```

## 配置参数

### 通用参数

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `tool_name` | "none" | 末端工具名称（单臂/Diana7） |
| `arm1_tool` | "none" | arm1 工具名称（多臂系统） |
| `arm2_tool` | "none" | arm2 工具名称（多臂系统） |
| `diana7_tool` | "none" | diana7 工具名称（三臂系统） |
| `pipeline` | "ompl" | 运动规划管道 |
| `use_rviz` | true | 是否启动 RViz |
| `use_gui` | false | 是否启动 MoveIt GUI |

### 硬件参数

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `robot_ip` | - | 机器人 IP 地址 |
| `arm1_ip` | 192.168.31.201 | FR5 Arm1 IP |
| `arm2_ip` | 192.168.31.202 | FR5 Arm2 IP |
| `loop_hz` | 30.0 (FR5) / 50.0 (Diana7) | 控制循环频率 |

### 位置参数（标定后调整）

```bash
# 命令行临时调整
roslaunch triple_arm_moveit_config demo.launch \
    diana7_xyz:="0 0 0" diana7_rpy:="0 0 0" \
    arm1_xyz:="1.5 -0.2 0" arm1_rpy:="0 0 0" \
    arm2_xyz:="1.5 0.4 0" arm2_rpy:="0 0 0"
```

## 末端工具系统

### 支持的工具

- `electronic_magnet` - 电磁铁
- `permanent_magnet` - 永磁铁
- `magnetic_sensor_bracket` - 磁传感器支架
- `magnetometer_array` - 磁力计阵列
- `calibration_tool` - 标定工具
- `calibrated_em` - 已标定电磁铁
- `calibrated_magmeterarray` - 已标定磁力计阵列

### 工具配置

工具通过 `tool_name` 参数加载，配置位于：
```
src/zlab_robots_descriptions/tools_description/tools/[tool_name]/config/tool.yaml
```

#### 多臂系统工具配置示例

```bash
# 三臂协作系统 - 不同机械臂使用不同工具
roslaunch zlab_robots_bringup triple_arm_moveit.launch \
    diana7_tool:=magnetometer_array \
    arm1_tool:=electronic_magnet \
    arm2_tool:=permanent_magnet

# 双臂 FR5 - 对称工具配置
roslaunch zlab_robots_bringup fr5_dual_moveit.launch \
    arm1_tool:=electronic_magnet \
    arm2_tool:=electronic_magnet
```

修改工具安装位姿：
```yaml
urdf_package:
  mount_offset:
    xyz: [0, 0, 0]
    rpy: [0, 0, 3.1415926]  # Roll, Pitch, Yaw (弧度)
```

## 标定系统

### 标定流程

1. **单臂标定**：使用标定工具确定各机械臂基座位置
2. **多臂标定**：建立机械臂之间的相对位置关系
3. **工具标定**：标定末端工具的安装位置和姿态

### 标定工具使用

```bash
# 启动标定节点
roslaunch zlab_robots_calibration calibration.launch

# 查看标定结果
ls src/zlab_robots_calibration/result/
```

## 命名规范

为确保系统的一致性和可扩展性，所有组件遵循统一命名规范：

### 关节和连杆命名
```
[arm_id]_joint_1, [arm_id]_joint_2, ..., [arm_id]_joint_N
[arm_id]_link_1, [arm_id]_link_2, ..., [arm_id]_link_N
```

### 机器人拓扑结构
```
world
└── [arm_id]_pedestal_link (固定底座)
    └── [arm_id]_base_link (机械臂基点)
        ├── [arm_id]_link_1
        │   ├── [arm_id]_link_2
        │   │   └── ...
        │   └── [arm_id]_ee_link (工具挂载点)
```

### 机械臂标识符
- `arm1`, `arm2` - FR5 机械臂
- `diana7` - Diana7 机械臂

## 重要说明

### DianaApi 依赖

Diana7 硬件接口依赖于专有的 **DianaApi** 库：
- 默认安装路径：`/opt/DianaApi`
- 如缺失此库，请联系系统管理员获取
- 编译前确保路径正确配置

### 网络配置

- FR5 默认 IP：192.168.31.201 (arm1), 192.168.31.202 (arm2)
- Diana7 默认 IP：192.168.31.200
- 确保网络连接稳定，延迟 < 10ms


## 更新日志

### v2.0.0 (2026-01-26)
- **架构重构**：统一包结构，优化依赖关系
- **多臂支持**：完善三臂协作系统
- **工具系统**：标准化末端工具接口
- **标定工具**：集成多臂系统标定功能
- **配置优化**：简化启动参数，提高易用性

### v1.5.0 (2025-12-25)
- **系统重构**：统一命名规范，简化 URDF 结构
- **工具加载**：改为静态 Xacro 加载机制
- **文件清理**：重命名和重组资源文件
- **修复**：标定节点和 RViz 配置中的硬编码问题

### v1.0.0 (2025-12-21)
- **初始版本**：支持 FR5 和 Diana7 基本功能
- **包结构**：建立模块化架构
- **启动脚本**：提供完整启动流程

## 许可证

本项目采用 MIT 许可证 - 查看 [LICENSE](LICENSE) 文件了解详情

## 贡献

欢迎 ZLab 实验室成员和其他开发者提交 Issue 和 Pull Request。


## 联系方式

如有问题或建议，请通过 GitHub Issues 联系 ZLab 团队。
