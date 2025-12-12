# ZLab Arm Hardware Interface

基于 `ros_control` 的精简硬件接口包，用于通过 TCP 通信控制 ZLab 机械臂。

## 特性

- 基于 `ros_control` 标准硬件接口
- TCP 通信协议（端口 8080）
- 支持 ServoJ 关节空间控制指令
- 支持多机械臂（通过命名空间隔离）
- 精简设计，易于维护和扩展

## 协议格式

### ServoJ 指令（376）
```
/f/bIII123III376III{length}III{ServoJ(j1,j2,j3,j4,j5,j6,acc,vel,cmdT,filterT,gain)}III/b/f
```

### GetActualJointPosRadian 指令（375）
```
/f/bIII123III375III25IIIGetActualJointPosRadian()III/b/f
```

## 使用方法

### 单机械臂

启动 arm1（默认 IP: 192.168.31.202）：
```bash
roslaunch zlab_arm_hardware arm_hardware.launch arm_id:=arm1
```

启动 arm2（默认 IP: 192.168.31.203）：
```bash
roslaunch zlab_arm_hardware arm_hardware.launch arm_id:=arm2
```

指定 IP 地址：
```bash
roslaunch zlab_arm_hardware arm_hardware.launch arm_id:=arm1 robot_ip:=192.168.31.202
```

指定工具：
```bash
roslaunch zlab_arm_hardware arm_hardware.launch arm_id:=arm1 tool_name:=permanent_magnet
```

### 双机械臂

```bash
roslaunch zlab_arm_hardware dual_arms_hardware.launch \
    arm1_ip:=192.168.31.202 \
    arm2_ip:=192.168.31.203 \
    arm1_tool:=permanent_magnet \
    arm2_tool:=electronic_magnet
```

## 配置参数

### 硬件接口参数
- `robot_ip`: 机械臂 IP 地址（必需）
- `loop_hz`: 控制循环频率，默认 60Hz
- `acc`: 加速度参数
- `vel`: 速度参数
- `cmd_t`: 指令周期，默认 0.002s
- `filter_t`: 滤波时间，默认 0.002s
- `gain`: 增益参数

### 控制器配置
配置文件位于 `config/` 目录：
- `arm1_controllers.yaml`: 单臂控制器配置
- `arm2_controllers.yaml`: 双臂配置中的 arm2 控制器

## 与 MoveIt 集成

启动硬件接口后，可以启动 MoveIt 进行运动规划：

```bash
# 单臂
roslaunch zlab_arm_single_moveit_config demo.launch \
    moveit_controller_manager:=ros_control

# 双臂
roslaunch zlab_arm_dual_moveit_config demo.launch \
    moveit_controller_manager:=ros_control
```

## 文件结构

```
zlab_arm_hardware/
├── include/zlab_arm_hardware/
│   └── zlab_arm_hw_interface.h    # 硬件接口头文件
├── src/
│   ├── zlab_arm_hw_interface.cpp  # 硬件接口实现
│   └── zlab_arm_hw_main.cpp       # 主程序和控制循环
├── config/
│   ├── arm1_controllers.yaml      # arm1 控制器配置
│   └── arm2_controllers.yaml      # arm2 控制器配置
├── launch/
│   ├── arm_hardware.launch        # 通用单臂启动文件（通过arm_id参数配置）
│   └── dual_arms_hardware.launch  # 双臂启动文件
└── README.md
```

## 依赖

- `hardware_interface`
- `controller_manager`
- `joint_limits_interface`
- `transmission_interface`
- `zlab_arm_description` (用于 URDF)

## 注意事项

1. 确保机械臂控制器已启动并监听 TCP 端口 8080
2. 确保网络连接正常，IP 地址配置正确
3. 控制循环频率建议设置为 60Hz，可根据实际情况调整
4. 运动参数（acc, vel 等）需要根据实际机械臂规格配置

