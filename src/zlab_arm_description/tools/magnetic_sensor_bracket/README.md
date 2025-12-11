# 磁传感器支架工具包

磁传感器支架末端执行器工具包。

## 工具信息

- **名称**: magnetic_sensor_bracket
- **显示名称**: 磁传感器支架
- **类型**: urdf_package（使用完整URDF包，从SolidWorks导出）
- **版本**: 1.0.0

## 目录结构

```
magnetic_sensor_bracket/
├── README.md              # 工具说明文档
├── config/
│   ├── tool.yaml         # 工具配置文件（必需）
│   └── joint_names_bracket.yaml  # 关节名称配置
├── urdf/                 # URDF文件目录
│   ├── bracket.urdf      # 工具URDF文件
│   └── bracket.csv       # 工具数据文件
├── meshes/               # 模型文件目录
│   ├── bracket_base_link.STL
│   └── bracket_tcp_link.STL
├── textures/             # 纹理文件目录（如果有）
├── launch/               # 启动文件目录
│   ├── display.launch
│   └── gazebo.launch
├── package.xml           # ROS包定义
└── CMakeLists.txt        # CMake构建文件
```

## 使用方法

### 在MoveIt中使用

```bash
roslaunch fr5v6_single_moveit_config demo.launch tool_name:=magnetic_sensor_bracket
```

### 在Gazebo中使用

```bash
roslaunch fr5v6_single_moveit_config demo_gazebo_with_tool.launch tool_name:=magnetic_sensor_bracket
```

### 单独显示工具

```bash
roslaunch zlab_arm_description display_arm_rviz.launch tool_name:=magnetic_sensor_bracket
```

### 在Gazebo中显示工具

工具现在集成在zlab_arm_description包中，不再有独立的launch文件。如需在Gazebo中测试，请使用主包的相应功能。

## 配置说明

工具使用 `urdf_package` 类型，配置在 `config/tool.yaml` 中：

- **urdf_package.urdf_file**: URDF文件路径
- **urdf_package.root_link_name**: 工具根链接名称（`bracket_base_link`）
- **urdf_package.tcp_link_name**: TCP链接名称（`bracket_tcp_link`）
- **mount_offset**: 工具相对于法兰的安装偏移
- **physical_properties**: 工具的物理属性（质量、惯性）

## URDF结构

工具包含以下链接：
- **bracket_base_link**: 工具根链接，连接到机械臂的ee_link
- **bracket_tcp_link**: TCP链接，工具中心点

## 注意事项

1. URDF文件中的mesh路径已更新为正确的package路径
2. 工具质量约为 0.058 kg
3. TCP链接已在URDF中定义，无需额外配置
4. 如果需要在安装时微调位置，可以修改 `mount_offset` 配置

