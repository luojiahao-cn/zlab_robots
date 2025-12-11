# 电磁铁工具

电磁铁末端执行器工具。

## 工具信息

- **名称**: electronic_magnet
- **显示名称**: 电磁铁
- **类型**: urdf_package（使用完整URDF包，从SolidWorks导出）
- **版本**: 1.0.0

## 包结构说明

此工具现在是 `zlab_arm_description` 包的组成部分，不是独立的ROS包。

## 使用方法

### 在MoveIt中使用

```bash
roslaunch zlab_arm_description display_arm_rviz.launch tool_name:=electronic_magnet
```

### 在Gazebo中使用

```bash
roslaunch zlab_arm_description display_dual_arm_rviz.launch arm1_tool:=electronic_magnet arm2_tool:=electronic_magnet
```

## 配置说明

工具使用 `urdf_package` 类型，配置在 `config/tool.yaml` 中：

- **urdf_package.urdf_file**: URDF文件路径
- **urdf_package.root_link_name**: 工具根链接名称
- **urdf_package.tcp_link_name**: TCP链接名称
- **mount_offset**: 工具相对于法兰的安装偏移
- **physical_properties**: 工具的物理属性（质量、惯性）

## 注意事项

1. URDF文件中的mesh路径已更新为正确的package路径
2. 工具质量和惯性要根据实际测量设置
3. 如果需要在安装时微调位置，可以修改 `mount_offset` 配置
