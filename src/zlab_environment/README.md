# zlab_environment

## 概述

`zlab_environment` 是一个用于自动将环境障碍物（碰撞物体）加载到 MoveIt! 规划场景中的 ROS 包。它允许用户通过 YAML 配置文件定义各种几何形状（如盒子、圆柱体和球体），并支持动态坐标系替换和自动路径解析。

## 主要功能

*   **YAML 配置驱动**：通过易于读写的 YAML 文件定义复杂的环境障碍物。
*   **支持多种几何形状**：
    *   `box` (盒子)：需指定 dimensions [x, y, z]。
    *   `cylinder` (圆柱体)：需指定 dimensions [height, radius]。
    *   `sphere` (球体)：需指定 dimensions [radius]。
*   **动态坐标系替换**：
    *   支持 `{arm_id}` 占位符替换，便于在同一套配置中切换不同手臂。
    *   自动处理以 `arm1_` 或 `arm2_` 开头的 `frame_id` 前缀。
*   **路径解析**：配置文件路径和内部引用的路径支持 `$(find package)` 和 `$(env VAR)` 宏扩展。
*   **可视化支持**：支持为每个碰撞物体设置自定义颜色（RGBA）。
*   **双臂支持**：配置文件中可以区分 `collision_objects` 和 `dual_arm_collision_objects`。

## 文件结构

*   `config/`: 预定义的障碍物配置文件。
    *   `arm1_environment.yaml`: 针对 arm1 的环境配置。
    *   `arm2_environment.yaml`: 针对 arm2 的环境配置。
    *   `scanning_table.yaml`: 包含扫描桌和地面的通用环境配置。
*   `launch/`: 启动文件。
    *   `load_environment.launch`: 用于加载环境的主启动文件。
*   `src/`: 核心实现。
    *   `environment_loader.cpp`: 处理 YAML 解析、坐标系替换及 MoveIt 接口通信。

## 使用方法

### 运行环境加载器

可以通过 launch 文件启动节点并指定配置文件：

```bash
roslaunch zlab_environment load_environment.launch environment_config:=$(find zlab_environment)/config/scanning_table.yaml
```

### 主要参数

*   `environment_config`: 配置文件路径。
*   `arm_id`: 用于 frame_id 替换的手臂标识符（默认为 `arm2`）。
*   `load_environment`: 是否加载（布尔值，默认为 `true`）。

## YAML 配置示例

```yaml
collision_objects:
  # 示例盒子
  - id: sample_box
    type: box
    frame_id: world
    position: [0.5, 0.0, 0.25]
    orientation: [0, 0, 0, 1]
    dimensions: [0.1, 0.1, 0.5]
    color: [0.8, 0.1, 0.1, 0.5] # R, G, B, A

  # 示例圆柱体 (利用 arm_id 替换)
  - id: sample_cylinder
    type: cylinder
    frame_id: "{arm_id}_base_link"
    position: [0.0, 0.0, 0.2]
    orientation: [0, 0, 0, 1]
    dimensions: [0.4, 0.05] # [height, radius]
    color: [0.1, 0.8, 0.1, 0.6]
```

## 注意事项

*   确保在运行此节点前，相关的 Moveit! 控制节点（如 `move_group`）已经启动，否则可能无法成功更新规划场景。
*   程序会在启动后尝试连接 Planning Scene 接口，并等待 1 秒以确保连接建立后再添加物体。
