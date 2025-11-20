# 工具包系统使用指南

## 概述

工具包系统允许您轻松管理和切换不同的末端执行器工具。每个工具都是一个独立的包，包含所有必要的配置和资源文件。

## 目录结构

```
frcobot_tools/
└── tools/
    ├── tool_none/          # 无工具配置
    ├── tool_250mm/         # 250mm工具示例
    ├── tool_template/      # 工具包模板
    └── your_new_tool/      # 您的新工具
        ├── README.md
        ├── config/
        │   └── tool.yaml   # 工具配置文件（必需）
        ├── urdf/           # URDF文件（如果使用urdf_package类型）
        └── meshes/         # 模型文件（如果使用simple类型）
```

## 创建新工具

### 步骤1: 复制模板

```bash
cd ~/zlab_robots/catkin_ws/src/frcobot_tools/tools
cp -r tool_template your_new_tool
```

### 步骤2: 编辑配置文件

编辑 `your_new_tool/config/tool.yaml`：

```yaml
tool_info:
  name: "your_new_tool"
  display_name: "我的新工具"
  description: "工具描述"
  version: "1.0.0"
  author: "您的名字"

tool_type: "simple"  # 或 "urdf_package"

origin:
  xyz: [0, 0, 0]
  rpy: [0, 0, 0]

tcp:
  xyz: [0, 0, 0.1]  # TCP位置
  rpy: [0, 0, 0]

physical_properties:
  mass: 0.2
  inertia:
    ixx: 0.001
    ixy: 0.0
    ixz: 0.0
    iyy: 0.001
    iyz: 0.0
    izz: 0.001

visual:
  mesh_path: "package://frcobot_description/meshes/fr5v6/visual/your_tool.DAE"
  scale: [0.001, 0.001, 0.001]

collision:
  mesh_path: "package://frcobot_description/meshes/fr5v6/collision/your_tool.STL"
  scale: [0.001, 0.001, 0.001]
```

### 步骤3: 添加资源文件

- **Simple类型**: 添加mesh文件到 `meshes/visual/` 和 `meshes/collision/`
- **URDF Package类型**: 添加URDF文件到 `urdf/` 目录

### 步骤4: 编译和测试

```bash
cd ~/zlab_robots/catkin_ws
catkin_make
source devel/setup.bash

# 测试工具
roslaunch fr5v6_single_moveit_config demo.launch tool_name:=your_new_tool
```

## 使用工具

### 在启动文件中指定工具

```xml
<launch>
  <arg name="tool_name" default="tool_250mm"/>
  <!-- ... -->
</launch>
```

### 在命令行中指定工具

```bash
roslaunch fr5v6_single_moveit_config demo.launch tool_name:=tool_250mm
```

### 列出所有可用工具

```bash
rosrun frcobot_tools list_tools.py
```

## 工具类型说明

### Simple类型

适用于简单的工具，只需要mesh文件和基本配置。

**配置要求:**
- `visual.mesh_path`: 可视化模型路径
- `collision.mesh_path`: 碰撞模型路径
- `physical_properties`: 物理属性（质量、惯性）

### URDF Package类型

适用于复杂的工具，特别是从SolidWorks导出的工具。

**配置要求:**
- `urdf_package.urdf_file`: URDF文件路径
- `urdf_package.root_link_name`: 工具根链接名称
- `urdf_package.tcp_link_name`: TCP链接名称（可选）

## 从SolidWorks导入工具

1. 从SolidWorks导出URDF文件
2. 创建新工具目录
3. 将URDF文件放到 `urdf/` 目录
4. 配置 `tool.yaml`:
   ```yaml
   tool_type: "urdf_package"
   urdf_package:
     urdf_file: "urdf/tool.urdf.xacro"
     root_link_name: "tool_base_link"  # 根据您的URDF修改
     tcp_link_name: ""  # 如果URDF中有TCP链接，填写链接名
   ```

## 注意事项

1. 工具名称必须与目录名一致
2. `config/tool.yaml` 文件是必需的
3. 确保所有路径使用 `package://` 格式
4. 工具包路径: `package://frcobot_tools/tools/tool_name/`
5. 如果工具包不存在，系统会尝试从旧的 `tools.yaml` 加载配置

## 故障排除

### 工具包未找到

如果xacro报错找不到工具包配置文件，可以：
1. 确保工具包目录存在且名称正确
2. 确保 `config/tool.yaml` 文件存在
3. 重新编译: `catkin_make`

### 工具配置错误

检查 `tool.yaml` 文件格式是否正确，特别是：
- YAML语法正确
- 所有必需字段都存在
- 路径格式正确（使用 `package://`）

### 向后兼容

如果工具包不存在，系统会自动回退到旧的 `tools.yaml` 配置。确保旧的 `tools.yaml` 中有对应的工具配置。

