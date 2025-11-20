# FRCobot 工具包管理系统

这是一个模块化的工具包管理系统，用于管理和切换不同的末端执行器工具。

## 目录结构

```
frcobot_tools/
├── README.md              # 本文件
├── package.xml            # ROS包定义
├── CMakeLists.txt         # 构建配置
├── tools/                 # 工具包目录
│   ├── tool_none/        # 无工具配置
│   ├── tool_250mm/       # 250mm工具示例
│   ├── tool_template/    # 工具包模板
│   └── ...               # 其他工具包
└── scripts/              # 工具管理脚本
    └── list_tools.py     # 列出所有可用工具
```

## 工具包结构

每个工具包必须包含以下结构：

```
tool_name/
├── README.md              # 工具说明（可选）
├── config/
│   └── tool.yaml         # 工具配置文件（必需）
├── urdf/                 # URDF文件（如果使用urdf_package类型）
│   └── tool.urdf.xacro
└── meshes/               # 模型文件（如果使用simple类型）
    ├── visual/
    └── collision/
```

## 使用方法

### 1. 列出所有可用工具

```bash
rosrun frcobot_tools list_tools.py
```

### 2. 在启动文件中指定工具

```xml
<launch>
  <arg name="tool_name" default="tool_250mm"/>
  <!-- ... -->
</launch>
```

或在命令行中：

```bash
roslaunch fr5v6_single_moveit_config demo.launch tool_name:=tool_250mm
```

### 3. 添加新工具

1. 复制工具模板：
   ```bash
   cp -r tools/tool_template tools/tool_my_new_tool
   ```

2. 编辑 `tools/tool_my_new_tool/config/tool.yaml`

3. 添加工具资源（mesh文件或URDF文件）

4. 重新编译：
   ```bash
   cd ~/zlab_robots/catkin_ws
   catkin_make
   ```

## 工具配置说明

详见 `tools/tool_template/config/tool.yaml` 中的注释说明。

## 支持的工具类型

- **simple**: 使用mesh文件的简单工具配置
- **urdf_package**: 使用完整URDF包的工具配置（适用于SolidWorks导出的工具）

## 注意事项

1. 工具名称必须与目录名一致
2. `config/tool.yaml` 文件是必需的
3. 工具包路径使用 `package://frcobot_tools/tools/tool_name/` 格式
4. 确保工具配置文件中的路径正确

