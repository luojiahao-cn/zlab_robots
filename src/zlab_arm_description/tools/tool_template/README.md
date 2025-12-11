# 工具模板

这是用于创建新的机器人工具的模板。复制此目录并修改文件来创建新的工具。

## 模板信息

- **名称**: tool_template
- **显示名称**: 工具模板
- **类型**: 配置驱动工具模板
- **版本**: 1.0.0

## 目录结构

```
tool_template/
├── README.md              # 工具说明文档（请修改）
├── config/
│   ├── tool.yaml         # 工具配置文件（必需，请修改）
│   └── joint_names_tool_template.yaml  # 关节名称配置（如果需要）
├── urdf/                 # URDF文件目录（如果使用urdf_package类型）
├── meshes/               # 模型文件目录（必需）
├── textures/             # 纹理文件目录（可选）
├── launch/               # 启动文件目录（可选）
├── package.xml           # ROS包定义（请修改包名）
└── CMakeLists.txt        # CMake构建文件
```

## 使用方法

### 1. 复制模板

```bash
cp -r tool_template your_new_tool_name
cd your_new_tool_name
```

### 2. 修改文件

#### 修改工具信息：
- `config/tool.yaml`: 配置工具参数（必需）
- `meshes/`: 添加3D模型文件（必需）
- `README.md`: 更新工具信息和说明
- `urdf/`: 如果使用urdf_package类型，添加URDF文件

### 3. 选择工具类型

根据您的工具特点选择合适的类型：

#### 简单工具类型（推荐）
- 使用 `builtin` 类型
- 只需提供可视化和碰撞mesh文件
- 自动生成TCP链接

#### URDF包类型
- 使用 `urdf_package` 类型
- 适用于从SolidWorks导出的完整URDF
- 需要提供完整的URDF文件

## 配置示例

### 简单工具配置 (builtin类型)

```yaml
tool_info:
  name: "my_tool"
  display_name: "我的工具"
  description: "工具描述"
  version: "1.0.0"
  author: "Your Name"

tool_type: "builtin"

# 工具相对于法兰的位置和姿态
origin:
  xyz: [0, 0, 0]
  rpy: [0, 0, 0]

# TCP相对于工具的位置和姿态
tcp:
  xyz: [0, 0, 0.1]
  rpy: [0, 0, 0]

# 物理属性
physical_properties:
  mass: 0.1  # kg
  inertia:
    ixx: 0.001
    ixy: 0.0
    ixz: 0.0
    iyy: 0.001
    iyz: 0.0
    izz: 0.001

# 可视化配置
visual:
  mesh_path: "package://your_tool_package/meshes/visual/tool.STL"
  scale: [0.001, 0.001, 0.001]
  color: [0.89804, 0.91765, 0.92941, 1]

# 碰撞配置
collision:
  mesh_path: "package://your_tool_package/meshes/collision/tool.STL"
  scale: [0.001, 0.001, 0.001]
```

### URDF包配置 (urdf_package类型)

```yaml
tool_info:
  name: "my_urdf_tool"
  display_name: "我的URDF工具"
  description: "基于URDF的工具"
  version: "1.0.0"
  author: "Your Name"

tool_type: "urdf_package"

# URDF包配置
urdf_package:
  urdf_file: "urdf/tool.urdf"
  root_link_name: "tool_base_link"
  tcp_link_name: "tool_tcp_link"
  mount_offset:
    xyz: [0, 0, 0]
    rpy: [0, 0, 0]

# 其他配置与builtin类型相同
origin:
  xyz: [0, 0, 0]
  rpy: [0, 0, 0]

tcp:
  xyz: [0, 0, 0.1]
  rpy: [0, 0, 0]

physical_properties:
  mass: 0.1
  inertia:
    ixx: 0.001
    ixy: 0.0
    ixz: 0.0
    iyy: 0.001
    iyz: 0.0
    izz: 0.001
```

## 测试工具

创建工具后，可以通过以下方式测试：

```bash
# 测试单个工具
roslaunch zlab_arm_description display_arm_rviz.launch tool_name:=your_tool_name

# 测试双臂工具
roslaunch zlab_arm_description display_dual_arm_rviz.launch arm1_tool:=your_tool_name arm2_tool:=your_tool_name
```

## 包结构说明

这些工具不是独立的ROS包，而是 `zlab_arm_description` 包的组成部分：

- **主包**: `zlab_arm_description`
- **工具目录**: `tools/your_tool_name/`
- **配置访问**: 通过 `zlab_arm_description` 包访问
- **文件路径**: `package://zlab_arm_description/tools/your_tool_name/...`

## 注意事项

1. 确保mesh文件路径正确
2. TCP链接不应该有可视化内容，只用于运动学计算
3. 工具质量和惯性要根据实际测量设置
4. 如果mesh单位是毫米，使用scale: [0.001, 0.001, 0.001]进行缩放
5. 配置文件中的所有路径都是相对于工具包目录的

## 常见问题

### Q: 如何确定TCP位置？
A: TCP（Tool Center Point）是工具的工作中心点，通常是工具末端或工作位置。

### Q: mesh文件放在哪个目录？
A: 建议放在 `meshes/` 目录下，可以创建 `visual/` 和 `collision/` 子目录。

### Q: 如何修改工具的安装位置？
A: 修改 `origin` 中的 `xyz` 和 `rpy` 参数。

### Q: 工具质量如何测量？
A: 使用电子秤测量工具总质量，惯性可以通过CAD软件计算或使用默认值。
