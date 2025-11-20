# 工具包框架总结

## 已完成的工作

### 1. 工具包目录结构
- ✅ 创建了 `frcobot_tools` 包
- ✅ 建立了标准的工具包目录结构
- ✅ 创建了工具模板 `tool_template`
- ✅ 创建了示例工具：`tool_none` 和 `tool_250mm`

### 2. 工具配置系统
- ✅ 定义了工具配置YAML格式
- ✅ 支持两种工具类型：
  - `simple`: 使用mesh文件的简单工具
  - `urdf_package`: 使用完整URDF包的工具（适用于SolidWorks导出）

### 3. Xacro集成
- ✅ 创建了 `tool_loader.xacro` 用于加载工具
- ✅ 修改了 `fr5v6_library.xacro` 支持工具包系统
- ✅ 修改了 `fr5v6_single_arm.xacro` 和 `fr5v6_dual_arms.xacro` 支持工具参数
- ✅ 保持向后兼容，支持旧的 `tools.yaml` 配置

### 4. 工具管理工具
- ✅ 创建了 `list_tools.py` 脚本列出所有可用工具
- ✅ 创建了 `load_tool_config.py` 脚本用于加载工具配置

### 5. 文档
- ✅ 创建了 `README.md` 主文档
- ✅ 创建了 `USAGE.md` 使用指南
- ✅ 创建了工具模板的 `README.md`

## 系统架构

```
frcobot_tools/
├── tools/                    # 工具包目录
│   ├── tool_none/           # 无工具配置
│   ├── tool_250mm/          # 示例工具
│   ├── tool_template/       # 工具模板
│   └── your_tool/           # 用户自定义工具
│       ├── config/
│       │   └── tool.yaml    # 工具配置（必需）
│       ├── urdf/            # URDF文件（urdf_package类型）
│       └── meshes/          # 模型文件（simple类型）
├── scripts/                 # 工具脚本
│   ├── list_tools.py       # 列出工具
│   └── load_tool_config.py # 加载配置
└── README.md               # 文档
```

## 使用方法

### 1. 列出所有工具
```bash
rosrun frcobot_tools list_tools.py
```

### 2. 使用工具
```bash
# 单臂
roslaunch fr5v6_single_moveit_config demo.launch tool_name:=tool_250mm

# 双臂
roslaunch fr5v6_dual_moveit_config demo.launch robot1_tool:=tool_250mm robot2_tool:=tool_none
```

### 3. 添加新工具
1. 复制模板：`cp -r tools/tool_template tools/your_tool`
2. 编辑 `tools/your_tool/config/tool.yaml`
3. 添加资源文件（mesh或URDF）
4. 重新编译：`catkin_make`

## 关键文件

### 工具配置文件 (`config/tool.yaml`)
- 定义工具的所有配置信息
- 支持simple和urdf_package两种类型
- 包含物理属性、TCP位置、可视化配置等

### Xacro文件
- `tool_loader.xacro`: 工具加载器，处理工具的实际加载
- `fr5v6_library.xacro`: 修改后支持工具包系统
- `fr5v6_single_arm.xacro`: 添加了tool_name参数
- `fr5v6_dual_arms.xacro`: 添加了robot1_tool和robot2_tool参数

## 向后兼容

系统保持向后兼容：
- 如果工具包不存在，会自动尝试从旧的 `tools.yaml` 加载
- 旧的工具配置仍然可以使用
- 可以逐步迁移到新的工具包系统

## 下一步

1. **添加SolidWorks工具**:
   - 将SolidWorks导出的URDF文件放到工具包的 `urdf/` 目录
   - 配置 `tool.yaml` 使用 `urdf_package` 类型
   - 设置正确的 `root_link_name` 和 `tcp_link_name`

2. **测试工具切换**:
   - 测试不同工具之间的切换
   - 验证运动学计算是否正确
   - 检查可视化是否正确

3. **优化**:
   - 根据需要调整工具配置格式
   - 添加更多工具管理功能
   - 完善错误处理

## 注意事项

1. **工具包路径**: 使用 `package://frcobot_tools/tools/tool_name/` 格式
2. **工具名称**: 必须与目录名一致
3. **配置文件**: `config/tool.yaml` 是必需的
4. **编译**: 添加新工具后需要重新编译 `catkin_make`
5. **xacro限制**: xacro无法检查文件是否存在，如果工具包不存在会报错，需要确保工具包存在或使用旧配置

## 支持

如有问题，请参考：
- `README.md`: 概述和快速开始
- `USAGE.md`: 详细使用指南
- `tools/tool_template/README.md`: 工具包创建指南

