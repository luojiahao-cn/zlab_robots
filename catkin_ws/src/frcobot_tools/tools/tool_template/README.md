# 工具包模板

这是一个工具包模板，用于创建新的末端执行器工具包。

## 目录结构

```
tool_template/
├── README.md              # 工具说明文档
├── config/
│   └── tool.yaml         # 工具配置文件（必需）
├── urdf/                 # URDF文件目录（如果使用urdf_package类型）
│   └── tool.urdf.xacro   # 工具URDF文件
└── meshes/               # 模型文件目录（如果使用simple类型）
    ├── visual/           # 可视化模型
    └── collision/        # 碰撞模型
```

## 创建新工具的步骤

1. **复制模板目录**
   ```bash
   cp -r tool_template tool_my_new_tool
   ```

2. **修改工具配置**
   - 编辑 `config/tool.yaml`
   - 设置 `tool_info.name` 为工具目录名
   - 根据工具类型配置相应参数

3. **添加工具资源**
   - 如果使用 `simple` 类型：添加mesh文件到 `meshes/` 目录
   - 如果使用 `urdf_package` 类型：添加URDF文件到 `urdf/` 目录

4. **测试工具**
   ```bash
   roslaunch fr5v6_single_moveit_config demo.launch tool_name:=tool_my_new_tool
   ```

## 工具类型说明

### Simple类型
适用于简单的工具，只需要mesh文件和基本配置。

### URDF Package类型
适用于复杂的工具，特别是从SolidWorks导出的工具。可以包含完整的URDF定义、多个链接、关节等。

