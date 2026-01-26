# 支架工具包 (bracket)

新增的支架工具包。

## 工具信息

- **名称**: bracket
- **显示名称**: 支架
- **类型**: urdf_package
- **版本**: 1.0.0

## 目录结构

```
bracket/
├── README.md              # 工具说明文档
├── config/
│   ├── tool.yaml         # 工具配置文件（必需）
├── urdf/                 # URDF文件目录
│   ├── bracket_macro.xacro   # 工具宏定义
│   └── bracket.urdf.xacro    # 工具实例化
├── meshes/               # 模型文件目录
│   ├── bracket_base_link.STL
│   └── bracket_ee_Link.STL
├── launch/               # 启动文件目录
```
