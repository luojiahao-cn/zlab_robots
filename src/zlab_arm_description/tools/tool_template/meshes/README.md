# 网格文件目录

此目录包含工具的3D模型文件。

## 目录结构

```
meshes/
├── visual/          # 可视化模型（高质量，包含细节）
│   └── tool_visual.STL
├── collision/       # 碰撞模型（简化几何体，用于碰撞检测）
│   └── tool_collision.STL
└── tool_visual.STL  # 如果不需要分离visual/collision，可以直接放在这里
```

## 文件命名约定

- 使用 `.STL` 或 `.DAE` 格式
- visual文件用于渲染显示
- collision文件用于碰撞检测（通常比visual文件更简单）
- 文件名应与 `config/tool.yaml` 中的配置匹配

## 注意事项

1. 如果您的CAD模型单位是毫米，请在配置文件中设置 `scale: [0.001, 0.001, 0.001]`
2. 确保mesh文件路径在 `config/tool.yaml` 中正确配置
3. visual和collision可以是同一个文件（如果不需要简化碰撞模型）

## 工具坐标系

- 工具的原点通常在法兰连接处
- Z轴通常指向工具前方
- TCP位置在 `config/tool.yaml` 中配置
