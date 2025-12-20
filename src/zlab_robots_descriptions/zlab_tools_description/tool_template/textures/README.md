# 纹理文件目录

此目录包含工具的纹理文件（如有需要）。

## 支持的文件格式

- `.png`
- `.jpg`
- `.tiff`
- 其他标准图像格式

## 使用方法

在URDF文件中引用纹理：

```xml
<material name="tool_material">
  <texture filename="package://tool_template/textures/tool_texture.png"/>
</material>
```

## 注意事项

1. 纹理文件应该与mesh文件在同一坐标系中对齐
2. 建议使用PNG格式以支持透明度
3. 文件名应具有描述性
