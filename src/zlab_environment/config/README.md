# 环境添加与更新指南

## 1. 手动添加新障碍物
在 `config/` 目录下创建或修改 YAML 文件。每个障碍物项需包含以下字段：
- `id`: 唯一标识符（字符串）。
- `type`: `box`, `cylinder`, 或 `sphere`。
- `frame_id`: 坐标系名称。建议使用 `world` 或 `{arm_id}_pedestal_link`。
- `position`: `[x, y, z]`（米）。
- `orientation`: `[x, y, z, w]`（四元数）。
- `dimensions`: 
  - `box`: `[长, 宽, 高]`
  - `cylinder`: `[高度, 半径]`
  - `sphere`: `[半径]`
- `color`: `[r, g, b, a]`（0.0-1.0）。

## 2. 根据真实机械臂位姿设置障碍物
若需在机械臂当前末端位置设置障碍物（例如“末端前方 1cm 处的墙”），可参考以下步骤：

### 第一步：获取当前位姿
在终端运行 `tf_echo` 获取末端相对于基座或世界坐标系的位姿：
```bash
# 获取 diana7 末端相对于 world 的位姿 (使用 diana7_ee_link 作为末端坐标)
rosrun tf tf_echo world diana7_ee_link
```
输出示例：
- Translation: `[-0.313, -0.137, 1.370]`
- Rotation (Quaternion): `[-0.307, -0.629, 0.252, 0.668]`

### 第二步：计算偏移位置
通常机械臂末端（Flange）的法线方向为 Z 轴。
- **前方 1cm**：中心点位置 = $P_{ee} + (0.01 + \text{厚度}/2) \times \vec{Z}_{world}$
- **朝向偏移**：四元数通常直接沿用末端的四元数即可使墙面与末端面平行。

### 第三步：更新配置文件
将计算出的 `position` 和 `orientation` 填入 YAML 的 `collision_objects` 列表中。

## 3. 使用占位符增强通用性
- **使用 `{arm_id}`**：在 `frame_id` 中使用 `{arm_id}_base_link`，加载时通过 launch 文件的 `arm_id` 参数即可自动适配不同的机械臂，无需为每个机械臂写死坐标系。
- **混合坐标系**：同一文件中可以有部分物体依附于 `world`，部分依附于机器人基座。
