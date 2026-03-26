# 坐标系反投影方法 (Frame Reprojection) 说明文档

Frame Reprojector 用于将 ROS TF 坐标系实时投射并绘制在相机图像上。

## 1. 核心功能
`display.py` 脚本实现了以下功能：
- **实时图像处理**：订阅相机图像和内参 (`camera_info`)。
- **TF 坐标变换**：利用 `tf2` 将任意目标坐标系 (`target_frame`) 的原点或轴线转换到相机坐标系下。
- **图像投影**：使用 `image_geometry` 将 3D 空间点投影到 2D 图像平面。
- **可视化绘制**：支持在图像上绘制三轴 (Axes)、单轴 (Z-Axis) 或单点 (Point)。

## 2. 软件原理

反投影的数学过程如下：
1. **坐标转换**：获取目标坐标系 $T_{target}$ 相对于相机坐标系 $T_{cam}$ 的变换矩阵。
2. **3D 投影**：使用相机内参矩阵 $K$，将 3D 点 $(X, Y, Z)$ 投影为像素坐标 $(u, v)$：
   $$ s \begin{bmatrix} u \\ v \\ 1 \end{bmatrix} = K \begin{bmatrix} X \\ Y \\ Z \end{bmatrix} $$
3. **渲染**：在 OpenCV 图像上根据计算出的像素位置绘制线段或圆点。

## 3. 使用方法

### 3.1 启动方式
在 [localization_tagslam.launch](../launch/localization_tagslam.launch) 中已经集成了该节点。可以通过以下命令启动：

```bash
roslaunch zlab_robots_calibration localization_tagslam.launch
```

### 3.2 配置参数
在 Launch 文件中，可以通过以下参数配置 `frame_reprojector_node`：

| 参数名 | 类型 | 说明 | 默认值 |
| :--- | :--- | :--- | :--- |
| `image_topic` | `string` | 相机图像话题 | `/zed2i/zed_node/left/image_rect_gray` |
| `camera_info_topic` | `string` | 相机内参话题 | `/zed2i/zed_node/left/camera_info` |
| `camera_frame` | `string` | 相机坐标系 ID | `cam0` (或从 camera_info 自动获取) |
| `target_frames` | `list` | 待投影的目标坐标系列表 | - |
| `show_window` | `bool` | 是否开启本地 OpenCV 窗口显示 | `false` |

### 3.3 目标坐标系定义 (`target_frames`)
`target_frames` 是一个字典列表，支持以下配置格式，可以配置若干个需要投影的坐标系：

```yaml
- {frame: 'diana7_em_tcp_filt', type: 'axes', length: 0.1}  # 绘制三轴，长度 0.1m
- {frame: 'sensor_array_filt', type: 'point'}               # 只绘制原点
- {frame: 'another_frame', type: 'z_axis', length: 0.05}     # 只绘制 Z 轴
- ...
```
正确配置后，将会自动查询目标坐标系到`camera_frame`的变换

## 4. 依赖项
- ROS Melodic/Noetic
- `tf2_ros`, `tf2_geometry_msgs`
- `image_geometry` (vision_opencv)
- `cv_bridge`
- `opencv-python`

## 5. 输出话题
- `~reprojected_image` (`sensor_msgs/Image`): 带有投影结果的叠加图像。
