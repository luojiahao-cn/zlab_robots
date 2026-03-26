# frame_description

这个markdown用于描述视觉系统下各个坐标系的定义，以及如何正确查询变换和发布新的坐标

## frames
启动launch文件`roslaunch zlab_robots_calibration localization_tagslam.launch`该脚本集成了视觉定位、滤波和坐标反投影功能，会自动发布坐标变换

以下列出了需要关注的坐标系：

```
sensor_array：传感器阵列原始视觉位姿
diana_em_tcp：戴安娜电磁铁tcp原始视觉位姿
arm1_em_tcp：arm1电磁铁tcp原始视觉位姿
arm2_em_tcp：arm2电磁铁tcp视觉位姿
diana_em_tcp_filt：戴安娜电磁铁tcp滤波后视觉位姿
arm1_em_tcp_filt：arm1电磁铁tcp滤波后视觉位姿
arm2_em_tcp_filt：arm2电磁铁tcp滤波后视觉位姿
sensor_array_filt：传感器阵列滤波后视觉位姿
cam0：摄像机位姿
lab_table：视觉坐标系原点（A4纸标定板，该坐标系原点定义在A4纸左上角）
```

## usage

### 查询两坐标系之间的变换

在 ROS 中查询两个坐标系（如 `target_frame` 相对于 `source_frame`）的变换，建议使用 `tf2_ros`。

1. 初始化 TF2 Buffer 和 Listener

```python
import rospy
import tf2_ros

# 初始化缓冲区和监听器
tf_buffer = tf2_ros.Buffer()
tf_listener = tf2_ros.TransformListener(tf_buffer)
```

2. 获取变换并提取 XYZ 位置

通过 `lookup_transform` 可以获取完整的变换信息。如果只需要提取具体的 `x, y, z` 位置，可以按照以下方式操作：

```python
try:
    # 查询变换：target_frame 相对于 source_frame 的变换
    # rospy.Time(0) 表示获取最新可用的变换
    trans = tf_buffer.lookup_transform(target_frame, source_frame, rospy.Time(0), rospy.Duration(1.0))

    # 提取具体的 XYZ 位置
    x = trans.transform.translation.x
    y = trans.transform.translation.y
    z = trans.transform.translation.z

    rospy.loginfo(f"Position: x={x:.3f}, y={y:.3f}, z={z:.3f}")

except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
    rospy.logerr(f"TF Translation lookup failed: {e}")
```

### 发布新的变换

在前序步骤中已经完成了查找`target_frame`在`source_frame`中的位姿。经过进解算后，如果想要发布`estimated_frame`在`source_frame`中的坐标，可以使用 `tf2_ros.TransformBroadcaster`：

#### 1. 初始化发布器

```python
import rospy
import tf2_ros
import geometry_msgs.msg

# 初始化坐标发布器
br = tf2_ros.TransformBroadcaster()
```

#### 2. 构造并发送变换

你需要构造一个 `TransformStamped` 消息来描述两坐标系之间的位置（Translation）和姿态（Rotation/Quaternion）。

```python
# 构造变换消息
t = geometry_msgs.msg.TransformStamped()

# 设置时间戳和父子坐标系
t.header.stamp = rospy.Time.now()
t.header.frame_id = "source_frame"     # 父坐标系
t.child_frame_id = "estimated_frame"  # 子坐标系

# 设置具体的 XYZ 位置 (从解算结果中提取)
t.transform.translation.x = x
t.transform.translation.y = y
t.transform.translation.z = z

# 设置姿态 (四元数，如果是无旋转则为 0,0,0,1)
t.transform.rotation.x = qx
t.transform.rotation.y = qy
t.transform.rotation.z = qz
t.transform.rotation.w = qw

# 发布变换
br.sendTransform(t)
```

在这样发布后，新发布的坐标系将会自动与父坐标系所在坐标树进行联系，可以查询到树中任意两坐标之间的变换

在本系统中，`source_frame`一般可以选定为`lab_table`相机基本坐标系或者

> **注意**：TF 变换通常需要在循环中持续发布，或者由静态发布器（StaticTransformBroadcaster）发布一次。如果是传感器解算出的实时位姿，建议在数据更新的回调函数中调用 `sendTransform`。
