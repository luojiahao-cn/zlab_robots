# filter_node

## function
通过接受参数获得：
1. 订阅的坐标系，对该坐标系的变化进行实时监听，如果该坐标系发生变化，则将新值输入滤波方法中，并发布滤波后的结果
2. 发布的坐标系，将滤波后的结果发布在该参数指定的坐标系中，保证结果坐标系和订阅的坐标系拥有相同结构的父节点和根节点，能够查询到完整的坐标变换

## usage:
最小示例：
```
  <!-- ===================== -->
  <!-- 4. Filter Node -->
  <!-- ===================== -->
  <arg name="common_window_size" default="15" />
  <arg name="common_trans_jump" default="0.007" />
  <arg name="common_rot_jump" default="0.04" />
  <arg name="common_reset_timeout" default="500" />

  <!-- 节点 1 -->
  <node pkg="zlab_robots_calibration" type="filter_node" name="filter_1" args="diana7_em_tcp diana7_em_tcp_filt">
      <param name="window_size" value="$(arg common_window_size)" />
      <param name="max_translation_jump" value="$(arg common_trans_jump)" />
      <param name="max_rotation_jump" value="$(arg common_rot_jump)" />
      <param name="reset_timeout_ms" value="$(arg common_reset_timeout)" />
  </node>
```

### 参数说明

| 参数 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `window_size` | int | 5 | 滑动窗口大小，控制中值滤波的帧数 |
| `max_translation_jump` | double | 0.5 | 平移跳动阈值(m)，超过则视为坏数据舍弃 |
| `max_rotation_jump` | double | 0.5 | 旋转跳动阈值(rad)，超过则视为坏数据舍弃 |
| `reset_timeout_ms` | double | 500 | 超时重置时间(ms)，超过此时间未收到新坐标则重新初始化滤波器 |

## algorithm
1. **坏数据过滤**：首先确认新输入的坐标是否是剧烈抖动的坏数据，若是坏数据，则舍弃

2. **超时重置**：如果超过 `reset_timeout_ms` 时间未收到有效坐标，则重置滤波器，清空历史窗口，重新开始学习
   - 作用：防止视觉定位丢失后重新恢复时，错误的历史数据持续影响输出

3. **滑动窗口中值滤波**：通过滑动窗口法，取前序的若干帧数据，做中值滤波

4. **输出滤波后的结果**

## 应用场景
适用于视觉定位系统（如定位码跟踪）：
- 当定位码被遮挡或丢失后重新出现时，超时重置机制可以避免错误的历史偏移累积
- 避免因视觉瞬时抖动导致的定位漂移