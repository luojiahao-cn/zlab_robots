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

  <!-- 节点 1 -->
  <node pkg="zlab_robots_calibration" type="filter_node" name="filter_1" args="diana7_em_tcp diana7_em_tcp_filt">
      <param name="window_size" value="$(arg common_window_size)" />
      <param name="max_translation_jump" value="$(arg common_trans_jump)" />
      <param name="max_rotation_jump" value="$(arg common_rot_jump)" />
  </node>
```

## algorithm
首先确认新输入的坐标是否是剧烈抖动的坏数据，若是坏数据，则舍弃

而后通过滑动窗口法，取前序的若干帧数据，做中值滤波

输出滤波后的结果