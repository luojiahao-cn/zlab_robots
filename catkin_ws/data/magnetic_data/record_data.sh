#!/bin/bash

# 创建保存数据的目录
DATA_DIR="$HOME/workshop/zlab_robots/catkin_ws/data/magnetic_data"
mkdir -p "$DATA_DIR"

# 生成带时间戳的文件名
TIMESTAMP=$(date +%Y%m%d_%H%M%S)
FILENAME="magnetic_and_tool_$TIMESTAMP.bag"

# 切换到数据目录
cd "$DATA_DIR"

# 记录指定话题
rosbag record -O "$FILENAME" \
    /magnetic_data \
    /tf \
    /tf_static

echo "数据已保存到: $DATA_DIR/$FILENAME"