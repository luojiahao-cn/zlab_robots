#!/bin/bash

# 设置数据目录
DATA_DIR="$HOME/workshop/zlab_robots/catkin_ws/data/magnetic_data"

# 列出可用的数据文件
echo "可用的数据文件："
ls -lt "$DATA_DIR"/*.bag

# 让用户选择要回放的文件
echo "请输入要回放的文件名（不含路径）："
read FILENAME

if [ -f "$DATA_DIR/$FILENAME" ]; then
    echo "开始回放 $FILENAME..."
    rosbag play --clock "$DATA_DIR/$FILENAME"
else
    echo "错误：文件不存在"
fi