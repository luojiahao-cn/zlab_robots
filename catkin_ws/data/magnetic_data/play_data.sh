#!/bin/bash

# 设置数据目录
DATA_DIR="$HOME/workshop/zlab_robots/catkin_ws/data/magnetic_data"

# 获取所有.bag文件
BAG_FILES=("$DATA_DIR"/*.bag)

# 检查文件数量
if [ ${#BAG_FILES[@]} -eq 0 ]; then
    echo "错误：目录中没有.bag文件"
    exit 1
elif [ ${#BAG_FILES[@]} -eq 1 ]; then
    # 只有一个文件时自动播放
    FILENAME=$(basename "${BAG_FILES[0]}")
    echo "检测到单个数据文件，自动播放 $FILENAME..."
    rosbag play --clock "$DATA_DIR/$FILENAME" /magnetic_data:=/magnetic/sensor_data
else
    # 多个文件时让用户选择
    echo "可用的数据文件："
    ls -lt "$DATA_DIR"/*.bag
    
    echo "请输入要回放的文件名（不含路径）："
    read FILENAME
    
    if [ -f "$DATA_DIR/$FILENAME" ]; then
        echo "开始回放 $FILENAME..."
        rosbag play --clock "$DATA_DIR/$FILENAME" /magnetic_data:=/magnetic/sensor_data
    else
        echo "错误：文件不存在"
        exit 1
    fi
fi