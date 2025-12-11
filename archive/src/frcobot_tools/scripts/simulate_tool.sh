#!/bin/bash
# 启动Gazebo仿真查看工具安装情况

TOOL_NAME=${1:-permanent_magnet}

echo "=========================================="
echo "启动Gazebo仿真查看工具: $TOOL_NAME"
echo "=========================================="

# 设置ROS环境
source /opt/ros/noetic/setup.bash 2>/dev/null || \
source /opt/ros/melodic/setup.bash 2>/dev/null

if [ -d "$HOME/zlab_robots/catkin_ws/devel" ]; then
    source $HOME/zlab_robots/catkin_ws/devel/setup.bash
fi

echo ""
echo "启动选项:"
echo "1. 仅Gazebo仿真（无MoveIt）"
echo "2. Gazebo + MoveIt完整仿真"
echo ""
read -p "请选择 (1/2，默认2): " choice
choice=${choice:-2}

case $choice in
    1)
        echo ""
        echo "启动Gazebo仿真..."
        roslaunch fr5v6_single_moveit_config gazebo_with_tool.launch tool_name:=$TOOL_NAME
        ;;
    2)
        echo ""
        echo "启动Gazebo + MoveIt仿真..."
        roslaunch fr5v6_single_moveit_config demo_gazebo_with_tool.launch tool_name:=$TOOL_NAME
        ;;
    *)
        echo "无效选择，使用默认选项2"
        roslaunch fr5v6_single_moveit_config demo_gazebo_with_tool.launch tool_name:=$TOOL_NAME
        ;;
esac

