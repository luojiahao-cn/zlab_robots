#!/bin/bash
# 检查工具连接关系的脚本

TOOL_NAME=${1:-permanent_magnet}

echo "=========================================="
echo "检查工具连接关系: $TOOL_NAME"
echo "=========================================="

# 设置ROS环境
source /opt/ros/noetic/setup.bash 2>/dev/null || \
source /opt/ros/melodic/setup.bash 2>/dev/null

if [ -d "$HOME/zlab_robots/catkin_ws/devel" ]; then
    source $HOME/zlab_robots/catkin_ws/devel/setup.bash
fi

# 生成URDF
TEMP_URDF="/tmp/check_tool_${TOOL_NAME}.urdf"
echo "生成URDF..."
rosrun xacro xacro $(rospack find frcobot_description)/urdf/fr5v6_single_arm.xacro tool_name:=$TOOL_NAME > $TEMP_URDF 2>&1

if [ $? -ne 0 ]; then
    echo "错误: URDF生成失败"
    cat $TEMP_URDF
    exit 1
fi

echo "✓ URDF生成成功"
echo ""

# 检查关键链接
echo "1. 检查关键链接:"
echo "----------------------------------------"
echo "机械臂最后关节链接 (frrobot_j6_link):"
grep -q "name=\"frrobot_j6_link\"" $TEMP_URDF && echo "  ✓ 存在" || echo "  ✗ 不存在"

echo ""
echo "工具根链接 (base_link):"
grep -q "name=\"base_link\"" $TEMP_URDF && echo "  ✓ 存在" || echo "  ✗ 不存在"

echo ""
echo "末端执行器链接 (ee_link):"
grep -q "name=\"frrobot_ee_link\"" $TEMP_URDF && echo "  ✓ 存在" || grep -q "name=\"robot1_ee_link\"" $TEMP_URDF && echo "  ✓ 存在" || echo "  ✗ 不存在"

echo ""
echo "TCP链接 (tcp_link):"
grep -q "name=\"tcp_link\"" $TEMP_URDF && echo "  ✓ 存在" || echo "  - 不存在"

echo ""
echo "工具链接 (frrobot_ee_link) - 机器人本体末端执行器链接:"
grep -q "name=\"frrobot_ee_link\"" $TEMP_URDF && echo "  ✓ 存在" || grep -q "name=\"robot1_ee_link\"" $TEMP_URDF && echo "  ✓ 存在" || echo "  - 不存在"

# 检查关节连接
echo ""
echo "2. 检查关节连接关系:"
echo "----------------------------------------"

echo ""
echo "检查 j6_link -> base_link 的连接:"
JOINT_J6_TO_BASE=$(grep -A 10 "parent.*link.*frrobot_j6_link" $TEMP_URDF | grep -A 5 "child.*link.*base_link")
if [ -n "$JOINT_J6_TO_BASE" ]; then
    echo "  ✓ 找到连接关节:"
    echo "$JOINT_J6_TO_BASE" | head -8 | sed 's/^/    /'
else
    echo "  ✗ 未找到 j6_link -> base_link 的连接"
fi

echo ""
echo "检查 j6_link -> ee_link 的连接:"
JOINT_J6_TO_EE=$(grep -A 10 "parent.*link.*frrobot_j6_link\|parent.*link.*robot1_j6_link\|parent.*link.*robot2_j6_link" $TEMP_URDF | grep -A 5 "child.*link.*ee_link")
if [ -n "$JOINT_J6_TO_EE" ]; then
    echo "  ✓ 找到连接关节:"
    echo "$JOINT_J6_TO_EE" | head -8 | sed 's/^/    /'
else
    echo "  ⚠ 未找到 j6_link -> ee_link 的连接（可能使用其他链接名）"
fi

# 检查所有与j6_link相关的关节
echo ""
echo "3. 所有连接到 j6_link 的关节:"
echo "----------------------------------------"
grep -B 5 -A 10 "parent.*link.*frrobot_j6_link" $TEMP_URDF | grep -E "joint|parent|child" | head -15 | sed 's/^/  /'

# 检查工具相关的所有关节
echo ""
echo "4. 工具相关的所有关节:"
echo "----------------------------------------"
grep -E "joint.*name.*tool|joint.*name.*permanent_magnet|joint.*name.*tcp" $TEMP_URDF | head -10 | sed 's/^/  /'

# 检查末端执行器链接
echo ""
echo "5. 末端执行器链接检查:"
echo "----------------------------------------"
if grep -q "name=\"frrobot_ee_link\"" $TEMP_URDF || grep -q "name=\"robot1_ee_link\"" $TEMP_URDF || grep -q "name=\"robot2_ee_link\"" $TEMP_URDF; then
    echo "  ✓ ee_link存在（机器人本体末端执行器链接）"
    echo "  ✓ 无需在代码中额外设置末端执行器链接"
else
    echo "  ⚠ ee_link不存在，可能需要设置末端执行器链接"
fi

# 检查链接层次结构
echo ""
echo "6. 链接层次结构 (从j6_link开始):"
echo "----------------------------------------"
echo "frrobot_j6_link"
grep -A 10 "parent.*link.*frrobot_j6_link" $TEMP_URDF | grep "child.*link" | sed 's/.*child.*link="\([^"]*\)".*/  └─> \1/' | head -3

CHILD_LINK=$(grep -A 10 "parent.*link.*frrobot_j6_link" $TEMP_URDF | grep "child.*link" | head -1 | sed 's/.*child.*link="\([^"]*\)".*/\1/')
if [ -n "$CHILD_LINK" ]; then
    echo "    └─> $CHILD_LINK"
    grep -A 10 "parent.*link.*$CHILD_LINK" $TEMP_URDF | grep "child.*link" | sed 's/.*child.*link="\([^"]*\)".*/      └─> \1/' | head -3
fi

echo ""
echo "=========================================="
echo "检查完成"
echo "=========================================="

