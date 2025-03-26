/**
 * Xbox 360控制器机器人伺服节点
 *
 * 使用Xbox 360控制器操作机械臂，支持笛卡尔模式和关节模式
 */

// ROS1头文件
#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <control_msgs/JointJog.h>
#include <geometry_msgs/TwistStamped.h>
#include <moveit_servo/servo.h>
#include <signal.h>
#include <string>

// Xbox控制机械臂的类
class XboxServo
{
public:
    XboxServo();
    ~XboxServo();

private:
    // ROS相关
    ros::NodeHandle nh_;       // 公共节点句柄
    ros::NodeHandle pnh_;      // 私有节点句柄
    ros::Publisher twist_pub_; // 扭曲命令发布者
    ros::Publisher joint_pub_; // 关节命令发布者
    ros::Subscriber joy_sub_;  // 手柄消息订阅者

    // 控制参数
    double linear_scale_;          // 线性速度系数
    double angular_scale_;         // 角速度系数
    double joint_scale_;           // 关节速度系数
    double deadzone_;              // 摇杆死区
    std::string command_frame_id_; // 命令坐标系ID
    bool is_joint_mode_;           // 是否为关节模式
    int current_joint_;            // 当前选择的关节
    bool is_xbox360_mode_;         // 是否为Xbox 360控制器模式

    // 话题名称
    std::string twist_topic_;       // 扭曲命令话题
    std::string joint_topic_;       // 关节命令话题
    std::string joy_topic_;         // 手柄输入话题
    std::string planning_frame_id_; // 规划坐标系ID
    std::string ee_frame_id_;       // 末端执行器坐标系ID

    // 按钮和轴映射
    struct ButtonMapping
    {
        int a;
        int b;
        int x;
        int y;
        int lb;
        int rb;
        int back;
        int start;
        int power;
        int left_stick;
        int right_stick;
        int dpad_left;
        int dpad_right;
        int dpad_up;
        int dpad_down;
    } buttons_;

    struct AxisMapping
    {
        int left_stick_x;
        int left_stick_y;
        int right_stick_x;
        int right_stick_y;
        int left_trigger;
        int right_trigger;
        int dpad_x;
        int dpad_y;
    } axes_;

    // 记录上一次的摇杆状态，用于确定何时需要停止
    double last_left_x_;
    double last_left_y_;
    double last_right_x_;
    double last_right_y_;
    double last_left_trigger_;
    double last_right_trigger_;

    // 主要功能函数
    void loadParameters();
    void joyCallback(const sensor_msgs::Joy::ConstPtr &joy);
    void sendTwistCommand(double x, double y, double z, double rx, double ry, double rz);
    void sendJointCommand(int joint_idx, double value);
    void stopAllMotion();

    // 辅助函数
    double applyDeadzone(double value);
    void printControllerHelp();
};

XboxServo::XboxServo() : pnh_("~"),
                         linear_scale_(0.2),
                         angular_scale_(0.2),
                         joint_scale_(0.1),
                         deadzone_(0.1),
                         is_joint_mode_(false),
                         current_joint_(0),
                         is_xbox360_mode_(true), // 默认为Xbox 360模式
                         twist_topic_("/servo_server/delta_twist_cmds"),
                         joint_topic_("/servo_server/delta_joint_cmds"),
                         joy_topic_("/joy"),
                         planning_frame_id_("world"),
                         ee_frame_id_("frrobot_tool_tcp_link"),
                         last_left_x_(0.0),
                         last_left_y_(0.0),
                         last_right_x_(0.0),
                         last_right_y_(0.0),
                         last_left_trigger_(0.0),
                         last_right_trigger_(0.0)
{
    // 加载参数
    loadParameters();

    // 创建发布者和订阅者
    twist_pub_ = nh_.advertise<geometry_msgs::TwistStamped>(twist_topic_, 10);
    joint_pub_ = nh_.advertise<control_msgs::JointJog>(joint_topic_, 10);
    joy_sub_ = nh_.subscribe<sensor_msgs::Joy>(joy_topic_, 10, &XboxServo::joyCallback, this);

    // 打印欢迎信息和控制器帮助
    ROS_INFO("Xbox 360控制器节点已初始化");
    ROS_INFO("按键配置: %s模式", is_xbox360_mode_ ? "Xbox 360" : "标准Xbox");
    printControllerHelp();

    // 设置调试级别
    bool debug;
    pnh_.param("debug", debug, false);
    if (debug)
    {
        if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug))
        {
            ros::console::notifyLoggerLevelsChanged();
            ROS_INFO("调试模式已启用");
        }
    }
}

XboxServo::~XboxServo()
{
    // 停止所有运动
    stopAllMotion();
    ROS_INFO("Xbox控制器节点已关闭");
}

void XboxServo::loadParameters()
{
    // 加载控制参数
    pnh_.param("linear_scale", linear_scale_, 0.2);
    pnh_.param("angular_scale", angular_scale_, 0.2);
    pnh_.param("joint_scale", joint_scale_, 0.1);
    pnh_.param("deadzone", deadzone_, 0.1);
    pnh_.param("is_xbox360_mode", is_xbox360_mode_, true);
    pnh_.param("planning_frame_id", planning_frame_id_, std::string("world"));
    pnh_.param("ee_frame_id", ee_frame_id_, std::string("frrobot_tool_tcp_link"));
    command_frame_id_ = planning_frame_id_;

    // 加载话题配置
    pnh_.param("twist_topic", twist_topic_, std::string("/servo_server/delta_twist_cmds"));
    pnh_.param("joint_topic", joint_topic_, std::string("/servo_server/delta_joint_cmds"));
    pnh_.param("joy_topic", joy_topic_, std::string("/joy"));

    // 加载按钮映射 - Xbox 360默认值
    if (is_xbox360_mode_)
    {
        // Xbox 360默认按钮映射
        pnh_.param("buttons/a", buttons_.a, 0);
        pnh_.param("buttons/b", buttons_.b, 1);
        pnh_.param("buttons/x", buttons_.x, 2);
        pnh_.param("buttons/y", buttons_.y, 3);
        pnh_.param("buttons/lb", buttons_.lb, 4);
        pnh_.param("buttons/rb", buttons_.rb, 5);
        pnh_.param("buttons/back", buttons_.back, 6);
        pnh_.param("buttons/start", buttons_.start, 7);
        pnh_.param("buttons/power", buttons_.power, 8);
        pnh_.param("buttons/left_stick", buttons_.left_stick, 9);
        pnh_.param("buttons/right_stick", buttons_.right_stick, 10);

        // Xbox 360没有十字键作为按钮，而是轴
        pnh_.param("buttons/dpad_left", buttons_.dpad_left, -1);
        pnh_.param("buttons/dpad_right", buttons_.dpad_right, -1);
        pnh_.param("buttons/dpad_up", buttons_.dpad_up, -1);
        pnh_.param("buttons/dpad_down", buttons_.dpad_down, -1);

        // Xbox 360特有的轴配置
        pnh_.param("axes/left_stick_x", axes_.left_stick_x, 0);
        pnh_.param("axes/left_stick_y", axes_.left_stick_y, 1);
        pnh_.param("axes/right_stick_x", axes_.right_stick_x, 3);
        pnh_.param("axes/right_stick_y", axes_.right_stick_y, 4);
        pnh_.param("axes/left_trigger", axes_.left_trigger, 2);   // 在Xbox 360中这是左右触发器共享的轴
        pnh_.param("axes/right_trigger", axes_.right_trigger, 2); // 同上
        pnh_.param("axes/dpad_x", axes_.dpad_x, 6);               // 十字键水平
        pnh_.param("axes/dpad_y", axes_.dpad_y, 7);               // 十字键垂直
    }
    else
    {
        // 标准Xbox控制器映射
        pnh_.param("buttons/a", buttons_.a, 0);
        pnh_.param("buttons/b", buttons_.b, 1);
        pnh_.param("buttons/x", buttons_.x, 2);
        pnh_.param("buttons/y", buttons_.y, 3);
        pnh_.param("buttons/lb", buttons_.lb, 4);
        pnh_.param("buttons/rb", buttons_.rb, 5);
        pnh_.param("buttons/back", buttons_.back, 6);
        pnh_.param("buttons/start", buttons_.start, 7);
        pnh_.param("buttons/power", buttons_.power, 8);
        pnh_.param("buttons/left_stick", buttons_.left_stick, 9);
        pnh_.param("buttons/right_stick", buttons_.right_stick, 10);
        pnh_.param("buttons/dpad_left", buttons_.dpad_left, 11);
        pnh_.param("buttons/dpad_right", buttons_.dpad_right, 12);
        pnh_.param("buttons/dpad_up", buttons_.dpad_up, 13);
        pnh_.param("buttons/dpad_down", buttons_.dpad_down, 14);

        pnh_.param("axes/left_stick_x", axes_.left_stick_x, 0);
        pnh_.param("axes/left_stick_y", axes_.left_stick_y, 1);
        pnh_.param("axes/right_stick_x", axes_.right_stick_x, 3);
        pnh_.param("axes/right_stick_y", axes_.right_stick_y, 4);
        pnh_.param("axes/left_trigger", axes_.left_trigger, 2);
        pnh_.param("axes/right_trigger", axes_.right_trigger, 5);
        pnh_.param("axes/dpad_x", axes_.dpad_x, -1); // 标准Xbox不使用
        pnh_.param("axes/dpad_y", axes_.dpad_y, -1); // 标准Xbox不使用
    }
}

void XboxServo::printControllerHelp()
{
    ROS_INFO("===== 控制器操作指南 =====");
    ROS_INFO("笛卡尔模式:");
    ROS_INFO("  左摇杆: 控制XY平面移动");
    ROS_INFO("  触发器: 控制Z轴移动 (左:下降, 右:上升)");
    ROS_INFO("  右摇杆: 控制姿态旋转");
    ROS_INFO("关节模式:");
    ROS_INFO("  十字键上/下: 选择关节");
    ROS_INFO("  左摇杆上/下: 控制当前选中关节");
    ROS_INFO("模式切换:");
    ROS_INFO("  X按钮: 切换到关节控制模式");
    ROS_INFO("  Y按钮: 切换到笛卡尔控制模式");
    ROS_INFO("坐标系切换:");
    ROS_INFO("  LB: 切换到规划坐标系");
    ROS_INFO("  RB: 切换到末端执行器坐标系");
    ROS_INFO("=========================");
}

// 辅助函数: 应用死区
double XboxServo::applyDeadzone(double value)
{
    return (std::abs(value) < deadzone_) ? 0.0 : value;
}

// 处理手柄输入的回调函数
void XboxServo::joyCallback(const sensor_msgs::Joy::ConstPtr &joy)
{
    // 调试信息: 显示按下的按钮
    for (size_t i = 0; i < joy->buttons.size(); ++i)
    {
        if (joy->buttons[i] == 1)
        {
            if (i == buttons_.a)
                ROS_DEBUG("按下: A按钮");
            else if (i == buttons_.b)
                ROS_DEBUG("按下: B按钮");
            else if (i == buttons_.x)
                ROS_DEBUG("按下: X按钮");
            else if (i == buttons_.y)
                ROS_DEBUG("按下: Y按钮");
            else if (i == buttons_.lb)
                ROS_DEBUG("按下: 左肩按钮(LB)");
            else if (i == buttons_.rb)
                ROS_DEBUG("按下: 右肩按钮(RB)");
            else if (i == buttons_.back)
                ROS_DEBUG("按下: Back按钮");
            else if (i == buttons_.start)
                ROS_DEBUG("按下: Start按钮");
            else if (i == buttons_.power)
                ROS_DEBUG("按下: 电源按钮");
            else if (i == buttons_.left_stick)
                ROS_DEBUG("按下: 左摇杆按钮");
            else if (i == buttons_.right_stick)
                ROS_DEBUG("按下: 右摇杆按钮");
            else if (i == buttons_.dpad_left)
                ROS_DEBUG("按下: 十字键左");
            else if (i == buttons_.dpad_right)
                ROS_DEBUG("按下: 十字键右");
            else if (i == buttons_.dpad_up)
                ROS_DEBUG("按下: 十字键上");
            else if (i == buttons_.dpad_down)
                ROS_DEBUG("按下: 十字键下");
            else
                ROS_DEBUG("按下: 未知按钮 %zu", i);
        }
    }

    // 调试信息: 显示摇杆状态
    for (size_t i = 0; i < joy->axes.size(); ++i)
    {
        if (std::abs(joy->axes[i]) > deadzone_)
        {
            if (i == axes_.left_stick_x)
                ROS_DEBUG("左摇杆X轴: %.2f", joy->axes[i]);
            else if (i == axes_.left_stick_y)
                ROS_DEBUG("左摇杆Y轴: %.2f", joy->axes[i]);
            else if (i == axes_.right_stick_x)
                ROS_DEBUG("右摇杆X轴: %.2f", joy->axes[i]);
            else if (i == axes_.right_stick_y)
                ROS_DEBUG("右摇杆Y轴: %.2f", joy->axes[i]);
            else if (i == axes_.left_trigger && !is_xbox360_mode_)
                ROS_DEBUG("左触发器: %.2f", joy->axes[i]);
            else if (i == axes_.right_trigger && !is_xbox360_mode_)
                ROS_DEBUG("右触发器: %.2f", joy->axes[i]);
            else if (i == axes_.left_trigger && is_xbox360_mode_)
                ROS_DEBUG("触发器轴: %.2f", joy->axes[i]);
            else if (i == axes_.dpad_x)
                ROS_DEBUG("十字键X轴: %.2f", joy->axes[i]);
            else if (i == axes_.dpad_y)
                ROS_DEBUG("十字键Y轴: %.2f", joy->axes[i]);
            else
                ROS_DEBUG("未知轴 %zu: %.2f", i, joy->axes[i]);
        }
    }

    // 检查模式切换按钮
    if (joy->buttons.size() > buttons_.x && joy->buttons[buttons_.x] == 1)
    {
        is_joint_mode_ = true;
        ROS_INFO("切换到关节控制模式");
    }

    if (joy->buttons.size() > buttons_.y && joy->buttons[buttons_.y] == 1)
    {
        is_joint_mode_ = false;
        ROS_INFO("切换到笛卡尔控制模式");
    }

    // 检查坐标系切换按钮
    if (joy->buttons.size() > buttons_.lb && joy->buttons[buttons_.lb] == 1)
    {
        command_frame_id_ = planning_frame_id_;
        ROS_INFO_STREAM("命令坐标系设置为: " << planning_frame_id_);
    }

    if (joy->buttons.size() > buttons_.rb && joy->buttons[buttons_.rb] == 1)
    {
        command_frame_id_ = ee_frame_id_;
        ROS_INFO_STREAM("命令坐标系设置为: " << ee_frame_id_);
    }

    if (is_joint_mode_)
    {
        // 关节控制模式

        // 使用十字键选择关节
        bool joint_changed = false;

        if (is_xbox360_mode_ && joy->axes.size() > axes_.dpad_y)
        {
            // Xbox 360模式: 使用十字键轴值
            if (joy->axes[axes_.dpad_y] > 0.5)
            { // 十字键上
                current_joint_ = (current_joint_ + 1) % 6;
                joint_changed = true;
            }
            else if (joy->axes[axes_.dpad_y] < -0.5)
            { // 十字键下
                current_joint_ = (current_joint_ + 5) % 6;
                joint_changed = true;
            }
        }
        else if (!is_xbox360_mode_ && joy->buttons.size() > buttons_.dpad_up &&
                 joy->buttons.size() > buttons_.dpad_down)
        {
            // 标准Xbox模式: 使用十字键按钮
            if (joy->buttons[buttons_.dpad_up] == 1)
            {
                current_joint_ = (current_joint_ + 1) % 6;
                joint_changed = true;
            }
            else if (joy->buttons[buttons_.dpad_down] == 1)
            {
                current_joint_ = (current_joint_ + 5) % 6;
                joint_changed = true;
            }
        }

        if (joint_changed)
        {
            ROS_INFO("当前选择关节: %d", current_joint_ + 1);
        }

        // 左摇杆Y轴控制当前关节
        if (joy->axes.size() > axes_.left_stick_y)
        {
            double joint_value = applyDeadzone(-joy->axes[axes_.left_stick_y]); // 反转Y轴
            if (joint_value != 0.0)
            {
                sendJointCommand(current_joint_, joint_value);
            }
            else if (last_left_y_ != 0.0)
            {
                // 摇杆回中，停止运动
                stopAllMotion();
            }
            last_left_y_ = joint_value;
        }
    }
    else
    {
        // 笛卡尔控制模式

        // 读取摇杆值
        double leftX = 0.0, leftY = 0.0, rightX = 0.0, rightY = 0.0;

        if (joy->axes.size() > axes_.left_stick_x)
            leftX = applyDeadzone(joy->axes[axes_.left_stick_x]);
        if (joy->axes.size() > axes_.left_stick_y)
            leftY = applyDeadzone(-joy->axes[axes_.left_stick_y]); // 反转Y轴
        if (joy->axes.size() > axes_.right_stick_x)
            rightX = applyDeadzone(joy->axes[axes_.right_stick_x]);
        if (joy->axes.size() > axes_.right_stick_y)
            rightY = applyDeadzone(-joy->axes[axes_.right_stick_y]); // 反转Y轴

        // 处理触发器
        double leftTrigger = 0.0, rightTrigger = 0.0;

        if (is_xbox360_mode_ && joy->axes.size() > axes_.left_trigger)
        {
            // Xbox 360模式: 左右触发器共享一个轴
            double triggerAxis = joy->axes[axes_.left_trigger];
            if (triggerAxis < -deadzone_)
            {
                // 右触发器被按下（负值）
                rightTrigger = (-triggerAxis - deadzone_) / (1.0 - deadzone_);
            }
            else if (triggerAxis > deadzone_)
            {
                // 左触发器被按下（正值）
                leftTrigger = (triggerAxis - deadzone_) / (1.0 - deadzone_);
            }
        }
        else if (!is_xbox360_mode_ &&
                 joy->axes.size() > axes_.left_trigger &&
                 joy->axes.size() > axes_.right_trigger)
        {
            // 标准Xbox模式: 左右触发器是独立轴
            // 触发器通常范围为1(未按下)到-1(完全按下)
            leftTrigger = (1.0 - joy->axes[axes_.left_trigger]) / 2.0;
            rightTrigger = (1.0 - joy->axes[axes_.right_trigger]) / 2.0;

            if (leftTrigger < deadzone_)
                leftTrigger = 0.0;
            if (rightTrigger < deadzone_)
                rightTrigger = 0.0;
        }

        // Z轴控制 = 右触发器(上) - 左触发器(下)
        double zControl = rightTrigger - leftTrigger;

        // 如果有任何输入，发送命令
        if (leftX != 0.0 || leftY != 0.0 || zControl != 0.0 || rightX != 0.0 || rightY != 0.0)
        {
            sendTwistCommand(leftY, -leftX, zControl, rightY, -rightX, 0.0);
        }
        else if (last_left_x_ != 0.0 || last_left_y_ != 0.0 ||
                 last_right_x_ != 0.0 || last_right_y_ != 0.0 ||
                 last_left_trigger_ != 0.0 || last_right_trigger_ != 0.0)
        {
            // 所有输入都为零，但之前不为零，停止运动
            stopAllMotion();
        }

        // 更新上一次的状态
        last_left_x_ = leftX;
        last_left_y_ = leftY;
        last_right_x_ = rightX;
        last_right_y_ = rightY;
        last_left_trigger_ = leftTrigger;
        last_right_trigger_ = rightTrigger;
    }
}

// 发送扭曲命令
void XboxServo::sendTwistCommand(double x, double y, double z, double rx, double ry, double rz)
{
    geometry_msgs::TwistStamped twist_msg;
    twist_msg.header.stamp = ros::Time::now();
    twist_msg.header.frame_id = command_frame_id_;

    twist_msg.twist.linear.x = x * linear_scale_;
    twist_msg.twist.linear.y = y * linear_scale_;
    twist_msg.twist.linear.z = z * linear_scale_;

    twist_msg.twist.angular.x = rx * angular_scale_;
    twist_msg.twist.angular.y = ry * angular_scale_;
    twist_msg.twist.angular.z = rz * angular_scale_;

    twist_pub_.publish(twist_msg);
}

// 发送关节命令
void XboxServo::sendJointCommand(int joint_idx, double value)
{
    control_msgs::JointJog joint_msg;

    joint_msg.header.stamp = ros::Time::now();
    joint_msg.header.frame_id = planning_frame_id_;

    // 设置关节名称
    joint_msg.joint_names.resize(6);
    joint_msg.joint_names = {"frrobot_j1", "frrobot_j2", "frrobot_j3", "frrobot_j4",
                             "frrobot_j5", "frrobot_j6"};

    // 将所有关节速度初始化为0
    joint_msg.velocities.resize(6);
    std::fill(joint_msg.velocities.begin(), joint_msg.velocities.end(), 0.0);

    // 设置指定关节的速度
    if (joint_idx >= 0 && joint_idx < 6)
    {
        joint_msg.velocities[joint_idx] = value * joint_scale_;
    }

    joint_pub_.publish(joint_msg);
}

// 停止所有运动
void XboxServo::stopAllMotion()
{
    // 停止笛卡尔空间运动
    geometry_msgs::TwistStamped twist_msg;
    twist_msg.header.stamp = ros::Time::now();
    twist_msg.header.frame_id = command_frame_id_;

    // 所有速度设为0
    twist_msg.twist.linear.x = 0.0;
    twist_msg.twist.linear.y = 0.0;
    twist_msg.twist.linear.z = 0.0;
    twist_msg.twist.angular.x = 0.0;
    twist_msg.twist.angular.y = 0.0;
    twist_msg.twist.angular.z = 0.0;

    twist_pub_.publish(twist_msg);

    // 停止关节空间运动
    control_msgs::JointJog joint_msg;
    joint_msg.header.stamp = ros::Time::now();
    joint_msg.header.frame_id = planning_frame_id_;

    // 设置关节名称
    joint_msg.joint_names.resize(6);
    joint_msg.joint_names = {"frrobot_j1", "frrobot_j2", "frrobot_j3", "frrobot_j4",
                             "frrobot_j5", "frrobot_j6"};

    // 所有关节速度设为0
    joint_msg.velocities.resize(6);
    std::fill(joint_msg.velocities.begin(), joint_msg.velocities.end(), 0.0);

    // 发布停止关节运动的命令
    joint_pub_.publish(joint_msg);
}

// 主函数
int main(int argc, char **argv)
{
    setlocale(LC_CTYPE, "zh_CN.utf8");

    ros::init(argc, argv, "servo_xbox_input");

    XboxServo xbox_servo;

    ros::spin();

    return 0;
}