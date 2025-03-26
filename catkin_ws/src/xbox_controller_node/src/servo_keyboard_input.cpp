// ROS1头文件
#include <ros/ros.h>
#include <control_msgs/JointJog.h>
#include <geometry_msgs/TwistStamped.h>
#include <moveit_servo/servo.h>
#include <signal.h>
#include <stdio.h>
#include <string>
#include <cstring>
#include <fcntl.h>
#ifndef WIN32
#include <termios.h>
#include <unistd.h>
#else
#include <conio.h>
#endif

// 定义使用的按键
namespace
{
    constexpr int8_t KEYCODE_RIGHT = 0x43;   // 右箭头
    constexpr int8_t KEYCODE_LEFT = 0x44;    // 左箭头
    constexpr int8_t KEYCODE_UP = 0x41;      // 上箭头
    constexpr int8_t KEYCODE_DOWN = 0x42;    // 下箭头
    constexpr int8_t KEYCODE_PERIOD = 0x2E;  // 句号键
    constexpr int8_t KEYCODE_SEMICOLON = 0x3B; // 分号键
    constexpr int8_t KEYCODE_1 = 0x31;       // 1键
    constexpr int8_t KEYCODE_2 = 0x32;       // 2键
    constexpr int8_t KEYCODE_3 = 0x33;       // 3键
    constexpr int8_t KEYCODE_4 = 0x34;       // 4键
    constexpr int8_t KEYCODE_5 = 0x35;       // 5键
    constexpr int8_t KEYCODE_6 = 0x36;       // 6键
    constexpr int8_t KEYCODE_7 = 0x37;       // 7键
    constexpr int8_t KEYCODE_Q = 0x71;       // Q键
    constexpr int8_t KEYCODE_R = 0x72;       // R键
    constexpr int8_t KEYCODE_J = 0x6A;       // J键
    constexpr int8_t KEYCODE_T = 0x74;       // T键
    constexpr int8_t KEYCODE_W = 0x77;       // W键
    constexpr int8_t KEYCODE_E = 0x65;       // E键
} // namespace

// Servo远程操作演示中使用的常量
namespace
{
    const std::string TWIST_TOPIC = "/servo_server/delta_twist_cmds";   // 扭曲命令话题
    const std::string JOINT_TOPIC = "/servo_server/delta_joint_cmds";   // 关节命令话题
    const size_t ROS_QUEUE_SIZE = 10;                                  // ROS队列大小
    const std::string PLANNING_FRAME_ID = "world";                     // 规划坐标系ID
    const std::string EE_FRAME_ID = "frrobot_tool_tcp_link";           // 末端执行器坐标系ID
} // namespace

// 用于从终端读取按键输入的类
class KeyboardReader
{
public:
    KeyboardReader() : file_descriptor_(0)
    {
#ifndef WIN32
        // 获取原始模式下的控制台
        tcgetattr(file_descriptor_, &cooked_);
        struct termios raw;
        memcpy(&raw, &cooked_, sizeof(struct termios));
        raw.c_lflag &= ~(ICANON | ECHO);
        // 设置换行和文件结束符
        raw.c_cc[VEOL] = 1;
        raw.c_cc[VEOF] = 2;
        tcsetattr(file_descriptor_, TCSANOW, &raw);
#endif
    }
    
    void readOne(char *c)
    {
#ifndef WIN32
        int rc = read(file_descriptor_, c, 1);
        if (rc < 0)
        {
            throw std::runtime_error("read failed");
        }
#else
        *c = static_cast<char>(_getch());
#endif
    }
    
    void shutdown()
    {
#ifndef WIN32
        tcsetattr(file_descriptor_, TCSANOW, &cooked_);
#endif
    }

private:
    int file_descriptor_;
#ifndef WIN32
    struct termios cooked_;
#endif
};

// 将键盘输入转换为Twist或JointJog命令的类
class KeyboardServo
{
public:
    KeyboardServo();
    int keyLoop();

private:
    ros::NodeHandle nh_;                // ROS1节点句柄
    ros::Publisher twist_pub_;          // ROS1扭曲命令发布者
    ros::Publisher joint_pub_;          // ROS1关节命令发布者
    ros::ServiceClient switch_input_;   // ROS1服务客户端
    
    double joint_vel_cmd_;              // 关节速度命令
    std::string command_frame_id_;      // 命令坐标系ID
    bool key_pressed_;                  // 记录是否有按键被按下
    ros::Time last_key_press_time_;     // 上次按键时间
    
    void stopAllMotion();               // 停止所有运动的函数
};

KeyboardServo::KeyboardServo() : 
    joint_vel_cmd_(1.0), 
    command_frame_id_(PLANNING_FRAME_ID),
    key_pressed_(false)
{
    // ROS1风格发布者创建
    twist_pub_ = nh_.advertise<geometry_msgs::TwistStamped>(TWIST_TOPIC, ROS_QUEUE_SIZE);
    joint_pub_ = nh_.advertise<control_msgs::JointJog>(JOINT_TOPIC, ROS_QUEUE_SIZE);

    // 在ROS1中，服务客户端创建方式不同
    // 注意：如果没有对应的服务类型，需要创建一个
    // switch_input_ = nh_.serviceClient<moveit_msgs::ServoCommandType>("servo_server/switch_command_type");
    
    // 如果没有对应的服务，可以注释掉相关代码
    ROS_INFO("Keyboard servo node initialized");
}

// 停止所有运动
void KeyboardServo::stopAllMotion()
{
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
    joint_msg.header.frame_id = PLANNING_FRAME_ID;

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

KeyboardReader input;

void quit(int sig)
{
    (void)sig;
    input.shutdown();
    ros::shutdown();
    exit(0);
}

int main(int argc, char **argv)
{
    setlocale(LC_CTYPE, "zh_CN.utf8");
    
    // ROS1初始化方式
    ros::init(argc, argv, "servo_keyboard_input");
    
    KeyboardServo keyboard_servo;
    signal(SIGINT, quit);

    int rc = keyboard_servo.keyLoop();
    
    input.shutdown();
    ros::shutdown();
    
    return rc;
}

int KeyboardServo::keyLoop()
{
    char c;
    bool publish_twist = false;
    bool publish_joint = false;
    
    // ROS1无需创建单独的线程进行spin
    
    puts("从键盘读取输入");
    puts("---------------------------");
    puts("所有命令都在规划坐标系中");
    puts("使用方向键和'.'、';'键进行笛卡尔坐标系运动");
    puts("使用1|2|3|4|5|6键进行关节运动。按'r'反转运动方向。");
    puts("使用'j'选择关节运动模式。");
    puts("使用't'选择笛卡尔运动模式。");
    puts("使用'w'和'e'在规划坐标系和末端执行器坐标系之间切换命令发送");
    puts("按'q'退出程序。");

    ros::Rate loop_rate(100); // 100Hz的循环速率
    
    // 设置非阻塞读取
    int kfd = 0;
    int flags = fcntl(kfd, F_GETFL, 0);
    fcntl(kfd, F_SETFL, flags | O_NONBLOCK);

    while (ros::ok())
    {
        ros::spinOnce(); // 处理ROS回调
        
        // 尝试读取按键
        bool new_key_pressed = false;
        int n = 0;
        
        try
        {
            // 在非阻塞模式下尝试读取
            if (read(kfd, &c, 1) > 0) {
                new_key_pressed = true;
                last_key_press_time_ = ros::Time::now();
            }
        }
        catch (const std::runtime_error &)
        {
            perror("read():");
            return -1;
        }

        if (new_key_pressed) {
            ROS_DEBUG("value: 0x%02X", c);

            // 创建将要发布的消息
            geometry_msgs::TwistStamped twist_msg;
            control_msgs::JointJog joint_msg;

            joint_msg.joint_names.resize(6);
            joint_msg.joint_names = {"frrobot_j1", "frrobot_j2", "frrobot_j3", "frrobot_j4",
                                     "frrobot_j5", "frrobot_j6"};

            joint_msg.velocities.resize(6); // ROS1中没有resize(7)
            std::fill(joint_msg.velocities.begin(), joint_msg.velocities.end(), 0.0);
            
            // 根据按键设置命令
            switch (c)
            {
            case KEYCODE_LEFT:
                ROS_DEBUG("LEFT");
                twist_msg.twist.linear.y = -0.5;
                publish_twist = true;
                break;
            case KEYCODE_RIGHT:
                ROS_DEBUG("RIGHT");
                twist_msg.twist.linear.y = 0.5;
                publish_twist = true;
                break;
            case KEYCODE_UP:
                ROS_DEBUG("UP");
                twist_msg.twist.linear.x = 0.5;
                publish_twist = true;
                break;
            case KEYCODE_DOWN:
                ROS_DEBUG("DOWN");
                twist_msg.twist.linear.x = -0.5;
                publish_twist = true;
                break;
            case KEYCODE_PERIOD:
                ROS_DEBUG("PERIOD");
                twist_msg.twist.linear.z = -0.5;
                publish_twist = true;
                break;
            case KEYCODE_SEMICOLON:
                ROS_DEBUG("SEMICOLON");
                twist_msg.twist.linear.z = 0.5;
                publish_twist = true;
                break;
            case KEYCODE_1:
                ROS_DEBUG("1");
                joint_msg.velocities[0] = joint_vel_cmd_;
                publish_joint = true;
                break;
            case KEYCODE_2:
                ROS_DEBUG("2");
                joint_msg.velocities[1] = joint_vel_cmd_;
                publish_joint = true;
                break;
            case KEYCODE_3:
                ROS_DEBUG("3");
                joint_msg.velocities[2] = joint_vel_cmd_;
                publish_joint = true;
                break;
            case KEYCODE_4:
                ROS_DEBUG("4");
                joint_msg.velocities[3] = joint_vel_cmd_;
                publish_joint = true;
                break;
            case KEYCODE_5:
                ROS_DEBUG("5");
                joint_msg.velocities[4] = joint_vel_cmd_;
                publish_joint = true;
                break;
            case KEYCODE_6:
                ROS_DEBUG("6");
                joint_msg.velocities[5] = joint_vel_cmd_;
                publish_joint = true;
                break;
            case KEYCODE_R:
                ROS_DEBUG("r");
                joint_vel_cmd_ *= -1;
                break;
            case KEYCODE_J:
                ROS_DEBUG("j");
                // 注释掉ROS2特有的服务调用，如果需要可以用ROS1语法重写
                // request_ = std::make_shared<moveit_msgs::srv::ServoCommandType::Request>();
                // request_->command_type = moveit_msgs::srv::ServoCommandType::Request::JOINT_JOG;
                // if (switch_input_->wait_for_service(std::chrono::seconds(1)))
                // {
                //     auto result = switch_input_->async_send_request(request_);
                //     if (result.get()->success)
                //     {
                //         RCLCPP_INFO_STREAM(nh_->get_logger(), "Switched to input type: JointJog");
                //     }
                //     else
                //     {
                //         RCLCPP_WARN_STREAM(nh_->get_logger(), "Could not switch input to: JointJog");
                //     }
                // }
                ROS_INFO("Switched to input type: JointJog");
                break;
            case KEYCODE_T:
                ROS_DEBUG("t");
                // 同上，注释ROS2特有代码
                ROS_INFO("Switched to input type: Twist");
                break;
            case KEYCODE_W:
                ROS_DEBUG("w");
                ROS_INFO_STREAM("Command frame set to: " << PLANNING_FRAME_ID);
                command_frame_id_ = PLANNING_FRAME_ID;
                break;
            case KEYCODE_E:
                ROS_DEBUG("e");
                ROS_INFO_STREAM("Command frame set to: " << EE_FRAME_ID);
                command_frame_id_ = EE_FRAME_ID;
                break;
            case KEYCODE_Q:
                ROS_DEBUG("quit");
                return 0;
            }

            // 发布消息
            if (publish_twist)
            {
                twist_msg.header.stamp = ros::Time::now();
                twist_msg.header.frame_id = command_frame_id_;
                twist_pub_.publish(twist_msg);
                publish_twist = false;
                key_pressed_ = true;
            }
            else if (publish_joint)
            {
                joint_msg.header.stamp = ros::Time::now();
                joint_msg.header.frame_id = PLANNING_FRAME_ID;
                joint_pub_.publish(joint_msg);
                publish_joint = false;
                key_pressed_ = true;
            }
        }
        else {
            // 检查按键是否松开
            ros::Duration key_timeout(0.02); // 20ms超时
            if (key_pressed_ && (ros::Time::now() - last_key_press_time_) > key_timeout) {
                // 按键松开，停止运动
                stopAllMotion();
                key_pressed_ = false;
            }
        }
        
        loop_rate.sleep();
    }

    return 0;
}