#include <frcobot_status/frcobot_status.h>
#include <frcobot_status/status.h>
#include <fcntl.h>
#include <signal.h>

FrRobotStatusCtrl::FrRobotStatusCtrl()
{
    frrobot_status_ = nh_.advertise<frcobot_status::status>("frcobot_status", 10);
    last_recv_time_ = ros::Time::now();
    initTcp(); //Initialize the TCPIP connection with the robot
}

void FrRobotStatusCtrl::initTcp()
{
    // 创建私有命名空间的节点句柄
    ros::NodeHandle private_nh("~");

    // 从私有命名空间获取参数
    private_nh.param<std::string>("robot_ip", ROBOTIP, "192.168.31.202"); 
    private_nh.param<int>("robot_port", PORT, 8083);

    const char *robotIP = ROBOTIP.c_str();
    ROS_INFO("尝试连接到机器人: %s:%d", robotIP, PORT);
    
    // 设置服务器地址
    memset(&serverSendAddr, 0, sizeof(serverSendAddr));
    serverSendAddr.sin_family = AF_INET;
    serverSendAddr.sin_addr.s_addr = inet_addr(robotIP);
    serverSendAddr.sin_port = htons(PORT);
    sendaddr_length = sizeof(serverSendAddr);

    // 创建socket
    if ((confd = socket(AF_INET, SOCK_STREAM, 0)) < 0)
    {
        ROS_ERROR("创建socket失败: %s", strerror(errno));
        is_connected_ = false;
        return;
    }

    // 设置socket为非阻塞模式
    int flags = fcntl(confd, F_GETFL, 0);
    if (flags == -1) {
        ROS_ERROR("无法获取socket标志: %s", strerror(errno));
    } else {
        if (fcntl(confd, F_SETFL, flags | O_NONBLOCK) == -1) {
            ROS_ERROR("无法设置非阻塞模式: %s", strerror(errno));
        } else {
            ROS_INFO("Socket已设置为非阻塞模式");
        }
    }

    // 尝试连接
    int ret = connect(confd, (struct sockaddr *)&serverSendAddr, sizeof(serverSendAddr));
    if (ret < 0) {
        if (errno == EINPROGRESS) {
            // 连接正在进行中，这是非阻塞模式的正常现象
            ROS_INFO("连接正在进行中...");
            
            // 使用select等待连接完成或超时
            fd_set write_fds;
            struct timeval timeout;
            
            FD_ZERO(&write_fds);
            FD_SET(confd, &write_fds);
            
            // 设置5秒超时
            timeout.tv_sec = 5;
            timeout.tv_usec = 0;
            
            ret = select(confd + 1, NULL, &write_fds, NULL, &timeout);
            
            if (ret < 0) {
                ROS_ERROR("select错误: %s", strerror(errno));
                close(confd);
                is_connected_ = false;
                return;
            } else if (ret == 0) {
                // 超时
                ROS_ERROR("连接超时");
                close(confd);
                is_connected_ = false;
                return;
            } else {
                // 连接可能已完成，检查是否有错误
                int error = 0;
                socklen_t len = sizeof(error);
                if (getsockopt(confd, SOL_SOCKET, SO_ERROR, &error, &len) < 0 || error) {
                    ROS_ERROR("连接失败: %s", strerror(error ? error : errno));
                    close(confd);
                    is_connected_ = false;
                    return;
                }
            }
        } else {
            // 其他连接错误
            ROS_ERROR("连接失败: %s", strerror(errno));
            close(confd);
            is_connected_ = false;
            return;
        }
    }

    ROS_INFO("成功连接到机器人状态服务器");
    is_connected_ = true;
}

void FrRobotStatusCtrl::read()
{
    // 如果连接未建立，直接返回
    if (!is_connected_) {
        return;
    }

    recv_length = 0;
    recv_length = recv(confd, recv_buf, sizeof(recv_buf), 0);
    
    if (recv_length < 0) {
        if (errno == EAGAIN || errno == EWOULDBLOCK) {
            // 没有数据可读，检查是否超时
            if (first_read_) {
                // 首次调用不检查超时
                return;
            }
            
            ros::Duration elapsed = ros::Time::now() - last_recv_time_;
            if (elapsed.toSec() > RECV_TIMEOUT) {
                ROS_ERROR("超过%.1f秒没有接收到数据，判定连接已断开", RECV_TIMEOUT);
                close(confd);
                is_connected_ = false;
                exit(1); // 直接退出程序
            }
            return;
        } else {
            // 其他错误
            ROS_ERROR("接收数据错误: %s", strerror(errno));
            close(confd);
            is_connected_ = false;
            exit(1); // 直接退出程序
        }
    } else if (recv_length == 0) {
        // 连接已关闭
        ROS_ERROR("连接已关闭 (服务器端关闭连接)");
        close(confd);
        is_connected_ = false;
        exit(1); // 直接退出程序
    } else {
        // 成功接收数据，更新时间戳
        last_recv_time_ = ros::Time::now();
        first_read_ = false;
        
        // 成功接收数据，处理数据
        frrobot_status.frame_count = recv_buf[2];
        frrobot_status.program_state = recv_buf[5];
        frrobot_status.error_code = recv_buf[6];
        frrobot_status.robot_mode = recv_buf[7];

        // joints
        for (int j = 0; j < 6; j++)
        {
            for (int i = 0; i < 8; i++)
            {
                doubleByte[i] = recv_buf[(j+1)*8+i];
            }
            memcpy(&doubleTemp, doubleByte, sizeof(double));
            frrobot_status.joints_sta[j] = doubleTemp;
        }

        // tcp
        for (int j = 0; j < 6; j++)
        {
            for (int i = 0; i < 8; i++)
            {
                doubleByte[i] = recv_buf[(j+7)*8+i];
            }
            memcpy(&doubleTemp, doubleByte, sizeof(double));
            frrobot_status.tcp_sta[j] = doubleTemp;
        }

        // torque
        for (int j = 0; j < 6; j++)
        {
            for (int i = 0; i < 8; i++)
            {
                doubleByte[i] = recv_buf[int(j+13.5)*8+i];
            }
            memcpy(&doubleTemp, doubleByte, sizeof(double));
            frrobot_status.torque_sta[j] = doubleTemp;
        }

        // tool_num
        for (int i = 0; i < 4; i++)
        {
            intByte[i] = recv_buf[13*8+i];
        }
        memcpy(&intTemp, intByte, sizeof(int));
        frrobot_status.tool_num = intTemp;

        // cl_dtg_ouput (DO8-DO15)
        for (int i = 0; i < 8; i++)
        {
            frrobot_status.cl_o_h[i] = (recv_buf[578] >> i) & 0x01;
        }

        // cl_dtg_ouput (DO0-DO7)
        for (int i = 0; i < 8; i++)
        {
            frrobot_status.cl_o_l[i] = (recv_buf[579] >> i) & 0x01;
        }

        // tl_dtg_ouput (end_DO1-end_DO0)
        for (int i = 0; i < 2; i++)
        {
            frrobot_status.tl_o_l[i] = (recv_buf[580] >> i) & 0x01;
        }

        // robot_motion_done
        for (int i = 0; i < 4; i++)
        {
            intByte[i] = recv_buf[int(72.625*8)+i];
        }
        memcpy(&intTemp, intByte, sizeof(int));
        frrobot_status.robot_motion_done = intTemp;

        // gripper_motion_done
        frrobot_status.gripper_motion_done = recv_buf[585];
    }
}

void FrRobotStatusCtrl::update()
{
    // update status tpoic
    frcobot_status::status status_msg;

    status_msg.header.stamp = ros::Time::now();
    status_msg.frame_count = frrobot_status.frame_count;
    status_msg.program_state = frrobot_status.program_state;
    status_msg.error_code = frrobot_status.error_code;
    status_msg.robot_mode = frrobot_status.robot_mode;

    std::vector<double> joints_sta_(frrobot_status.joints_sta, frrobot_status.joints_sta+6);
    std::vector<double> tcp_sta_(frrobot_status.tcp_sta, frrobot_status.tcp_sta+6);
    std::vector<double> torque_sta_(frrobot_status.torque_sta, frrobot_status.torque_sta+6);
    status_msg.cur_joints_pose = joints_sta_;
    status_msg.cur_tcp_pose = tcp_sta_;
    status_msg.cur_joints_torque = torque_sta_;

    status_msg.tool_num = frrobot_status.tool_num;

    std::vector<uint8_t> cl_dgt_output_h_(frrobot_status.cl_o_h, frrobot_status.cl_o_h+8);
    std::vector<uint8_t> cl_dgt_output_l_(frrobot_status.cl_o_l, frrobot_status.cl_o_l+8);
    std::vector<uint8_t> tl_dgt_output_l_(frrobot_status.tl_o_l, frrobot_status.tl_o_l+2);
    status_msg.cl_dgt_output_h = cl_dgt_output_h_;
    status_msg.cl_dgt_output_l = cl_dgt_output_l_;
    status_msg.tl_dgt_output_l = tl_dgt_output_l_;

    status_msg.robot_motion_done = frrobot_status.robot_motion_done;
    status_msg.gripper_motion_done = frrobot_status.gripper_motion_done;
    
    frrobot_status_.publish(status_msg);
}

void FrRobotStatusCtrl::run()
{
    ros::Rate rate(125);
    while (ros::ok())
    {
        read();
        update();

        ros::spinOnce();
        rate.sleep();
    }
}

int main(int argc, char **argv)
{
    setlocale(LC_CTYPE, "zh_CN.utf8");

    ros::init(argc, argv, "FrRobotStatusCtrl");

    FrRobotStatusCtrl FrRobotStatusCtrl;

    FrRobotStatusCtrl.run();
}

