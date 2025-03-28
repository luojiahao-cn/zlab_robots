#include <ros/ros.h>
#include <QApplication>
#include <QMainWindow>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QGridLayout>
#include <QGroupBox>
#include <QLabel>
#include <QProgressBar>
#include <QCheckBox>
#include <QTimer>
#include <QString>
#include <QMap>
#include <frcobot_status/status.h>

class FRCobotStatusGUI : public QMainWindow
{
public:
    FRCobotStatusGUI(int argc, char **argv, QWidget *parent = nullptr)
        : QMainWindow(parent), new_status_received_(false)
    {
        // 初始化ROS
        ros::init(argc, argv, "frcobot_status_gui");
        nh_ = new ros::NodeHandle();
        
        // 初始化故障码映射
        initErrorCodeMap();
        
        // 订阅状态话题
        status_sub_ = nh_->subscribe("/frcobot_status", 10,
                                     &FRCobotStatusGUI::statusCallback, this);

        initUI();

        // 设置定时器处理ROS消息
        ros_timer_ = new QTimer(this);
        connect(ros_timer_, &QTimer::timeout, this, &FRCobotStatusGUI::handleRosMsgs);
        ros_timer_->start(100); // 10Hz

        setWindowTitle("FR机器人状态监视器");
        resize(900, 700);
    }

    ~FRCobotStatusGUI()
    {
        delete nh_;
        delete ros_timer_;
    }

private:
    // ROS相关
    ros::NodeHandle *nh_;
    ros::Subscriber status_sub_;
    frcobot_status::status current_status_;
    bool new_status_received_;

    // 界面组件
    QWidget *central_widget_;
    QLabel *program_state_label_;
    QLabel *robot_mode_label_;
    QLabel *error_code_label_;
    QLabel *error_description_label_; // 添加故障描述标签
    QProgressBar *joint_position_bars_[6];
    QLabel *joint_position_values_[6];
    QLabel *tool_position_labels_[6];
    QProgressBar *joint_torque_bars_[6];
    QLabel *joint_torque_values_[6];
    QCheckBox *io_checkboxes_[24]; // 简化IO显示
    QLabel *robot_motion_done_label_;
    QLabel *gripper_motion_done_label_;

    // 定时器
    QTimer *ros_timer_;

    // 故障码映射
    QMap<int, QString> error_code_map_;

    void initUI()
    {
        central_widget_ = new QWidget(this);
        setCentralWidget(central_widget_);

        QVBoxLayout *main_layout = new QVBoxLayout(central_widget_);

        // 机器人状态概览区
        QGroupBox *overview_group = new QGroupBox("机器人状态概览", central_widget_);
        QGridLayout *overview_layout = new QGridLayout(overview_group);

        overview_layout->addWidget(new QLabel("程序状态:"), 0, 0);
        program_state_label_ = new QLabel("未知");
        overview_layout->addWidget(program_state_label_, 0, 1);

        overview_layout->addWidget(new QLabel("机器人模式:"), 0, 2);
        robot_mode_label_ = new QLabel("未知");
        overview_layout->addWidget(robot_mode_label_, 0, 3);

        overview_layout->addWidget(new QLabel("错误代码:"), 1, 0);
        error_code_label_ = new QLabel("0");
        overview_layout->addWidget(error_code_label_, 1, 1);

        overview_layout->addWidget(new QLabel("故障描述:"), 1, 2);
        error_description_label_ = new QLabel("无故障");
        // 设置错误描述标签的样式
        error_description_label_->setStyleSheet("QLabel { color: green; font-weight: bold; }");
        overview_layout->addWidget(error_description_label_, 1, 3);

        main_layout->addWidget(overview_group);

        // 关节位置区域
        QGroupBox *joint_position_group = new QGroupBox("关节位置(度)", central_widget_);
        QGridLayout *joint_pos_layout = new QGridLayout(joint_position_group);

        for (int i = 0; i < 6; i++)
        {
            joint_pos_layout->addWidget(new QLabel(QString("关节 %1:").arg(i + 1)), i, 0);

            joint_position_bars_[i] = new QProgressBar();
            joint_position_bars_[i]->setRange(-180, 180);
            joint_position_bars_[i]->setValue(0);
            joint_pos_layout->addWidget(joint_position_bars_[i], i, 1);

            joint_position_values_[i] = new QLabel("0.000");
            joint_pos_layout->addWidget(joint_position_values_[i], i, 2);
        }

        main_layout->addWidget(joint_position_group);

        // 工具位置区域
        QGroupBox *tool_position_group = new QGroupBox("工具位置与姿态", central_widget_);
        QGridLayout *tool_pos_layout = new QGridLayout(tool_position_group);

        QStringList pos_labels = {"X (mm):", "Y (mm):", "Z (mm):", "A (度):", "B (度):", "C (度):"};
        for (int i = 0; i < 6; i++)
        {
            tool_pos_layout->addWidget(new QLabel(pos_labels[i]), i / 3, (i % 3) * 2);

            tool_position_labels_[i] = new QLabel("0.000");
            tool_pos_layout->addWidget(tool_position_labels_[i], i / 3, (i % 3) * 2 + 1);
        }

        main_layout->addWidget(tool_position_group);

        // 运动状态
        QGroupBox *motion_status_group = new QGroupBox("运动状态", central_widget_);
        QGridLayout *motion_layout = new QGridLayout(motion_status_group);

        motion_layout->addWidget(new QLabel("机器人运动到位:"), 0, 0);
        robot_motion_done_label_ = new QLabel("否");
        motion_layout->addWidget(robot_motion_done_label_, 0, 1);

        motion_layout->addWidget(new QLabel("夹爪运动状态:"), 0, 2);
        gripper_motion_done_label_ = new QLabel("运动中");
        motion_layout->addWidget(gripper_motion_done_label_, 0, 3);

        main_layout->addWidget(motion_status_group);
    }

    void initErrorCodeMap()
    {
        error_code_map_[0] = "无故障";
        error_code_map_[1] = "驱动器故障";
        error_code_map_[2] = "超出软限位故障";
        error_code_map_[3] = "碰撞故障";
        error_code_map_[4] = "奇异位姿";
        error_code_map_[5] = "从站错误";
        error_code_map_[6] = "指令点错误";
        error_code_map_[7] = "IO错误";
        error_code_map_[8] = "夹爪错误";
        error_code_map_[9] = "文件错误";
        error_code_map_[10] = "参数错误";
        error_code_map_[11] = "扩展轴超出软限位错误";
        error_code_map_[12] = "关节配置警告";
    }

    void statusCallback(const frcobot_status::status::ConstPtr &msg)
    {
        current_status_ = *msg;
        new_status_received_ = true;
    }

    void handleRosMsgs()
    {
        ros::spinOnce();

        if (new_status_received_)
        {
            updateStatus();
            new_status_received_ = false;
        }
    }

    void updateStatus()
    {
        // 更新程序状态
        QString state_str;
        switch (current_status_.program_state)
        {
        case 1:
            state_str = "停止";
            break;
        case 2:
            state_str = "运行";
            break;
        case 3:
            state_str = "暂停";
            break;
        case 4:
            state_str = "拖动";
            break;
        default:
            state_str = "未知";
        }
        program_state_label_->setText(state_str);

        // 更新机器人模式
        QString mode_str;
        switch (current_status_.robot_mode)
        {
        case 0:
            mode_str = "自动模式";
            break;
        case 1:
            mode_str = "手动模式";
            break;
        case 2:
            mode_str = "拖动模式";
            break;
        default:
            mode_str = "未知";
        }
        robot_mode_label_->setText(mode_str);

        // 更新错误代码
        int error_code = current_status_.error_code;
        error_code_label_->setText(QString::number(error_code));

        // 更新故障描述
        if (error_code_map_.contains(error_code)) {
            error_description_label_->setText(error_code_map_[error_code]);
            
            // 根据故障严重性更改颜色
            if (error_code == 0) {
                error_description_label_->setStyleSheet("QLabel { color: green; font-weight: bold; }");
            } else if (error_code <= 4) {
                error_description_label_->setStyleSheet("QLabel { color: red; font-weight: bold; }");
            } else {
                error_description_label_->setStyleSheet("QLabel { color: orange; font-weight: bold; }");
            }
        } else {
            error_description_label_->setText("未知故障");
            error_description_label_->setStyleSheet("QLabel { color: red; font-weight: bold; }");
        }

        // 更新关节位置
        for (int i = 0; i < 6 && i < current_status_.cur_joints_pose.size(); i++)
        {
            double value = current_status_.cur_joints_pose[i];
            joint_position_bars_[i]->setValue(int(value));
            joint_position_values_[i]->setText(QString::number(value, 'f', 3));
        }

        // 更新工具位置
        for (int i = 0; i < 6 && i < current_status_.cur_tcp_pose.size(); i++)
        {
            tool_position_labels_[i]->setText(QString::number(current_status_.cur_tcp_pose[i], 'f', 3));
        }

        // 更新运动状态
        robot_motion_done_label_->setText(current_status_.robot_motion_done ? "是" : "否");

        // 更新夹爪状态
        QString gripper_str;
        switch (current_status_.gripper_motion_done)
        {
        case 0:
            gripper_str = "运动未完成";
            break;
        case 1:
            gripper_str = "停止(打开过程)";
            break;
        case 2:
            gripper_str = "停止(关闭过程)";
            break;
        case 3:
            gripper_str = "停止(指定位置)";
            break;
        default:
            gripper_str = "未知";
        }
        gripper_motion_done_label_->setText(gripper_str);
    }
};

int main(int argc, char **argv)
{
    QApplication app(argc, argv);
    FRCobotStatusGUI gui(argc, argv);
    gui.show();
    return app.exec();
}