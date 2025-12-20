#include <iostream>
#include <vector>
#include <unistd.h>
#include <cstring>
#include <cmath>
#include "DianaAPI.h"

// 简单的错误回调
void errorControl(int e, const char *strIpAddress)
{
    printf("DianaAPI Error Callback: %d\n", e);
    printf("Error Description: %s\n", formatError(e, strIpAddress));
}

// 简单的状态回调
void robotState(StrRobotStateInfo *pinfo, const char *strIpAddress)
{
    static int count = 0;
    if (count++ % 200 == 0) { // Print occasionally
        printf("[State] Brake: %d, Err: %d, Safety: %d\n", 
            pinfo->trajState.isRobotHoldBrake, 
            pinfo->trajState.errorCode,
            pinfo->trajState.isSafetyDriving);
    }
}

int main(int argc, char** argv)
{
    const char* ip = "192.168.31.200";
    if(argc > 1) ip = argv[1];

    printf("------------------------------------------------\n");
    printf("Test DianaAPI Connection to %s\n", ip);
    printf("------------------------------------------------\n");

    // 1. 初始化结构体
    srv_net_st net_info;
    memset(&net_info, 0, sizeof(net_info));
    strcpy(net_info.SrvIp, ip);
    net_info.LocHeartbeatPort = 0;
    net_info.LocRobotStatePort = 0;
    net_info.LocSrvPort = 0;

    // 2. 连接
    printf("Calling initSrv...\n");
    int ret = initSrv(errorControl, robotState, &net_info);
    if(ret != 0) {
        printf("FATAL: initSrv failed with code %d\n", ret);
        return 1;
    }
    printf("SUCCESS: initSrv returned 0.\n");
    sleep(1); // Wait for state callback to start

    // 3. 读取初始关节角
    double joints[7];
    ret = getJointPos(joints, ip);
    printf("Initial getJointPos ret: %d\n", ret);
    printf("Joints: [%f, %f, %f, %f, %f, %f, %f]\n", 
        joints[0], joints[1], joints[2], joints[3], joints[4], joints[5], joints[6]);

    // 4. 尝试释放抱闸
    printf("\nAttempting to clean error info...\n");
    cleanErrorInfo(ip);
    
    printf("\nAttempting to release brake...\n");
    ret = releaseBrake(ip);
    printf("releaseBrake ret: %d\n", ret);
    
    printf("Waiting 5 seconds for brake release...\n");
    sleep(5);

    // Set Control Mode to Position
    printf("Setting Control Mode to T_MODE_POSITION...\n");
    ret = changeControlMode(T_MODE_POSITION, ip);
    printf("changeControlMode ret: %d\n", ret);
    sleep(1);
    
    cleanErrorInfo(ip);

    // 5. 尝试使用 moveJ 循环移动关节 7
    printf("\nAttempting to move Joint 7 using moveJ in a loop...\n");
    
    double target_joints[7];
    getJointPos(target_joints, ip);
    
    double speed = 0.5;
    double accel = 1.0;
    
    for(int i=0; i<50; i++) { // 5 seconds at 10Hz
        target_joints[6] += 0.01; // 0.01 rad per 100ms = 0.1 rad/s
        
        int ret = moveJ(target_joints, speed, accel, ip);
        
        if(i % 5 == 0) {
             double current[7];
             getJointPos(current, ip);
             printf("[moveJ %d] Tgt: %.4f, Cur: %.4f, Ret: %d\n", i, target_joints[6], current[6], ret);
        }
        usleep(100000); // 100ms
    }

    // 6. 尝试使用 servoJ_ex 进行高频控制
    printf("\nAttempting to move Joint 7 using servoJ_ex in a 100Hz loop...\n");

    getJointPos(target_joints, ip);
    double base_joint7 = target_joints[6];
    const double two_pi = 6.28318530717958647692;

    double servo_period = 0.01;      // seconds -> 100Hz
    double lookahead = 0.03;         // seconds, per API recommendation (t * 3)
    double gain = 200.0;             // dimensionless gain, tune as needed
    bool reliable = true;            // enforce reliable transmission

    for (int i = 0; i < 300; ++i) // 3 seconds @ 100Hz
    {
        // Make a small sinusoidal motion around the current position
        double delta = 0.02 * sin(two_pi * 0.2 * (i * servo_period)); // +/-0.02 rad
        target_joints[6] = base_joint7 + delta;

        int ret = servoJ_ex(target_joints, servo_period, lookahead, gain, reliable, ip);

        if (i % 50 == 0)
        {
            double current[7];
            getJointPos(current, ip);
            printf("[servoJ_ex %d] Tgt: %.4f, Cur: %.4f, Ret: %d\n", i, target_joints[6], current[6], ret);
        }

        usleep(10000); // 10ms
    }
    
    // 7. 清理
    printf("\nTest finished.\n");
    return 0;
}
