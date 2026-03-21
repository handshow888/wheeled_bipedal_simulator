#pragma once
#include "rclcpp/rclcpp.hpp"
#include "kalmanFilter.h"
#include "QuaternionEKF.h"
#include "utils.h"

typedef struct
{
    float q[4]; // 四元数估计值

    float Gyro[3];          // 角速度
    float Accel[3];         // 加速度
    float MotionAccel_b[3]; // 机体坐标加速度
    float MotionAccel_n[3]; // 绝对系加速度

    float AccelLPF; // 加速度低通滤波系数

    // 加速度在绝对系的向量表示
    float xn[3];
    float yn[3];
    float zn[3];

    // 位姿
    float Roll;
    float Pitch;
    float Yaw;
    float YawTotalAngle;
} INS_t;

extern INS_t INS;   // imu

void INS_Init();
void INS_Task(double aX, double aY, double aZ, double gX, double gY, double gZ, double dt);

void BodyFrameToEarthFrame(const float *vecBF, float *vecEF, float *q);
void EarthFrameToBodyFrame(const float *vecEF, float *vecBF, float *q);