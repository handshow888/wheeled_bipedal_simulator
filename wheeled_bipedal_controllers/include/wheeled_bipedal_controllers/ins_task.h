#ifndef INS_TASK_H
#define INS_TASK_H

#include "rclcpp/rclcpp.hpp"
#include "kalmanFilter.h"
#include "QuaternionEKF.h"
#include "utils.h"

typedef struct
{
    double q[4]; // 四元数估计值

    double Gyro[3];          // 角速度
    double Accel[3];         // 加速度
    double MotionAccel_b[3]; // 机体坐标加速度
    double MotionAccel_n[3]; // 绝对系加速度

    double AccelLPF; // 加速度低通滤波系数

    // 加速度在绝对系的向量表示
    double xn[3];
    double yn[3];
    double zn[3];

    // 位姿
    double Roll;
    double Pitch;
    double Yaw;
    double YawTotalAngle;
} INS_t;

extern INS_t INS;   // imu

void INS_Init();
void INS_Task(double aX, double aY, double aZ, double gX, double gY, double gZ, double dt);

void BodyFrameToEarthFrame(const double *vecBF, double *vecEF, double *q);
void EarthFrameToBodyFrame(const double *vecEF, double *vecBF, double *q);

#endif