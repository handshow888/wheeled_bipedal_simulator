#ifndef QUATERNIONEKF_H
#define QUATERNIONEKF_H

#include "kalmanFilter.h"
#include "utils.h"

typedef struct
{
    uint8_t Initialized;
    KalmanFilter_t IMU_QuaternionEKF;
    uint8_t ConvergeFlag;
    uint8_t StableFlag;
    uint64_t ErrorCount;
    uint64_t UpdateCount;

    double q[4];        // 四元数估计值
    double GyroBias[3]; // 陀螺仪零偏估计值

    double Gyro[3];
    double Accel[3];

    double OrientationCosine[3];

    double accLPFcoef;
    double gyro_norm;
    double accl_norm;
    double AdaptiveGainScale;

    double Roll;
    double Pitch;
    double Yaw;

    double YawTotalAngle;

    double Q1; // 四元数更新过程噪声
    double Q2; // 陀螺仪零偏过程噪声
    double R;  // 加速度计量测噪声

    double dt; // 姿态更新周期
    Eigen::Matrix<double, 1, 1> ChiSquare;// 卡方检验检测函数
    double ChiSquareTestThreshold; // 卡方检验阈值
    double lambda;                 // 渐消因子

    int16_t YawRoundCount;

    double YawAngleLast;
} QEKF_INS_t;

extern QEKF_INS_t QEKF_INS;

void IMU_QuaternionEKF_Init(double process_noise1, double process_noise2, double measure_noise, double lambda, double lpf);
void IMU_QuaternionEKF_Update(double gx, double gy, double gz, double ax, double ay, double az, double dt);

#endif