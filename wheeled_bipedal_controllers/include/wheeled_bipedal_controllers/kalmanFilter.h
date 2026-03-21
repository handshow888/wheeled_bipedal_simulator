#pragma once
#include "rclcpp/rclcpp.hpp"
#include <Eigen/Dense>
#include "utils.h"

// 状态维数 Xsize，观测维数 Zsize，控制维数 Usize
constexpr int Xsize = 6;
constexpr int Zsize = 3;
constexpr int Usize = 0; // 若有控制输入，改为 >0

typedef struct kf_t
{
    float FilteredValue[Xsize];
    float MeasuredVector[Zsize];
    // float ControlVector[Usize];

    // uint8_t xhatSize = (uint8_t)Xsize;
    // uint8_t uSize = (uint8_t)Usize;
    // uint8_t zSize = (uint8_t)Zsize;

    // uint8_t MeasurementMap[Zsize];      // 量测与状态的关系 how measurement relates to the state
    // float MeasurementDegree[Zsize];     // 测量值对应H矩阵元素值 elements of each measurement in H
    // float MatR_DiagonalElements[Zsize]; // 量测方差 variance for each measurement
    float StateMinVariance[Xsize]; // 最小方差 避免方差过度收敛 suppress filter excessive convergence
    // uint8_t temp[Zsize];

    // 配合用户定义函数使用,作为标志位用于判断是否要跳过标准KF中五个环节中的任意一个
    uint8_t SkipEq1, SkipEq2, SkipEq3, SkipEq4, SkipEq5;

    // definiion of struct mat: rows & cols & pointer to vars
    Eigen::MatrixXd xhat = Eigen::MatrixXd::Zero(Xsize, 1);           // x(k|k)
    Eigen::MatrixXd xhatminus = Eigen::MatrixXd::Zero(Xsize, 1);      // x(k|k-1)
    Eigen::MatrixXd u = Eigen::MatrixXd::Zero(Usize, 1);              // control vector u
    Eigen::MatrixXd z = Eigen::MatrixXd::Zero(Zsize, 1);              // measurement vector z
    Eigen::MatrixXd P = Eigen::MatrixXd::Identity(Xsize, Xsize);      // covariance matrix P(k|k)
    Eigen::MatrixXd Pminus = Eigen::MatrixXd::Identity(Xsize, Xsize); // covariance matrix P(k|k-1)
    Eigen::MatrixXd F = Eigen::MatrixXd::Identity(Xsize, Xsize);      // state transition matrix F FT
    Eigen::MatrixXd FT = Eigen::MatrixXd::Identity(Xsize, Xsize);     // state transition matrix F FT
    Eigen::MatrixXd B = Eigen::MatrixXd::Zero(Xsize, Usize);          // control matrix B
    Eigen::MatrixXd H = Eigen::MatrixXd::Zero(Zsize, Xsize);          // measurement matrix H
    Eigen::MatrixXd HT = Eigen::MatrixXd::Zero(Xsize, Zsize);
    Eigen::MatrixXd Q = Eigen::MatrixXd::Identity(Xsize, Xsize); // process noise covariance matrix Q
    Eigen::MatrixXd R = Eigen::MatrixXd::Identity(Zsize, Zsize); // measurement noise covariance matrix R
    Eigen::MatrixXd K = Eigen::MatrixXd::Zero(Xsize, Zsize);     // kalman gain  K
    Eigen::MatrixXd S = Eigen::MatrixXd::Zero(Zsize, Zsize);

    int8_t MatStatus;

    // 用户定义函数,可以替换或扩展基准KF的功能
    void (*User_Func0_f)(struct kf_t *kf) = nullptr;
    void (*User_Func1_f)(struct kf_t *kf) = nullptr;
    void (*User_Func2_f)(struct kf_t *kf) = nullptr;
    void (*User_Func3_f)(struct kf_t *kf) = nullptr;
    void (*User_Func4_f)(struct kf_t *kf) = nullptr;
    void (*User_Func5_f)(struct kf_t *kf) = nullptr;
    void (*User_Func6_f)(struct kf_t *kf) = nullptr;

    // // 矩阵存储空间指针
    // float *xhat_data, *xhatminus_data;
    // float *u_data;
    // float *z_data;
    // float *P_data, *Pminus_data;
    // float *F_data, *FT_data;
    // float *B_data;
    // float *H_data, *HT_data;
    // float *Q_data;
    // float *R_data;
    // float *K_data;
    // float *S_data;
} KalmanFilter_t;

void Kalman_Filter_Init(KalmanFilter_t *kf);
void Kalman_Filter_Measure(KalmanFilter_t *kf);
void Kalman_Filter_xhatMinusUpdate(KalmanFilter_t *kf);
void Kalman_Filter_PminusUpdate(KalmanFilter_t *kf);
void Kalman_Filter_SetK(KalmanFilter_t *kf);
void Kalman_Filter_xhatUpdate(KalmanFilter_t *kf);
void Kalman_Filter_P_Update(KalmanFilter_t *kf);
float *Kalman_Filter_Update(KalmanFilter_t *kf);