#ifndef KALMANFILTER_H
#define KALMANFILTER_H

#include "rclcpp/rclcpp.hpp"
#include <Eigen/Dense>
#include "utils.h"

// 状态维数 Xsize，观测维数 Zsize，控制维数 Usize
constexpr int Xsize = 6;
constexpr int Zsize = 3;
constexpr int Usize = 0; // 若有控制输入，改为 >0

typedef struct kf_t
{
    double FilteredValue[Xsize];
    double MeasuredVector[Zsize];
    // double ControlVector[Usize];

    // uint8_t xhatSize = (uint8_t)Xsize;
    // uint8_t uSize = (uint8_t)Usize;
    // uint8_t zSize = (uint8_t)Zsize;

    // uint8_t MeasurementMap[Zsize];      // 量测与状态的关系 how measurement relates to the state
    // double MeasurementDegree[Zsize];     // 测量值对应H矩阵元素值 elements of each measurement in H
    // double MatR_DiagonalElements[Zsize]; // 量测方差 variance for each measurement
    double StateMinVariance[Xsize]; // 最小方差 避免方差过度收敛 suppress filter excessive convergence
    // uint8_t temp[Zsize];

    // 配合用户定义函数使用,作为标志位用于判断是否要跳过标准KF中五个环节中的任意一个
    uint8_t SkipEq1, SkipEq2, SkipEq3, SkipEq4, SkipEq5;

    // definiion of struct mat: rows & cols & pointer to vars
    Eigen::Matrix<double, Xsize, 1> xhat = Eigen::Matrix<double, Xsize, 1>::Zero(Xsize, 1);           // x(k|k)
    Eigen::Matrix<double, Xsize, 1> xhatminus = Eigen::Matrix<double, Xsize, 1>::Zero(Xsize, 1);      // x(k|k-1)
    Eigen::Matrix<double, Usize, 1> u = Eigen::Matrix<double, Usize, 1>::Zero(Usize, 1);              // control vector u
    Eigen::Matrix<double, Zsize, 1> z = Eigen::Matrix<double, Zsize, 1>::Zero(Zsize, 1);              // measurement vector z
    Eigen::Matrix<double, Xsize, Xsize> P = Eigen::Matrix<double, Xsize, Xsize>::Identity(Xsize, Xsize);      // covariance matrix P(k|k)
    Eigen::Matrix<double, Xsize, Xsize> Pminus = Eigen::Matrix<double, Xsize, Xsize>::Identity(Xsize, Xsize); // covariance matrix P(k|k-1)
    Eigen::Matrix<double, Xsize, Xsize> F = Eigen::Matrix<double, Xsize, Xsize>::Identity(Xsize, Xsize);      // sUsizetate transition matrix F FT
    Eigen::Matrix<double, Xsize, Xsize> FT = Eigen::Matrix<double, Xsize, Xsize>::Identity(Xsize, Xsize);     // state transition matrix F FT
    Eigen::Matrix<double, Xsize, Usize> B = Eigen::Matrix<double, Xsize, Usize>::Zero(Xsize, Usize);          // control matrix B
    Eigen::Matrix<double, Zsize, Xsize> H = Eigen::Matrix<double, Zsize, Xsize>::Zero(Zsize, Xsize);          // measurement matrix H
    Eigen::Matrix<double, Xsize, Zsize> HT = Eigen::Matrix<double, Xsize, Zsize>::Zero(Xsize, Zsize);
    Eigen::Matrix<double, Xsize, Xsize> Q = Eigen::Matrix<double, Xsize, Xsize>::Identity(Xsize, Xsize); // process noise covariance matrix Q
    Eigen::Matrix<double, Zsize, Zsize> R = Eigen::Matrix<double, Zsize, Zsize>::Identity(Zsize, Zsize); // measurement noise covariance matrix R
    Eigen::Matrix<double, Xsize, Zsize> K = Eigen::Matrix<double, Xsize, Zsize>::Zero(Xsize, Zsize);     // kalman gain  K
    Eigen::Matrix<double, Zsize, Zsize> S = Eigen::Matrix<double, Zsize, Zsize>::Zero(Zsize, Zsize);

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
    // double *xhat_data, *xhatminus_data;
    // double *u_data;
    // double *z_data;
    // double *P_data, *Pminus_data;
    // double *F_data, *FT_data;
    // double *B_data;
    // double *H_data, *HT_data;
    // double *Q_data;
    // double *R_data;
    // double *K_data;
    // double *S_data;
} KalmanFilter_t;

void Kalman_Filter_Init(KalmanFilter_t *kf);
void Kalman_Filter_Measure(KalmanFilter_t *kf);
void Kalman_Filter_xhatMinusUpdate(KalmanFilter_t *kf);
void Kalman_Filter_PminusUpdate(KalmanFilter_t *kf);
void Kalman_Filter_SetK(KalmanFilter_t *kf);
void Kalman_Filter_xhatUpdate(KalmanFilter_t *kf);
void Kalman_Filter_P_Update(KalmanFilter_t *kf);
double *Kalman_Filter_Update(KalmanFilter_t *kf);


#endif