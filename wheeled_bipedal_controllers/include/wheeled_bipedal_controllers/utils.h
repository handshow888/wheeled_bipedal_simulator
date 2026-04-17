#ifndef UTILS_H
#define UTILS_H

#include "rclcpp/rclcpp.hpp"
#include <cmath>
#include <Eigen/Dense>

#define clamp(amt, low, high) ((amt) < (low) ? (low) : ((amt) > (high) ? (high) : (amt))) // 限幅函数

double lowPassFilter(double currentValue, double previousValue, double alpha);
void EularAngleToQuaternion(double Yaw, double Pitch, double Roll, double *q);
void QuaternionToEularAngle(double *q, double *Yaw, double *Pitch, double *Roll);
void wrapToPi(double &radian);

template <int Rows, int Cols>
void matrixToRowMajorFixedArray(const Eigen::Matrix<double, Rows, Cols> &mat, double *output)
{
    // 将 output 映射为固定尺寸行优先矩阵，直接赋值
    Eigen::Map<Eigen::Matrix<double, Rows, Cols, Eigen::RowMajor>>{output} = mat;
}

template <int N>
void vectorToFixedArray(const Eigen::Matrix<double, N, 1> &vec, double *output)
{
    Eigen::Map<Eigen::Matrix<double, N, 1>>{output} = vec;
}

constexpr double deg2rad = M_PI / 180.0; // 乘以此变量可从角度变换到弧度
constexpr double rad2deg = 180.0 / M_PI; // 乘以此变量可从弧度变换到角度

#endif