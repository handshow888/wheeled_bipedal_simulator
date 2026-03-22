#ifndef UTILS_H
#define UTILS_H

#include "rclcpp/rclcpp.hpp"
#include <cmath>
#include <Eigen/Dense>

#define clamp(amt, low, high) ((amt) < (low) ? (low) : ((amt) > (high) ? (high) : (amt))) // 限幅函数

float lowPassFilter(float currentValue, float previousValue, float alpha);
float invSqrt(float x);
void EularAngleToQuaternion(float Yaw, float Pitch, float Roll, float *q);
void QuaternionToEularAngle(float *q, float *Yaw, float *Pitch, float *Roll);

Eigen::MatrixXd arrayToMatrixXd(const float* data, int rows, int cols, bool rowMajor = true);
void matrixToFloatArray(const Eigen::MatrixXd& vec, float* output);
void matrixToRowMajorFloatArray(const Eigen::MatrixXd& mat, float* output);


constexpr float deg2rad = M_PI / 180.0f;  // 乘以此变量可从角度变换到弧度
constexpr float rad2deg = 180.0f / M_PI;// 乘以此变量可从弧度变换到角度

#endif