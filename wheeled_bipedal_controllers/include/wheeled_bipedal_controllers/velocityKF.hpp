#ifndef VELOCITY_KF_HPP
#define VELOCITY_KF_HPP

#include <iostream>
#include <Eigen/Dense>

class VelocityAccelerationKF
{
public:
    VelocityAccelerationKF();

    /**
     * @brief 初始化滤波器
     * @param v0 初始速度
     * @param a0 初始加速度
     * @param var_v0 初始速度估计方差
     * @param var_a0 初始加速度估计方差
     */
    void init(double v0, double a0, double var_v0, double var_a0);

    /**
     * @brief 设置过程噪声方差 Q = sigma^2
     *
     * 这里的 sigma_w2 对应公式中的 Q_k = sigma^2。
     * 由于 Gamma = [0.5*dt^2, dt]^T，
     * 实际加入 P 的过程噪声为 Gamma * sigma_w2 * Gamma^T。
     */
    void setProcessNoiseVariance(double sigma_w2);

    /**
     * @brief 设置量测噪声协方差 R
     * @param sigma_v2 速度测量噪声方差
     * @param sigma_a2 加速度测量噪声方差
     */
    void setMeasurementNoiseVariance(double sigma_v2, double sigma_a2);
    /**
     * @brief 预测步骤
     * @param dt 采样时间间隔，单位 s
     */
    void predict(double dt);

    /**
     * @brief 更新步骤
     * @param v_meas 速度量测值
     * @param a_meas 加速度量测值
     */
    void update(double v_meas, double a_meas);

    double velocity() const;
    double acceleration() const;
    Eigen::Vector2d state() const;

    Eigen::Matrix2d covariance() const;

private:
    Eigen::Vector2d x_; // 状态 x = [v, a]^T
    Eigen::Matrix2d P_; // 状态协方差 P

    Eigen::Matrix2d H_; // 量测矩阵 H
    Eigen::Matrix2d R_; // 量测噪声协方差 R

    double sigma_w2_; // 过程噪声方差 Q = sigma^2

    bool initialized_;
};

#endif