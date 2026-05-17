#ifndef ANGULAR_VELOCITY_KF_HPP
#define ANGULAR_VELOCITY_KF_HPP

#include <iostream>
#include <Eigen/Dense>

class AngularVelocityKF
{
public:
    AngularVelocityKF();
    /**
     * @brief 初始化滤波器
     * @param omega0 初始角速度，单位 rad/s
     * @param alpha0 初始角加速度，单位 rad/s^2
     * @param var_omega0 初始角速度估计方差
     * @param var_alpha0 初始角加速度估计方差
     */
    void init(double omega0, double alpha0, double var_omega0, double var_alpha0);

    /**
     * @brief 设置过程噪声方差
     *
     * 这里对应模型中的 Q = sigma^2。
     * 过程噪声反映角加速度变化的不确定性。
     * 值越大，滤波器越容易跟随快速变化的角速度。
     */
    void setProcessNoiseVariance(double sigma_process);

    /**
     * @brief 设置量测噪声方差
     * @param sigma_wheel_yaw2 轮速解算 yaw 角速度的噪声方差
     * @param sigma_gyro_z2 陀螺仪 z 轴角速度的噪声方差
     */
    void setMeasurementNoiseVariance(double sigma_wheel_yaw2, double sigma_gyro_z2);
    /**
     * @brief 预测步骤
     * @param dt 采样时间间隔，单位 s
     */
    void predict(double dt);

    /**
     * @brief 更新步骤
     * @param omega_wheel_yaw 轮速解算得到的 yaw 角速度，单位 rad/s
     * @param omega_gyro_z 陀螺仪 z 轴角速度，单位 rad/s
     */
    void update(double omega_wheel_yaw, double omega_gyro_z);

    double angularVelocity() const;

    double angularAcceleration() const;

    Eigen::Vector2d state() const;

    Eigen::Matrix2d covariance() const;

private:
    Eigen::Vector2d x_; // x = [omega_z, alpha_z]^T
    Eigen::Matrix2d P_; // 状态协方差

    Eigen::Matrix2d H_; // 量测矩阵
    Eigen::Matrix2d R_; // 量测噪声协方差

    double sigma_process_; // 过程噪声方差

    bool initialized_;
};

#endif