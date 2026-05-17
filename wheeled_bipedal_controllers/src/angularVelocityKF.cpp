#include "wheeled_bipedal_controllers/angularVelocityKF.hpp"

AngularVelocityKF::AngularVelocityKF()
{
    x_.setZero();
    P_.setIdentity();

    H_ << 1.0, 0.0,
        1.0, 0.0;

    R_.setIdentity();

    sigma_process_ = 1.0;
    initialized_ = false;
}

/**
 * @brief 初始化滤波器
 * @param omega0 初始角速度，单位 rad/s
 * @param alpha0 初始角加速度，单位 rad/s^2
 * @param var_omega0 初始角速度估计方差
 * @param var_alpha0 初始角加速度估计方差
 */
void AngularVelocityKF::init(double omega0, double alpha0,
          double var_omega0, double var_alpha0)
{
    x_ << omega0, alpha0;

    P_.setZero();
    P_(0, 0) = var_omega0;
    P_(1, 1) = var_alpha0;

    initialized_ = true;
}

/**
 * @brief 设置过程噪声方差
 *
 * 这里对应模型中的 Q = sigma^2。
 * 过程噪声反映角加速度变化的不确定性。
 * 值越大，滤波器越容易跟随快速变化的角速度。
 */
void AngularVelocityKF::setProcessNoiseVariance(double sigma_process)
{
    sigma_process_ = sigma_process;
}

/**
 * @brief 设置量测噪声方差
 * @param sigma_wheel_yaw2 轮速解算 yaw 角速度的噪声方差
 * @param sigma_gyro_z2 陀螺仪 z 轴角速度的噪声方差
 */
void AngularVelocityKF::setMeasurementNoiseVariance(double sigma_wheel_yaw2,
                                 double sigma_gyro_z2)
{
    R_.setZero();
    R_(0, 0) = sigma_wheel_yaw2;
    R_(1, 1) = sigma_gyro_z2;
}

/**
 * @brief 预测步骤
 * @param dt 采样时间间隔，单位 s
 */
void AngularVelocityKF::predict(double dt)
{
    if (!initialized_)
    {
        return;
    }

    Eigen::Matrix2d F;
    F << 1.0, dt,
        0.0, 1.0;

    Eigen::Vector2d Gamma;
    Gamma << 0.5 * dt * dt,
        dt;

    // 状态预测
    x_ = F * x_;

    // 协方差预测
    P_ = F * P_ * F.transpose() + Gamma * sigma_process_ * Gamma.transpose();

    // 保持协方差矩阵对称
    P_ = 0.5 * (P_ + P_.transpose());
}

/**
 * @brief 更新步骤
 * @param omega_wheel_yaw 轮速解算得到的 yaw 角速度，单位 rad/s
 * @param omega_gyro_z 陀螺仪 z 轴角速度，单位 rad/s
 */
void AngularVelocityKF::update(double omega_wheel_yaw, double omega_gyro_z)
{
    if (!initialized_)
    {
        // 初始角速度可以取两个量测的平均值
        double omega0 = 0.5 * (omega_wheel_yaw + omega_gyro_z);
        init(omega0, 0.0, 1.0, 10.0);
        return;
    }

    Eigen::Vector2d z;
    z << omega_wheel_yaw,
        omega_gyro_z;

    // 残差 y = z - Hx
    Eigen::Vector2d y = z - H_ * x_;

    // 残差协方差 S = HPH^T + R
    Eigen::Matrix2d S = H_ * P_ * H_.transpose() + R_;

    // 卡尔曼增益 K = PH^T S^-1
    Eigen::Matrix2d K = S.ldlt().solve(H_ * P_).transpose();

    // 状态更新
    x_ = x_ + K * y;

    // Joseph 形式更新协方差，数值稳定性更好
    Eigen::Matrix2d I = Eigen::Matrix2d::Identity();
    Eigen::Matrix2d KH = K * H_;

    P_ = (I - KH) * P_ * (I - KH).transpose() + K * R_ * K.transpose();

    P_ = 0.5 * (P_ + P_.transpose());
}

double AngularVelocityKF::angularVelocity() const
{
    return x_(0);
}

double AngularVelocityKF::angularAcceleration() const
{
    return x_(1);
}

Eigen::Vector2d AngularVelocityKF::state() const
{
    return x_;
}

Eigen::Matrix2d AngularVelocityKF::covariance() const
{
    return P_;
}