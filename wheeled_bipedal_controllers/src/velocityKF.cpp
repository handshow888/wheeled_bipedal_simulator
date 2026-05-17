#include "wheeled_bipedal_controllers/velocityKF.hpp"

VelocityAccelerationKF::VelocityAccelerationKF()
{
    x_.setZero();     // x = [v, a]^T
    P_.setIdentity(); // 状态协方差

    H_.setIdentity(); // H = I
    R_.setIdentity(); // 量测噪声协方差

    sigma_w2_ = 1.0; // 过程噪声方差 sigma^2
    initialized_ = false;
}

/**
 * @brief 初始化滤波器
 * @param v0 初始速度
 * @param a0 初始加速度
 * @param var_v0 初始速度估计方差
 * @param var_a0 初始加速度估计方差
 */
void VelocityAccelerationKF::init(double v0, double a0, double var_v0, double var_a0)
{
    x_ << v0, a0;

    P_.setZero();
    P_(0, 0) = var_v0;
    P_(1, 1) = var_a0;

    initialized_ = true;
}

/**
 * @brief 设置过程噪声方差 Q = sigma^2
 *
 * 这里的 sigma_w2 对应公式中的 Q_k = sigma^2。
 * 由于 Gamma = [0.5*dt^2, dt]^T，
 * 实际加入 P 的过程噪声为 Gamma * sigma_w2 * Gamma^T。
 */
void VelocityAccelerationKF::setProcessNoiseVariance(double sigma_w2)
{
    sigma_w2_ = sigma_w2;
}

/**
 * @brief 设置量测噪声协方差 R
 * @param sigma_v2 速度测量噪声方差
 * @param sigma_a2 加速度测量噪声方差
 */
void VelocityAccelerationKF::setMeasurementNoiseVariance(double sigma_v2, double sigma_a2)
{
    R_.setZero();
    R_(0, 0) = sigma_v2;
    R_(1, 1) = sigma_a2;
}

/**
 * @brief 预测步骤
 * @param dt 采样时间间隔，单位 s
 */
void VelocityAccelerationKF::predict(double dt)
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
    P_ = F * P_ * F.transpose() + Gamma * sigma_w2_ * Gamma.transpose();

    // 保持协方差矩阵对称，减少数值误差
    P_ = 0.5 * (P_ + P_.transpose());
}

/**
 * @brief 更新步骤
 * @param v_meas 速度量测值
 * @param a_meas 加速度量测值
 */
void VelocityAccelerationKF::update(double v_meas, double a_meas)
{
    if (!initialized_)
    {
        init(v_meas, a_meas, 1.0, 1.0);
        return;
    }

    Eigen::Vector2d z;
    z << v_meas, a_meas;

    // 残差 y = z - Hx
    Eigen::Vector2d y = z - H_ * x_;

    // 残差协方差 S = HPH^T + R
    Eigen::Matrix2d S = H_ * P_ * H_.transpose() + R_;

    // 卡尔曼增益 K = P H^T S^-1
    // 这里不用 inverse()，数值稳定性更好
    Eigen::Matrix2d K = S.ldlt().solve(H_ * P_).transpose();

    // 状态更新
    x_ = x_ + K * y;

    // 协方差更新，采用 Joseph 形式，数值稳定性更好
    Eigen::Matrix2d I = Eigen::Matrix2d::Identity();
    Eigen::Matrix2d KH = K * H_;
    P_ = (I - KH) * P_ * (I - KH).transpose() + K * R_ * K.transpose();

    // 保持协方差矩阵对称
    P_ = 0.5 * (P_ + P_.transpose());
}

double VelocityAccelerationKF::velocity() const
{
    return x_(0);
}

double VelocityAccelerationKF::acceleration() const
{
    return x_(1);
}

Eigen::Vector2d VelocityAccelerationKF::state() const
{
    return x_;
}

Eigen::Matrix2d VelocityAccelerationKF::covariance() const
{
    return P_;
}