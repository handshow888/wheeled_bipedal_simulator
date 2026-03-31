#include "wheeled_bipedal_controllers/pid_controller.hpp"

PIDController::PIDController() : setParams_(false)
{
    P_ = 0.0;
    I_ = 0.0;
    D_ = 0.0;
    previousError_ = 0.0;
    integral_ = 0.0;
}

PIDController::PIDController(double p, double i, double d) : P_(p), I_(i), D_(d), previousError_(0.0), integral_(0.0), setParams_(true)
{
}

void PIDController::setParams(double p, double i, double d)
{
    if (p == P_ && i == I_ && d == D_)
        return;
    P_ = p;
    I_ = i;
    D_ = d;
    previousError_ = 0.0;
    integral_ = 0.0;
    setParams_ = true;
}

double PIDController::compute(double targetValue, double nowValue, double dt)
{
    if (!setParams_)
        return 0.0;

    double error = targetValue - nowValue;                               // 计算当前误差
    integral_ += error * dt;                                             // 更新积分项
    double derivative = (error - previousError_) / dt;                   // 计算微分项
    double output = (P_ * error) + (I_ * integral_) + (D_ * derivative); // PID公式计算输出
    previousError_ = error;                                              // 更新上一次误差
    return output;
}