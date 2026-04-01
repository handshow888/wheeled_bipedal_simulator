#ifndef PID_CONTROLLER_HPP
#define PID_CONTROLLER_HPP

class PIDController
{
private:
    double P_;              // 比例系数
    double I_;              // 积分系数
    double D_;              // 微分系数
    double previousError_; // 上一次的误差
    double integral_;       // 误差的积分
    bool setParams_;
public:
    PIDController();
    PIDController(double p, double i, double d);

    void setParams(double p, double i, double d);

    double compute(double targetValue, double nowValue, double dt);

    double clear();
};

#endif