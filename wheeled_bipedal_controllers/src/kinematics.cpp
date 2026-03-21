#include "wheeled_bipedal_controllers/kinematics.h"
#include "rclcpp/rclcpp.hpp"

/**
 * @brief 运动学正解(关节电机角度解出足端坐标)
 */
Point forwardKinematics(float phi1, float phi4)
{
    Point result;

    float x_b = L1 * cos(phi1);
    float y_b = L1 * sin(phi1);
    float x_d = L5 + L4 * cos(phi4);
    float y_d = L4 * sin(phi4);

    float a = 2 * (x_d - x_b) * L2;
    float b = 2 * (y_d - y_b) * L2;
    float L_BD = sqrt((x_d - x_b) * (x_d - x_b) + (y_d - y_b) * (y_d - y_b));
    float c = -L3 * L3 + L2 * L2 + L_BD * L_BD;
    // phi2 ∈ [0, pi/2]
    float phi2 = 2 * atan2((b + sqrt(a * a + b * b - c * c)), (a + c));
    // if (phi2 >= M_PI)
    //     phi2 -= M_PI * 2;
    // else if (phi2 <= -M_PI)
    //     phi2 += M_PI * 2;
    // phi2 = wrapTo2Pi(phi2);

    // if (!(phi2 >= 0 && phi2 <= M_PI / 2))
    // {
    //     phi2 = 2 * atan((b - sqrt(a * a + b * b - c * c)) / (a + c));
    //     phi2 = wrapTo2Pi(phi2);
    // }

    result.x = L1 * cos(phi1) + L2 * cos(phi2);
    result.y = L1 * sin(phi1) + L2 * sin(phi2);

    // RCLCPP_INFO(rclcpp::get_logger("Kinematics"), "phi1:%.2f phi4:%.2f x: %.2f y:%.2f phi2:%.2f",
    //             phi1, phi4,
    //             result.x, result.y,
    //             phi2);

    return result;
}

/**
 * @brief 把角度限制在[0,2PI]
 */
inline float wrapTo2Pi(float radian)
{
    return (radian >= 0.0f) ? radian : (radian + 2 * M_PI);
}