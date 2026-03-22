#include "wheeled_bipedal_controllers/kinematics.h"
#include "rclcpp/rclcpp.hpp"

namespace kinematics
{

    /**
     * @brief 运动学正解(关节电机角度解出足端坐标)
     */
    fwdKinematicsResult forwardKinematics(float phi1, float phi4)
    {
        fwdKinematicsResult result;

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

        // if (!(phi2 >= 0 && phi2 <= M_PI / 2))
        // {
        //     phi2 = 2 * atan((b - sqrt(a * a + b * b - c * c)) / (a + c));
        // }

        result.wheelPos.x = L1 * cos(phi1) + L2 * cos(phi2);
        result.wheelPos.y = L1 * sin(phi1) + L2 * sin(phi2);

        result.L0 = sqrt((result.wheelPos.x - (L5 / 2)) * (result.wheelPos.x - (L5 / 2)) + result.wheelPos.y * result.wheelPos.y);
        result.phi0 = atan2(result.wheelPos.y, result.wheelPos.x - (L5 / 2));
        result.phi1 = atan2(result.wheelPos.y - y_b, result.wheelPos.x - x_b);
        result.phi2 = phi2;
        result.phi3 = atan2(result.wheelPos.y - y_d, result.wheelPos.x - x_d);
        result.phi4 = atan2(y_d, x_d - L5);

        // RCLCPP_INFO(rclcpp::get_logger("Kinematics"), "phi1:%.2f phi4:%.2f x: %.2f y:%.2f phi2:%.2f",
        //             phi1, phi4,
        //             result.wheelPos.x, result.wheelPos.y,
        //             phi2);
        // RCLCPP_INFO(rclcpp::get_logger("Kinematics"), "x: %.2f y:%.2f L0:%.2f phi0:%.2f",
        //             result.wheelPos.x, result.wheelPos.y,
        //             result.L0,
        //             result.phi0);

        return result;
    }

} // namespace kinematics