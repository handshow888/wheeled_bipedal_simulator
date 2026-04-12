#include "wheeled_bipedal_controllers/kinematics.h"
#include "rclcpp/rclcpp.hpp"

namespace kinematics
{

    /**
     * @brief 运动学正解(关节电机角度解出足端坐标)
     */
    fwdKinematicsResult forwardKinematics(double phi1, double phi4)
    {
        fwdKinematicsResult result;

        double x_b = L1 * cos(phi1);
        double y_b = L1 * sin(phi1);
        double x_d = L5 + L4 * cos(phi4);
        double y_d = L4 * sin(phi4);

        double a = 2 * (x_d - x_b) * L2;
        double b = 2 * (y_d - y_b) * L2;
        double L_BD = sqrt((x_d - x_b) * (x_d - x_b) + (y_d - y_b) * (y_d - y_b));
        double c = -L3 * L3 + L2 * L2 + L_BD * L_BD;
        // phi2 ∈ [0, pi/2]
        double phi2 = 2 * atan2((b + sqrt(a * a + b * b - c * c)), (a + c));
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
        result.phi1 = atan2(y_b, x_b);
        result.phi2 = phi2;
        result.phi3 = atan2(result.wheelPos.y - y_d, result.wheelPos.x - x_d);
        result.phi4 = atan2(y_d, x_d - L5);

        // RCLCPP_INFO(rclcpp::get_logger("Kinematics"), "phi0:%.2f phi1:%.2f phi2:%.2f phi3:%.2f phi4:%.2f",
        //             result.phi0 * rad2deg, result.phi1 * rad2deg, result.phi2 * rad2deg, result.phi3 * rad2deg, result.phi4 * rad2deg);

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

    void inverseKinematics(point wheelPos, double &phi1, double &phi4)
    {
        // 解算alpha用
        double a = 2 * wheelPos.x * L1;
        double b = 2 * wheelPos.y * L1;
        double c = wheelPos.x * wheelPos.x + wheelPos.y * wheelPos.y + L1 * L1 - L2 * L2;
        // 解算beta用
        double d = 2 * L4 * (wheelPos.x - L5);
        double e = 2 * L4 * wheelPos.y;
        double f = ((wheelPos.x - L5) * (wheelPos.x - L5) + L4 * L4 + wheelPos.y * wheelPos.y - L3 * L3);
        phi1 = 2 * atan2((b + sqrt(a * a + b * b - c * c)), (a + c));
        phi4 = 2 * atan2((e + sqrt(d * d + e * e - f * f)), (d + f));
        phi1 = wrapTo2Pi(phi1); // phi1必须为大于 pi/2 的角度，仿真中才不会出现旋转方向出现干涉问题
        phi4 = wrapTo2Pi(phi4);

        // 如果解不在合理范围内，计算第二个解
        if (!(phi1 >= M_PI / 2 && phi1 <= (M_PI + M_PI / 2)))
        {
            RCLCPP_INFO(rclcpp::get_logger("Kinematics"), "phi1 1: %.2f invalid\t", phi1 / M_PI * 180.0);
            phi1 = 2 * atan2((b - sqrt(a * a + b * b - c * c)), (a + c));
            phi1 = wrapTo2Pi(phi1);
        }
        if (!(phi4 >= -M_PI / 2 && phi4 <= M_PI / 2))
        {
            RCLCPP_INFO(rclcpp::get_logger("Kinematics"), "phi4 1: %.2f invalid\t", phi4 / M_PI * 180.0);
            phi4 = 2 * atan2((e - sqrt(d * d + e * e - f * f)), (d + f));
        }

        // phi4 必须在正负pi/2才不会干涉
        if (phi4 >= M_PI)
            phi4 -= M_PI * 2;
        else if (phi4 <= -M_PI)
            phi4 += M_PI * 2;

        // RCLCPP_INFO(rclcpp::get_logger("Kinematics"), "phi1: %.2f phi4:%.2f", phi1, phi4);
    }

    /**
     * @brief 把角度限制在[0,2PI]
     */
    inline double wrapTo2Pi(double radian)
    {
        return (radian >= 0.0) ? radian : (radian + 2 * M_PI);
    }

} // namespace kinematics