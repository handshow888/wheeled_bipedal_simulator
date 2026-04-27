#ifndef WHEELED_BIPEDAL_CONTROLLER_HPP
#define WHEELED_BIPEDAL_CONTROLLER_HPP

#include <chrono>
#include <ctime>
#include "rclcpp/rclcpp.hpp"
#include "controller_interface/controller_interface.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

#include "wheeled_bipedal_controllers/ins_task.h"
#include "wheeled_bipedal_controllers/kinematics.h"
#include "wheeled_bipedal_controllers/LQR.h"
#include "wheeled_bipedal_controllers/VMC.h"
#include "wheeled_bipedal_controllers/structural_params.h"
#include "wheeled_bipedal_controllers/pid_controller.hpp"

namespace wheeled_bipedal_controller
{
    struct motorStates
    {
        double position;
        double velocity;
        double effort;
    };

    struct imuStates
    {
        double ang_vel_x;
        double ang_vel_y;
        double ang_vel_z;
        double lin_acc_x;
        double lin_acc_y;
        double lin_acc_z;
    };

    enum class commandType : u_int8_t
    {
        position = 0,
        velocity = 1,
        effort = 2
    };

    class WheeledBipedalController : public controller_interface::ControllerInterface
    {
    public:
        WheeledBipedalController();

        controller_interface::InterfaceConfiguration command_interface_configuration() const override;
        controller_interface::InterfaceConfiguration state_interface_configuration() const override;

        controller_interface::CallbackReturn on_init() override;
        controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State &previous_state) override;
        controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State &previous_state) override;

        controller_interface::return_type update(const rclcpp::Time &time, const rclcpp::Duration &period) override;

        // 自定义函数
        void loadStates();
        void joyCB(sensor_msgs::msg::Joy::SharedPtr msg);

    private:
        rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joySub_;
        // rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr testPointSub_;
        // std::vector<double> iKMotorPosTarget_;
        rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr testInfoPub_;
        rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmdVelSub_;
        double recCmdVelTime_ = 0.0;
        geometry_msgs::msg::Twist recCmdVel_;

        PIDController angularVelPID; // 角速度控制

        // 腿高控制
        PIDController leftLegLengthPID, rightLegLengthPID;
        double legLengthFeedforward_;
        double leftLegLengthTarget_, rightLegLengthTarget_;
        bool rightLegSameWithLeft_;

        // roll角误差控制
        PIDController rollErrPID;

        PIDController deltaPhi0PID; // 防劈叉控制

        bool debug_; // 是否在update函数中实时获取动态参数

        double pitchCompensation = 0.0;

        // ros2_control 参数
        std::vector<std::string> joint_names_, state_interface_names_;
        std::string command_interface_name_, imu_name_;
        std::vector<double> joints_bias_values_ = {0.0, 0.0, 0.0, 0.0}; // 关节电机实际角度与tf角度偏差（rad）
        double joint_command_limit_ = 10.0;
        commandType joint_control_type = commandType::effort;
        motorStates lfMotorStates_, lrMotorStates_, rfMotorStates_, rrMotorStates_, lwMotorStates_, rwMotorStates_; // 存储各电机状态值
        imuStates imuStates_;
        // 用于保存 motor 各个状态接口的索引
        std::vector<size_t> lf_motor_state_indices_, lr_motor_state_indices_, rf_motor_state_indices_, rr_motor_state_indices_; // 关节电机
        std::vector<size_t> lw_motor_state_indices_, rw_motor_state_indices_;                                                   // 轮毂电机
        // IMU 索引缓存
        std::vector<size_t> imu_state_indices_;
        // 预定义 IMU 的 6 个状态接口名称
        const std::vector<std::string> imu_interface_names_ = {
            "angular_velocity.x",
            "angular_velocity.y",
            "angular_velocity.z",
            "linear_acceleration.x",
            "linear_acceleration.y",
            "linear_acceleration.z"};

    }; // WheeledBipedalController
} // namespace wheeled_bipedal_controller

#endif