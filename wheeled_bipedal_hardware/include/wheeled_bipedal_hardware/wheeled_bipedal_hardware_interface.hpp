#pragma once

#include <vector>
#include "rclcpp/rclcpp.hpp"
#include "hardware_interface/system_interface.hpp"
#include "serial_core.hpp"

namespace wheeled_bipedal_hardware
{
    struct imuState
    {
        double ax = 0.0;
        double ay = 0.0;
        double az = 0.0;
        double gx = 0.0;
        double gy = 0.0;
        double gz = 0.0;
        double timestamp = 0.0;
        int timestampLoopCount = 0; // 计数器溢出计数
    };

    struct motorState
    {
        double pos = 0.0;
        double vel = 0.0;
        double tor = 0.0;
    };

    struct motorCommand
    {
        // double pos = 0.0;
        // double vel = 0.0;
        double tor = 0.0;
    };

    // WheeledBipedalHardwareInterface
    class WheeledBipedalHardwareInterface : public hardware_interface::SystemInterface
    {
    public:
        // SystemInterface methods
        hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo &info) override;

        // Lifecycle methods
        hardware_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State &pre) override;
        hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State &pre) override;
        hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &pre) override;

        // SystemInterface methods
        hardware_interface::return_type read(const rclcpp::Time &time, const rclcpp::Duration &period) override;
        hardware_interface::return_type write(const rclcpp::Time &time, const rclcpp::Duration &period) override;

        // 导出接口方法
        std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
        std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

        void receiveCallback(const uint8_t *data);
        void applyCalibration();

    private:
        std::unique_ptr<USBSerial> serial_core_;
        std::mutex state_mutex_;
        // std::mutex cmd_mutex_;

        std::unordered_map<uint8_t, size_t> rec_pkg_header_map_; // 帧头到包长度的映射

        // 接收线程最新数据缓存
        imuState latest_imu_state_;
        double imuGyroOffset[3];
        double calibration_matrix_[3][3];
        double accel_scale_ = 1.0;
        int useCalibration_ = 1;
        std::array<motorState, 2> latest_motors_state_{};
        double jointMotorState = 0.0; // 关节电机: 0失能 非0使能

        // 状态和命令变量
        imuState hw_state_imu_;
        double oriXYZ_ = 0.0;
        double oriW_ = 1.0;
        double hw_state_joint_motor_ = 0.0;

        std::array<motorState, 2> hw_state_motors_{};
        std::array<motorCommand, 2> hw_command_motors_{};
    };
} // namespace wheeled_bipedal_hardware