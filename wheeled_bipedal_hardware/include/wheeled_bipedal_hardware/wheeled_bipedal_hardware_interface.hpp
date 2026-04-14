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
    };

    struct motorState
    {
        double pos = 0.0;
        double vel = 0.0;
        double tor = 0.0;
    };

    struct motorCommand
    {
        double pos = 0.0;
        double vel = 0.0;
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

    private:
        std::unique_ptr<USBSerial> serial_core_;
        std::mutex state_mutex_;
        // std::mutex cmd_mutex_;

        std::unordered_map<uint8_t, size_t> rec_pkg_header_map_; // 帧头到包长度的映射

        // 接收线程最新数据缓存
        imuState latest_imu_state_;
        std::array<motorState, 6> latest_motors_state_;

        // 状态和命令变量
        imuState hw_state_imu_;
        double oriXYZ = 0.0;
        double oriW = 1.0;
        // motorState hw_state_rr_motor_; // 右后关节电机
        // motorState hw_state_lr_motor_; // 左后关节电机
        // motorState hw_state_rf_motor_; // 右前关节电机
        // motorState hw_state_lf_motor_; // 左前关节电机
        // motorState hw_state_lw_motor_; // 左轮
        // motorState hw_state_rw_motor_; // 右轮
        std::array<motorState, 6> hw_state_motors; // 按照上述顺序

        std::array<motorCommand, 6> hw_command_motors;
    };
} // namespace wheeled_bipedal_hardware