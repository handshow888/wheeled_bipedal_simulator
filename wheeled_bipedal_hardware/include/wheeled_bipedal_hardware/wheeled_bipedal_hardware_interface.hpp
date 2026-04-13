#pragma once

#include "rclcpp/rclcpp.hpp"
#include "hardware_interface/system_interface.hpp"
#include "serial_core.hpp"

namespace wheeled_bipedal_hardware
{
    // WheeledBipedalHardwareInterface
    class WheeledBipedalHardwareInterface : public hardware_interface::SystemInterface
    {
    public:
        // SystemInterface methods - 必须实现on_init
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
        std::mutex cmd_mutex_;

        std::unordered_map<uint8_t, size_t> header_map_; // 帧头到包长度的映射

        // 状态和命令变量 - 为两个电机分别定义
        double hw_states_position_right_;
        double hw_states_velocity_right_;
        double hw_commands_velocity_right_;

        double hw_states_position_left_;
        double hw_states_velocity_left_;
        double hw_commands_velocity_left_;
    };
} // namespace wheeled_bipedal_hardware