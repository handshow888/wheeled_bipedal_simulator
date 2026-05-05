#pragma once

#include <queue>
#include "rclcpp/rclcpp.hpp"
#include "hardware_interface/system_interface.hpp"
#include "CanSerialCore.hpp"

#define LIMIT_MIN_MAX(x, min, max) (x) = (((x) <= (min)) ? (min) : (((x) >= (max)) ? (max) : (x)))

namespace can_hardware
{
    float uint_to_float(int x_int, float x_min, float x_max, int bits);
    uint16_t float_to_uint(float x, float x_min, float x_max, uint8_t bits);

    struct MIT
    {
        double pos = 0.0f;
        double vel = 0.0f;
        double tor = 0.0f;
        double kp = 0.0f;
        double kd = 0.0f;
        float posMin = -12.56f;
        float posMax = 12.56f;
        float velMin = -45.0f;
        float velMax = 45.0f;
        float kpMin = 0.0f;
        float kpMax = 500.0f;
        float kdMin = 0.0f;
        float kdMax = 5.0f;
        float torMin = -18.0f;
        float torMax = 18.0f;
    };

    class CanHardwareInterface : public hardware_interface::SystemInterface
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

        void handle_can_frame(const can_frame &frame);
        void send_can_frame(int motorId, MIT &cmd, bool sync = false);

    private:
        std::unique_ptr<CanSerial> can_core_;
        std::string can_device_ = "can0";
        std::mutex state_mutex_;

        std::array<std::queue<MIT>,4> motorStatesQueue_;

        // 接收到的数据
        std::array<MIT, 4> motorStates_{};

        // 状态和命令变量
        std::array<MIT, 4> hw_state_motors_{};
        std::array<MIT, 4> hw_command_motors_{};
    };

} // namespace can_hardware