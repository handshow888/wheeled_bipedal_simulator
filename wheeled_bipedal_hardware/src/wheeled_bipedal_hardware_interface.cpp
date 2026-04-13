#include "wheeled_bipedal_hardware/wheeled_bipedal_hardware_interface.hpp"

namespace wheeled_bipedal_hardware
{
    hardware_interface::CallbackReturn WheeledBipedalHardwareInterface::on_init(const hardware_interface::HardwareInfo &info)
    {
        if (hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS)
        {
            return hardware_interface::CallbackReturn::ERROR;
        }
        auto type = info.type;
        RCLCPP_INFO(rclcpp::get_logger("WheeledBipedalHardwareInterface"), "type: %s", type.c_str());

        header_map_[0x5A] = sizeof(ReceivePackage);
        header_map_[0x5B] = sizeof(ReceivePackage2);

        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn WheeledBipedalHardwareInterface::on_configure(const rclcpp_lifecycle::State &pre)
    {
        (void)pre;
        serial_core_ = std::make_unique<USBSerial>(info_.hardware_parameters["port_name"], std::stoi(info_.hardware_parameters["baud_rate"]));
        serial_core_->start_receive_thread([this](const uint8_t *data)
                                           { this->receiveCallback(data); },
                                           header_map_);
        return hardware_interface::CallbackReturn::SUCCESS;
    }
    hardware_interface::CallbackReturn WheeledBipedalHardwareInterface::on_activate(const rclcpp_lifecycle::State &pre)
    {
        (void)pre;
        return hardware_interface::CallbackReturn::SUCCESS;
    }
    hardware_interface::CallbackReturn WheeledBipedalHardwareInterface::on_deactivate(const rclcpp_lifecycle::State &pre)
    {
        (void)pre;
        // serial_core_.reset(); // 析构
        return hardware_interface::CallbackReturn::SUCCESS;
    }
    hardware_interface::return_type WheeledBipedalHardwareInterface::read(const rclcpp::Time &time, const rclcpp::Duration &period)
    {
        (void)time;
        (void)period;
        return hardware_interface::return_type::OK;
    }
    hardware_interface::return_type WheeledBipedalHardwareInterface::write(const rclcpp::Time &time, const rclcpp::Duration &period)
    {
        (void)time;
        (void)period;
        return hardware_interface::return_type::OK;
    }
    std::vector<hardware_interface::StateInterface> WheeledBipedalHardwareInterface::export_state_interfaces()
    {
        std::vector<hardware_interface::StateInterface> state_interfaces;
        return state_interfaces;
    }
    std::vector<hardware_interface::CommandInterface> WheeledBipedalHardwareInterface::export_command_interfaces()
    {
        std::vector<hardware_interface::CommandInterface> command_interfaces;
        return command_interfaces;
    }

    void WheeledBipedalHardwareInterface::receiveCallback(const uint8_t *data)
    {
        uint8_t header = data[0];
        switch (header)
        {
        case 0x5A:
        {
            std::lock_guard<std::mutex> lock(state_mutex_);
            auto pkg = reinterpret_cast<const ReceivePackage *>(data);
            std::cout << "received pkg with header: " << std::hex << std::showbase << pkg->header << std::endl;
            break;
        }
        case 0x5B:
        {
            std::lock_guard<std::mutex> lock(state_mutex_);
            auto pkg = reinterpret_cast<const ReceivePackage2 *>(data);
            std::cout << "received pkg with header: " << std::hex << std::showbase << pkg->header << std::endl;
            break;
        }
        default:
            break;
        }
    }

} // namespace wheeled_bipedal_hardware

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(wheeled_bipedal_hardware::WheeledBipedalHardwareInterface, hardware_interface::SystemInterface)