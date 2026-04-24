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

        rec_pkg_header_map_[0x5A] = sizeof(ReceivePackage);
        rec_pkg_header_map_[0x5B] = sizeof(ReceivePackage2);
        rec_pkg_header_map_[0x5C] = sizeof(ReceivePackage3);

        imuGyroOffset[0] = std::stod(info_.hardware_parameters["imuGyroOffset_X"]);
        imuGyroOffset[1] = std::stod(info_.hardware_parameters["imuGyroOffset_Y"]);
        imuGyroOffset[2] = std::stod(info_.hardware_parameters["imuGyroOffset_Z"]);

        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn WheeledBipedalHardwareInterface::on_configure(const rclcpp_lifecycle::State &pre)
    {
        (void)pre;
        serial_core_ = std::make_unique<USBSerial>(info_.hardware_parameters["port_name"], std::stoi(info_.hardware_parameters["baud_rate"]));
        serial_core_->start_receive_thread([this](const uint8_t *data)
                                           { this->receiveCallback(data); },
                                           rec_pkg_header_map_);
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
        // 结构体初值全零，发送0力矩
        SendPackage zero_packet;
        uint8_t buffer[sizeof(SendPackage)];
        std::memcpy(buffer, &zero_packet, sizeof(SendPackage));
        Append_CRC16_Check_Sum(buffer, sizeof(SendPackage));
        serial_core_->send_raw(buffer, sizeof(SendPackage));
        serial_core_->stopRunning();
        serial_core_.reset(); // 完全释放串口资源
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::return_type WheeledBipedalHardwareInterface::read(const rclcpp::Time &time, const rclcpp::Duration &period)
    {
        (void)time;
        (void)period;
        std::lock_guard<std::mutex> lock(state_mutex_);
        hw_state_imu_ = latest_imu_state_;
        hw_state_motors_ = latest_motors_state_;
        // RCLCPP_INFO(rclcpp::get_logger("WBHI read"),"read"); 
        // hw_state_motors_[1].pos *= -1;
        // hw_state_motors_[3].pos *= -1;
        // RCLCPP_INFO(rclcpp::get_logger("WBHI read"),
        //             "%.5f %.5f %.5f %.5f %.5f %.5f",
        //             hw_state_motors_[0].pos, hw_state_motors_[1].pos, hw_state_motors_[2].pos, hw_state_motors_[3].pos,
        //             hw_state_motors_[4].vel, hw_state_motors_[5].vel);
        // RCLCPP_INFO(rclcpp::get_logger("WBHI read"),
        //             "ax:%.3f \tay:%.3f \taz:%.3f \tgx:%.3f \tgy:%.3f \tgz:%.3f\n",
        //             hw_state_imu_.ax, hw_state_imu_.ay, hw_state_imu_.az, hw_state_imu_.gx, hw_state_imu_.gy, hw_state_imu_.gz);
        return hardware_interface::return_type::OK;
    }

    hardware_interface::return_type WheeledBipedalHardwareInterface::write(const rclcpp::Time &time, const rclcpp::Duration &period)
    {
        (void)time;
        (void)period;
        SendPackage packet2send;
        // packet2send.motors_effort[0] = static_cast<float>(hw_command_motors_[0].tor);
        // packet2send.motors_effort[1] = static_cast<float>(-hw_command_motors_[1].tor);
        // packet2send.motors_effort[2] = static_cast<float>(hw_command_motors_[2].tor);
        // packet2send.motors_effort[3] = static_cast<float>(-hw_command_motors_[3].tor);
        packet2send.motors_effort[0] = static_cast<float>(hw_command_motors_[0].tor);
        packet2send.motors_effort[1] = static_cast<float>(hw_command_motors_[1].tor);
        uint8_t buffer[sizeof(SendPackage)];
        std::memcpy(buffer, &packet2send, sizeof(SendPackage));
        Append_CRC16_Check_Sum(buffer, sizeof(SendPackage));
        serial_core_->send_raw(buffer, sizeof(SendPackage));
        // RCLCPP_INFO(rclcpp::get_logger("WBHI write"),
        //             "%.5f %.5f %.5f %.5f %.5f %.5f",
        //             packet2send.motors_effort[0], -packet2send.motors_effort[1], packet2send.motors_effort[2], -packet2send.motors_effort[3],
        //             packet2send.motors_effort[4], packet2send.motors_effort[5]);
        return hardware_interface::return_type::OK;
    }

    std::vector<hardware_interface::StateInterface> WheeledBipedalHardwareInterface::export_state_interfaces()
    {
        std::vector<hardware_interface::StateInterface> state_interfaces;
        state_interfaces.emplace_back(hardware_interface::StateInterface("imu_sensor", "orientation.x", &oriXYZ_));
        state_interfaces.emplace_back(hardware_interface::StateInterface("imu_sensor", "orientation.y", &oriXYZ_));
        state_interfaces.emplace_back(hardware_interface::StateInterface("imu_sensor", "orientation.z", &oriXYZ_));
        state_interfaces.emplace_back(hardware_interface::StateInterface("imu_sensor", "orientation.w", &oriW_));
        state_interfaces.emplace_back(hardware_interface::StateInterface("imu_sensor", "linear_acceleration.x", &hw_state_imu_.ax));
        state_interfaces.emplace_back(hardware_interface::StateInterface("imu_sensor", "linear_acceleration.y", &hw_state_imu_.ay));
        state_interfaces.emplace_back(hardware_interface::StateInterface("imu_sensor", "linear_acceleration.z", &hw_state_imu_.az));
        state_interfaces.emplace_back(hardware_interface::StateInterface("imu_sensor", "angular_velocity.x", &hw_state_imu_.gx));
        state_interfaces.emplace_back(hardware_interface::StateInterface("imu_sensor", "angular_velocity.y", &hw_state_imu_.gy));
        state_interfaces.emplace_back(hardware_interface::StateInterface("imu_sensor", "angular_velocity.z", &hw_state_imu_.gz));

        state_interfaces.emplace_back(hardware_interface::StateInterface("left_wheel_joint", "position", &hw_state_motors_[0].pos));
        state_interfaces.emplace_back(hardware_interface::StateInterface("left_wheel_joint", "velocity", &hw_state_motors_[0].vel));
        state_interfaces.emplace_back(hardware_interface::StateInterface("right_wheel_joint", "position", &hw_state_motors_[1].pos));
        state_interfaces.emplace_back(hardware_interface::StateInterface("right_wheel_joint", "velocity", &hw_state_motors_[1].vel));
        return state_interfaces;
    }

    std::vector<hardware_interface::CommandInterface> WheeledBipedalHardwareInterface::export_command_interfaces()
    {
        std::vector<hardware_interface::CommandInterface> command_interfaces;
        command_interfaces.emplace_back(hardware_interface::CommandInterface("left_wheel_joint", "effort", &hw_command_motors_[0].tor));
        command_interfaces.emplace_back(hardware_interface::CommandInterface("right_wheel_joint", "effort", &hw_command_motors_[1].tor));
        return command_interfaces;
    }

    void WheeledBipedalHardwareInterface::receiveCallback(const uint8_t *data)
    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        uint8_t header = data[0];
        switch (header)
        {
        case 0x5A:
        {
            auto pkg = reinterpret_cast<const ReceivePackage *>(data);
            latest_imu_state_.ax = static_cast<double>(pkg->ax) * 9.81;
            latest_imu_state_.ay = static_cast<double>(pkg->ay) * 9.81;
            latest_imu_state_.az = static_cast<double>(pkg->az) * 9.81;
            latest_imu_state_.gx = (static_cast<double>(pkg->gx) - imuGyroOffset[0]) * (M_PI / 180.0);
            latest_imu_state_.gy = (static_cast<double>(pkg->gy) - imuGyroOffset[1]) * (M_PI / 180.0);
            latest_imu_state_.gz = (static_cast<double>(pkg->gz) - imuGyroOffset[2]) * (M_PI / 180.0);
            // RCLCPP_INFO(rclcpp::get_logger("WBHI read"),
            //         "ax:%.3f \tay:%.3f \taz:%.3f \tgx:%.3f \tgy:%.3f \tgz:%.3f\n",
            //         latest_imu_state_.ax, latest_imu_state_.ay, latest_imu_state_.az,
            //         latest_imu_state_.gx, latest_imu_state_.gy, latest_imu_state_.gz);
            break;
        }
        case 0x5B:
        {
            auto pkg = reinterpret_cast<const ReceivePackage2 *>(data);
            int index = static_cast<int>(pkg->motorID) - 5;
            // latest_motors_state_[index].pos = static_cast<double>(pkg->motorPos);
            latest_motors_state_[index].vel = static_cast<double>(pkg->motorVel);
            // latest_motors_state_[index].tor = static_cast<double>(pkg->motorTor);
            // RCLCPP_INFO(rclcpp::get_logger("WBHI read"),
            //             "%.5f %.5f",
            //             hw_state_motors_[0].vel, hw_state_motors_[1].vel);
            break;
        }
        case 0x5C:
        {
            // auto pkg = reinterpret_cast<const ReceivePackage3 *>(data);
            // RCLCPP_INFO(rclcpp::get_logger("WBHI test"),
            //             "%.5f %.5f %.5f %.5f %.5f %.5f",
            //             pkg->motors_effort[0], pkg->motors_effort[1], pkg->motors_effort[2], pkg->motors_effort[3],
            //             pkg->motors_effort[4], pkg->motors_effort[5]);
            break;
        }
        default:
            break;
        }
    }

} // namespace wheeled_bipedal_hardware

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(wheeled_bipedal_hardware::WheeledBipedalHardwareInterface, hardware_interface::SystemInterface)