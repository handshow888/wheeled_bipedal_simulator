#include "wheeled_bipedal_hardware/can_hardware_interface.hpp"

namespace can_hardware
{
    hardware_interface::CallbackReturn CanHardwareInterface::on_init(const hardware_interface::HardwareInfo &info)
    {
        if (hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS)
        {
            return hardware_interface::CallbackReturn::ERROR;
        }
        auto type = info.type;
        RCLCPP_INFO(rclcpp::get_logger("CanHardwareInterface"), "type: %s", type.c_str());
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn CanHardwareInterface::on_configure(const rclcpp_lifecycle::State &pre)
    {
        (void)pre;
        // 初始化CAN驱动
        can_core_ = std::make_unique<CanSerial>("can0");
        try
        {
            can_core_->init();
            can_core_->set_frame_callback(
                std::bind(&CanHardwareInterface::handle_can_frame, this, std::placeholders::_1));
            can_core_->async_read();
            // 启动Boost.Asio事件循环线程
            can_core_->start_io_service();
            std::cout << "Boost.Asio线程已启动" << std::endl;
        }
        catch (const std::exception &e)
        {
            RCLCPP_FATAL(rclcpp::get_logger("CanHardwareInterface"), "CAN初始化失败: %s", e.what());
            throw;
        }
        return hardware_interface::CallbackReturn::SUCCESS;
    }
    hardware_interface::CallbackReturn CanHardwareInterface::on_activate(const rclcpp_lifecycle::State &pre)
    {
        (void)pre;
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn CanHardwareInterface::on_deactivate(const rclcpp_lifecycle::State &pre)
    {
        (void)pre;
        MIT stopCmd;
        for (int j = 0; j < 10; ++j)
        {
            for (int i = 0; i < 4; ++i)
            {
                send_can_frame(i + 1, stopCmd, true);
            }
        }
        // 等待 tx 队列被硬件发出（给一点时间）
        double waitTimeStart = rclcpp::Clock().now().seconds();
        while (rclcpp::Clock().now().seconds() - waitTimeStart <= 20 * 0.001)
        {
        }
        RCLCPP_INFO(rclcpp::get_logger("Can"), "Stop frames sent.");
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::return_type CanHardwareInterface::read(const rclcpp::Time &time, const rclcpp::Duration &period)
    {
        (void)time;
        (void)period;
        // RCLCPP_INFO(rclcpp::get_logger("CAN read"),"read");
        std::lock_guard<std::mutex> lock(state_mutex_);
        for (int i = 0; i < 4; ++i)
        {
            hw_state_motors_[i].pos = motorStates_[i].pos;
            hw_state_motors_[i].vel = motorStates_[i].vel;
            hw_state_motors_[i].tor = motorStates_[i].tor;
            // 电机 2 (i=1) 和电机 4 (i=3) 需要反转方向
            if (i == 1 || i == 3)
            {
                hw_state_motors_.at(i).pos = -hw_state_motors_.at(i).pos;
                hw_state_motors_.at(i).vel = -hw_state_motors_.at(i).vel;
                hw_state_motors_.at(i).tor = -hw_state_motors_.at(i).tor;
            }
        }
        // RCLCPP_INFO(rclcpp::get_logger("CAN read"), "pos:1:%.2f 2:%.2f 3:%.2f 4:%.2f",
        //             hw_state_motors_[0].pos, hw_state_motors_[1].pos, hw_state_motors_[2].pos, hw_state_motors_[3].pos);
        return hardware_interface::return_type::OK;
    }

    hardware_interface::return_type CanHardwareInterface::write(const rclcpp::Time &time, const rclcpp::Duration &period)
    {
        (void)time;
        (void)period;
        for (int i = 0; i < 4; ++i)
        {
            MIT cmd = hw_command_motors_[i]; // 拷贝一份
            if (i == 1 || i == 3)            // 电机 2 和电机 4 反向
            {
                cmd.pos = -cmd.pos;
                cmd.vel = -cmd.vel;
                cmd.tor = -cmd.tor;
                cmd.kp = -cmd.kp;
                cmd.kd = -cmd.kd;
            }
            send_can_frame(i + 1, cmd);
            // RCLCPP_INFO(rclcpp::get_logger("CAN write"), "%d:pos:%.3f vel:%.3f tor:%.3f kp:%.3f kd:%.3f",
            //             i,
            //             cmd.pos, cmd.vel, cmd.tor, cmd.kp, cmd.kd);
        }
        // RCLCPP_INFO(rclcpp::get_logger("CAN write"), "tor: 1:%.3f 2:%.3f 3:%.3f 4:%.3f",
        //             hw_command_motors_[0].tor, -hw_command_motors_[1].tor, hw_command_motors_[2].tor, -hw_command_motors_[3].tor);
        return hardware_interface::return_type::OK;
    }

    std::vector<hardware_interface::StateInterface> CanHardwareInterface::export_state_interfaces()
    {
        std::vector<hardware_interface::StateInterface> state_interfaces;
        state_interfaces.emplace_back(hardware_interface::StateInterface("RR_hip_joint", "position", &hw_state_motors_[0].pos));
        state_interfaces.emplace_back(hardware_interface::StateInterface("RR_hip_joint", "velocity", &hw_state_motors_[0].vel));
        state_interfaces.emplace_back(hardware_interface::StateInterface("RR_hip_joint", "effort", &hw_state_motors_[0].tor));

        state_interfaces.emplace_back(hardware_interface::StateInterface("LR_hip_joint", "position", &hw_state_motors_[1].pos));
        state_interfaces.emplace_back(hardware_interface::StateInterface("LR_hip_joint", "velocity", &hw_state_motors_[1].vel));
        state_interfaces.emplace_back(hardware_interface::StateInterface("LR_hip_joint", "effort", &hw_state_motors_[1].tor));

        state_interfaces.emplace_back(hardware_interface::StateInterface("RF_hip_joint", "position", &hw_state_motors_[2].pos));
        state_interfaces.emplace_back(hardware_interface::StateInterface("RF_hip_joint", "velocity", &hw_state_motors_[2].vel));
        state_interfaces.emplace_back(hardware_interface::StateInterface("RF_hip_joint", "effort", &hw_state_motors_[2].tor));

        state_interfaces.emplace_back(hardware_interface::StateInterface("LF_hip_joint", "position", &hw_state_motors_[3].pos));
        state_interfaces.emplace_back(hardware_interface::StateInterface("LF_hip_joint", "velocity", &hw_state_motors_[3].vel));
        state_interfaces.emplace_back(hardware_interface::StateInterface("LF_hip_joint", "effort", &hw_state_motors_[3].tor));
        return state_interfaces;
    }

    std::vector<hardware_interface::CommandInterface> CanHardwareInterface::export_command_interfaces()
    {
        std::vector<hardware_interface::CommandInterface> command_interfaces;
        command_interfaces.emplace_back(hardware_interface::CommandInterface("RR_hip_joint", "position", &hw_command_motors_[0].pos));
        command_interfaces.emplace_back(hardware_interface::CommandInterface("RR_hip_joint", "velocity", &hw_command_motors_[0].vel));
        command_interfaces.emplace_back(hardware_interface::CommandInterface("RR_hip_joint", "effort", &hw_command_motors_[0].tor));

        command_interfaces.emplace_back(hardware_interface::CommandInterface("LR_hip_joint", "position", &hw_command_motors_[1].pos));
        command_interfaces.emplace_back(hardware_interface::CommandInterface("LR_hip_joint", "velocity", &hw_command_motors_[1].vel));
        command_interfaces.emplace_back(hardware_interface::CommandInterface("LR_hip_joint", "effort", &hw_command_motors_[1].tor));

        command_interfaces.emplace_back(hardware_interface::CommandInterface("RF_hip_joint", "position", &hw_command_motors_[2].pos));
        command_interfaces.emplace_back(hardware_interface::CommandInterface("RF_hip_joint", "velocity", &hw_command_motors_[2].vel));
        command_interfaces.emplace_back(hardware_interface::CommandInterface("RF_hip_joint", "effort", &hw_command_motors_[2].tor));

        command_interfaces.emplace_back(hardware_interface::CommandInterface("LF_hip_joint", "position", &hw_command_motors_[3].pos));
        command_interfaces.emplace_back(hardware_interface::CommandInterface("LF_hip_joint", "velocity", &hw_command_motors_[3].vel));
        command_interfaces.emplace_back(hardware_interface::CommandInterface("LF_hip_joint", "effort", &hw_command_motors_[3].tor));
        return command_interfaces;
    }

    void CanHardwareInterface::handle_can_frame(const can_frame &frame)
    {
        switch ((frame.can_id & 0x0780))
        {
        case 0x780:
        {
            std::lock_guard<std::mutex> lock(state_mutex_);
            // pppp pppp pppp pppp vvvv vvvv vvvv tttt tttt tttt
            // Byte1     Byte2     Byte3     Byte4     Byte5
            uint16_t posInt = (frame.data[1] << 8) | frame.data[2];
            uint16_t velInt = (frame.data[3] << 4) | (frame.data[4] >> 4 & 0x0F);
            uint16_t torInt = ((frame.data[4] & 0x0F) << 8) | frame.data[5];

            int index = (frame.can_id & 0x0F) - 1;
            motorStates_[index].pos = static_cast<double>(uint_to_float(posInt, motorStates_[index].posMin, motorStates_[index].posMax, 16));
            motorStates_[index].vel = static_cast<double>(uint_to_float(velInt, motorStates_[index].velMin, motorStates_[index].velMax, 12));
            motorStates_[index].tor = static_cast<double>(uint_to_float(torInt, motorStates_[index].torMin, motorStates_[index].torMax, 12));

            // RCLCPP_INFO(rclcpp::get_logger("CAN"), "pos:1:%.2f 2:%.2f 3:%.2f 4:%.2f",
            //             motorStates_[0].pos, motorStates_[1].pos, motorStates_[2].pos, motorStates_[3].pos);
            // RCLCPP_INFO(rclcpp::get_logger("CAN"), "vel:1:%.5f 2:%.5f 3:%.5f 4:%.5f",
            //             motorStates_[0].vel, motorStates_[1].vel, motorStates_[2].vel, motorStates_[3].vel);
            // RCLCPP_INFO(rclcpp::get_logger("CAN"), "tor:1:%.2f 2:%.2f 3:%.2f 4:%.2f",
            //             motorStates_[0].tor, motorStates_[1].tor, motorStates_[2].tor, motorStates_[3].tor);

            break;
        }
        default:
            break;
        }
    }

    void CanHardwareInterface::send_can_frame(int motorId, MIT &cmd, bool sync)
    {
        can_frame frame_to_send;
        frame_to_send.can_id = 0x200 + motorId;
        frame_to_send.can_dlc = 8;
        LIMIT_MIN_MAX(cmd.pos, cmd.posMin, cmd.posMax);
        LIMIT_MIN_MAX(cmd.vel, cmd.velMin, cmd.velMax);
        LIMIT_MIN_MAX(cmd.kp, cmd.kpMin, cmd.kpMax);
        LIMIT_MIN_MAX(cmd.kd, cmd.kdMin, cmd.kdMax);
        LIMIT_MIN_MAX(cmd.tor, cmd.torMin, cmd.torMax);

        uint16_t posInt = float_to_uint((float)cmd.pos, cmd.posMin, cmd.posMax, 16);
        uint16_t velInt = float_to_uint((float)cmd.vel, cmd.velMin, cmd.velMax, 12);
        uint16_t kpInt = float_to_uint((float)cmd.kp, cmd.kpMin, cmd.kpMax, 12);
        uint16_t kdInt = float_to_uint((float)cmd.kd, cmd.kdMin, cmd.kdMax, 12);
        uint16_t torInt = float_to_uint((float)cmd.tor, cmd.torMin, cmd.torMax, 12);

        // 填充数据
        frame_to_send.data[0] = posInt >> 8;
        frame_to_send.data[1] = posInt & 0xFF;
        frame_to_send.data[2] = velInt >> 4;
        frame_to_send.data[3] = ((velInt & 0xF) << 4) | (kpInt >> 8);
        frame_to_send.data[4] = kpInt & 0xFF;
        frame_to_send.data[5] = kdInt >> 4;
        frame_to_send.data[6] = ((kdInt & 0xF) << 4) | (torInt >> 8);
        frame_to_send.data[7] = torInt & 0xFF;

        // 发送数据
        if (sync)
        {
            can_core_->send_frame_sync(frame_to_send);
            // RCLCPP_INFO(rclcpp::get_logger("Can"), "sent sync frame once");
        }
        else
            can_core_->send_frame(frame_to_send);
    }

    float uint_to_float(int x_int, float x_min, float x_max, int bits)
    {
        float span = x_max - x_min;
        float offset = x_min;
        return ((float)x_int) * span / ((float)((1 << bits) - 1)) + offset;
    }

    uint16_t float_to_uint(float x, float x_min, float x_max, uint8_t bits)
    {
        float span = x_max - x_min;
        float offset = x_min;
        return (uint16_t)((x - offset) * ((float)((1 << bits) - 1)) / span);
    }

} // namespace can_hardware

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(can_hardware::CanHardwareInterface, hardware_interface::SystemInterface)