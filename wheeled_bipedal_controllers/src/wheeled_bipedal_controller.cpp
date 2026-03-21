#include "wheeled_bipedal_controllers/wheeled_bipedal_controller.hpp"
#include <chrono>
#include <ctime>

namespace wheeled_bipedal_controller
{
    WheeledBipedalController::WheeledBipedalController() : controller_interface::ControllerInterface()
    {
    }

    controller_interface::CallbackReturn WheeledBipedalController::on_init()
    {
        joint_names_ = auto_declare<std::vector<std::string>>("joints", {});
        command_interface_name_ = auto_declare<std::string>("command_interface_type", "effort");
        state_interface_names_ = auto_declare<std::vector<std::string>>("state_interface_type", {"position", "velocity", "effort"});
        imu_name_ = auto_declare<std::string>("imu_name", "imu_sensor");
        if ((int)joint_names_.size() != 6)
        {
            RCLCPP_ERROR(get_node()->get_logger(), "expected 6 joints, but get %d joints", (int)joint_names_.size());
            return CallbackReturn::ERROR;
        }
        return CallbackReturn::SUCCESS;
    }

    // 通常在on_configure读取参数，并做好启动控制器的准备工作
    controller_interface::CallbackReturn WheeledBipedalController::on_configure(const rclcpp_lifecycle::State &previous_state)
    {
        (void)previous_state;
        return CallbackReturn::SUCCESS;
    }

    controller_interface::InterfaceConfiguration WheeledBipedalController::command_interface_configuration() const
    {
        // static int runCount = 0;
        // RCLCPP_INFO(get_node()->get_logger(),"------------------------------\ncall command_interface_configuration() %d times\n------------------------------", ++runCount);
        controller_interface::InterfaceConfiguration config;
        config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
        config.names.reserve(joint_names_.size());
        for (auto &joint_name : joint_names_)
        {
            auto config_name = joint_name + "/" + command_interface_name_;
            // RCLCPP_INFO(get_node()->get_logger(), "command_config_name:%s", config_name.c_str());
            config.names.push_back(config_name);
        }
        return config;
    }

    controller_interface::InterfaceConfiguration WheeledBipedalController::state_interface_configuration() const
    {
        // static int runCount = 0;
        // RCLCPP_INFO(get_node()->get_logger(),"------------------------------\ncall state_interface_configuration() %d times\n------------------------------", ++runCount);
        controller_interface::InterfaceConfiguration config;
        config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
        config.names.reserve(joint_names_.size() * state_interface_names_.size() + imu_interface_names_.size());
        for (auto &joint_name : joint_names_)
        {
            for (auto &interface_name : state_interface_names_)
            {
                auto config_name = joint_name + "/" + interface_name;
                // RCLCPP_INFO(get_node()->get_logger(), "state_config_name:%s", config_name.c_str());
                config.names.push_back(config_name);
            }
        }
        // 注册 IMU 状态接口
        for (const auto &imu_interface_name : imu_interface_names_)
        {
            config.names.push_back(imu_name_ + "/" + imu_interface_name);
        }

        return config;
    }

    // on_activate检查接口并根据需要进行排序，同时为成员分配初始值。此方法是实时循环的一部分，因此应避免任何内存占用，并尽可能保持其简洁。
    controller_interface::CallbackReturn WheeledBipedalController::on_activate(const rclcpp_lifecycle::State &previous_state)
    {
        // RCLCPP_INFO(get_node()->get_logger(),"------------------------------\non_activate()\n------------------------------");
        (void)previous_state;
        lf_motor_state_indices_.clear();
        lr_motor_state_indices_.clear();
        rf_motor_state_indices_.clear();
        rr_motor_state_indices_.clear();
        lw_motor_state_indices_.clear();
        rw_motor_state_indices_.clear();

        // 定义一个 Lambda 辅助函数，用于根据前缀匹配并存储索引
        auto get_motor_indices = [&](const std::string &prefix, std::vector<size_t> &indices_array)
        {
            indices_array.clear();
            for (const auto &interface_type : state_interface_names_)
            { // "position", "velocity", "effort"
                std::string target_name = prefix + "/" + interface_type;
                for (size_t i = 0; i < state_interfaces_.size(); ++i)
                {
                    if (state_interfaces_[i].get_name() == target_name)
                    {
                        indices_array.push_back(i);
                        break;
                    }
                }
            }
        };
        // 批量获取 6 个电机的索引
        get_motor_indices(joint_names_[0], lf_motor_state_indices_);
        get_motor_indices(joint_names_[1], lr_motor_state_indices_);
        get_motor_indices(joint_names_[2], rf_motor_state_indices_);
        get_motor_indices(joint_names_[3], rr_motor_state_indices_);
        get_motor_indices(joint_names_[4], lw_motor_state_indices_);
        get_motor_indices(joint_names_[5], rw_motor_state_indices_);

        // 获取 IMU 的索引
        imu_state_indices_.clear();
        for (const auto &imu_interface_name : imu_interface_names_)
        {
            for (size_t i = 0; i < state_interfaces_.size(); ++i)
            {
                if (state_interfaces_[i].get_name() == imu_name_ + "/" + imu_interface_name)
                {
                    imu_state_indices_.push_back(i);
                    break;
                }
            }
        }

        return CallbackReturn::SUCCESS;
    }

    controller_interface::return_type WheeledBipedalController::update(const rclcpp::Time &time, const rclcpp::Duration &period)
    {
        (void)time;
        (void)period;
        loadStates();
        // update函数运行频率测试
        // static auto lastTime = std::chrono::system_clock::now();
        // auto now = std::chrono::system_clock::now();
        // auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(now - lastTime).count();
        // RCLCPP_INFO(get_node()->get_logger(), "duration: %ldms rate:%.2fhz", duration,
        //             1.0 / (duration / 1000.0));
        // lastTime = std::chrono::system_clock::now();
        // RCLCPP_INFO(get_node()->get_logger(), "time:%.3f rate:%.1fhz",
        //             time.seconds(),
        //             1.0 / period.seconds());

        // 读取state测试
        // RCLCPP_INFO(get_node()->get_logger(), "gx:%.2f,gy:%.2f,gz:%.2f,ax:%.2f,ay:%.2f,az:%.2f\n",
        //             imuStates_.ang_vel_x,
        //             imuStates_.ang_vel_y,
        //             imuStates_.ang_vel_z,
        //             imuStates_.lin_acc_x,
        //             imuStates_.lin_acc_y,
        //             imuStates_.lin_acc_z);
        // motorStates *motorStatePtr = &rrMotorStates_;
        // std::vector<size_t> *motor_state_indices = &rr_motor_state_indices_;
        // for (size_t index : *motor_state_indices)
        // {
        //     printf("%s(index:%d) state:%.2f == ", state_interfaces_[index].get_name().c_str(),
        //                 (int)index,
        //                 state_interfaces_[index].get_value());
        //     if (state_interfaces_[index].get_interface_name() == "position")
        //     {
        //         printf("%.2f\n", motorStatePtr->position);
        //     }
        //     else if (state_interfaces_[index].get_interface_name() == "velocity")
        //     {
        //         printf("%.2f\n", motorStatePtr->velocity);
        //     }
        //     else
        //     {
        //         printf("%.2f\n", motorStatePtr->effort);
        //     }
        // }
        return controller_interface::return_type::OK;
    }

    void WheeledBipedalController::loadStates()
    {
        // --- 1. 读取电机数据 ---
        // if (lf_motor_state_indices_.size() == 3)
        // {
        lfMotorStates_.position = state_interfaces_[lf_motor_state_indices_[0]].get_value();
        lfMotorStates_.velocity = state_interfaces_[lf_motor_state_indices_[1]].get_value();
        lfMotorStates_.effort = state_interfaces_[lf_motor_state_indices_[2]].get_value();
        // }
        // if (lr_motor_state_indices_.size() == 3)
        // {
        lrMotorStates_.position = state_interfaces_[lr_motor_state_indices_[0]].get_value();
        lrMotorStates_.velocity = state_interfaces_[lr_motor_state_indices_[1]].get_value();
        lrMotorStates_.effort = state_interfaces_[lr_motor_state_indices_[2]].get_value();
        // }
        // if (rf_motor_state_indices_.size() == 3)
        // {
        rfMotorStates_.position = state_interfaces_[rf_motor_state_indices_[0]].get_value();
        rfMotorStates_.velocity = state_interfaces_[rf_motor_state_indices_[1]].get_value();
        rfMotorStates_.effort = state_interfaces_[rf_motor_state_indices_[2]].get_value();
        // }
        // if (rr_motor_state_indices_.size() == 3)
        // {
        rrMotorStates_.position = state_interfaces_[rr_motor_state_indices_[0]].get_value();
        rrMotorStates_.velocity = state_interfaces_[rr_motor_state_indices_[1]].get_value();
        rrMotorStates_.effort = state_interfaces_[rr_motor_state_indices_[2]].get_value();
        // }
        // if (lw_motor_state_indices_.size() == 3)
        // {
        lwMotorStates_.position = state_interfaces_[lw_motor_state_indices_[0]].get_value();
        lwMotorStates_.velocity = state_interfaces_[lw_motor_state_indices_[1]].get_value();
        lwMotorStates_.effort = state_interfaces_[lw_motor_state_indices_[2]].get_value();
        // }
        // if (rw_motor_state_indices_.size() == 3)
        // {
        rwMotorStates_.position = state_interfaces_[rw_motor_state_indices_[0]].get_value();
        rwMotorStates_.velocity = state_interfaces_[rw_motor_state_indices_[1]].get_value();
        rwMotorStates_.effort = state_interfaces_[rw_motor_state_indices_[2]].get_value();
        // }
        // --- 2. 读取 IMU 数据 ---
        // if (imu_state_indices_.size() == 6)
        // {
        imuStates_.ang_vel_x = state_interfaces_[imu_state_indices_[0]].get_value();
        imuStates_.ang_vel_y = state_interfaces_[imu_state_indices_[1]].get_value();
        imuStates_.ang_vel_z = state_interfaces_[imu_state_indices_[2]].get_value();
        imuStates_.lin_acc_x = state_interfaces_[imu_state_indices_[3]].get_value();
        imuStates_.lin_acc_y = state_interfaces_[imu_state_indices_[4]].get_value();
        imuStates_.lin_acc_z = state_interfaces_[imu_state_indices_[5]].get_value();
        // }
    }

} // namespace wheeled_bipedal_controller

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(wheeled_bipedal_controller::WheeledBipedalController, controller_interface::ControllerInterface)