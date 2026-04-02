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
        joints_bias_values_ = auto_declare<std::vector<double>>("joints_bias_values", {});
        command_interface_name_ = auto_declare<std::string>("command_interface_type", "effort");
        state_interface_names_ = auto_declare<std::vector<std::string>>("state_interface_type", {"position", "velocity", "effort"});
        imu_name_ = auto_declare<std::string>("imu_name", "imu_sensor");
        rodLengths = auto_declare<std::vector<double>>("rod_lengths", {});
        wheelRadius = auto_declare<double>("wheel_radius", 0.0);
        wheelSeparation = auto_declare<double>("wheel_separation", 0.0);
        legLengthMin = auto_declare<double>("leg_length_min", 0.15);
        legLengthMax = auto_declare<double>("leg_length_max", 0.37);
        LQR::K11poly = auto_declare<std::vector<double>>("K11poly", {});
        LQR::K12poly = auto_declare<std::vector<double>>("K12poly", {});
        LQR::K13poly = auto_declare<std::vector<double>>("K13poly", {});
        LQR::K14poly = auto_declare<std::vector<double>>("K14poly", {});
        LQR::K15poly = auto_declare<std::vector<double>>("K15poly", {});
        LQR::K16poly = auto_declare<std::vector<double>>("K16poly", {});
        LQR::K21poly = auto_declare<std::vector<double>>("K21poly", {});
        LQR::K22poly = auto_declare<std::vector<double>>("K22poly", {});
        LQR::K23poly = auto_declare<std::vector<double>>("K23poly", {});
        LQR::K24poly = auto_declare<std::vector<double>>("K24poly", {});
        LQR::K25poly = auto_declare<std::vector<double>>("K25poly", {});
        LQR::K26poly = auto_declare<std::vector<double>>("K26poly", {});

        debug_ = auto_declare<bool>("debug", true);
        rightLegSameWithLeft_ = auto_declare<bool>("right.same_with_left", true);
        leftLegLengthTarget_ = auto_declare<double>("left.leg_length", 0.25);
        rightLegLengthTarget_ = auto_declare<double>("right.leg_length", 0.25);
        leftLegLengthPID.setParams(auto_declare<double>("left.leg_length_P", 1250.0),
                                   auto_declare<double>("left.leg_length_I", 1000.0),
                                   auto_declare<double>("left.leg_length_D", 125.0));
        rightLegLengthPID.setParams(auto_declare<double>("right.leg_length_P", 1250.0),
                                    auto_declare<double>("right.leg_length_I", 1000.0),
                                    auto_declare<double>("right.leg_length_D", 125.0));
        deltaPhi0PID.setParams(auto_declare<double>("delta_phi0_P", 50.0),
                               auto_declare<double>("delta_phi0_I", 0.0),
                               auto_declare<double>("delta_phi0_D", 1.0));
        angularVelPID.setParams(auto_declare<double>("angular_vel_P", 0.0),
                                auto_declare<double>("angular_vel_I", 0.0),
                                auto_declare<double>("angular_vel_D", 0.0));
        rollErrPID.setParams(auto_declare<double>("roll_error_P", 0.0),
                             auto_declare<double>("roll_error_I", 0.0),
                             auto_declare<double>("roll_error_D", 0.0));

        if ((int)joint_names_.size() != 6)
        {
            RCLCPP_ERROR(get_node()->get_logger(), "expected 6 joints, but get %d joints", (int)joint_names_.size());
            return CallbackReturn::ERROR;
        }
        if ((int)joints_bias_values_.size() != 4)
        {
            RCLCPP_ERROR(get_node()->get_logger(), "expected 4 joints_downward_values, but get %d joints'", (int)joints_bias_values_.size());
            return CallbackReturn::ERROR;
        }
        if ((int)rodLengths.size() != 5)
        {
            RCLCPP_ERROR(get_node()->get_logger(), "expected 5 rod_lengths, but get %d lengths", (int)rodLengths.size());
            return CallbackReturn::ERROR;
        }
        if (wheelRadius <= 0.0)
        {
            RCLCPP_ERROR(get_node()->get_logger(), "expected correct wheel_radius, but get value: %.2f", wheelRadius);
            return CallbackReturn::ERROR;
        }
        if (wheelSeparation <= 0.0)
        {
            RCLCPP_ERROR(get_node()->get_logger(), "expected correct wheel_separation, but get value: %.2f", wheelSeparation);
            return CallbackReturn::ERROR;
        }

        return CallbackReturn::SUCCESS;
    }

    // 通常在on_configure读取参数，并做好启动控制器的准备工作
    controller_interface::CallbackReturn WheeledBipedalController::on_configure(const rclcpp_lifecycle::State &previous_state)
    {
        (void)previous_state;
        // auto callback = [this](const geometry_msgs::msg::Point::SharedPtr msg)
        // {
        //     iKMotorPosTarget_.clear();
        //     iKMotorPosTarget_.push_back(msg->x);
        //     iKMotorPosTarget_.push_back(msg->y);
        // };
        // testPointSub_ = get_node()->create_subscription<geometry_msgs::msg::Point>("/test_point", 10, callback);
        auto cmdVelCB = [this](const geometry_msgs::msg::Twist::SharedPtr msg)
        {
            recCmdVelTime_ = get_node()->now().seconds();
            recCmdVel_ = *msg;
        };
        cmdVelSub_ = get_node()->create_subscription<geometry_msgs::msg::Twist>("/cmd_vel", 10, cmdVelCB);
        joySub_ = get_node()->create_subscription<sensor_msgs::msg::Joy>("/joy", 10, std::bind(&WheeledBipedalController::joyCB, this, std::placeholders::_1));
        testInfoPub_ = get_node()->create_publisher<std_msgs::msg::Float64MultiArray>("/test_info", 10);
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
            { // "wheelPos", "velocity", "effort"
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
        // (void)time;
        // (void)period;
        static double initTime = time.seconds();

        loadStates();
        double leftWheelVel = lwMotorStates_.velocity * wheelRadius;
        double rightWheelVel = rwMotorStates_.velocity * wheelRadius;
        double dt = period.seconds();

        INS_Task(imuStates_.lin_acc_x, imuStates_.lin_acc_y, imuStates_.lin_acc_z,
                 imuStates_.ang_vel_x, imuStates_.ang_vel_y, imuStates_.ang_vel_z,
                 dt);

        if (time.seconds() - initTime <= 2.0)
            return controller_interface::return_type::OK;
        // RCLCPP_INFO(rclcpp::get_logger("MotorCalib"), "motorPos: LF:%.4f LR:%.4f RF:%.4f RR:%.4f\n",
        //             lfMotorStates_.wheelPos, lrMotorStates_.wheelPos,
        //             rfMotorStates_.wheelPos, rrMotorStates_.wheelPos);
        // if (iKMotorPosTarget_.size() == 2)//运动学逆解测试
        // {
        //     kinematics::point wheelPosTarget = {iKMotorPosTarget_.at(0), iKMotorPosTarget_.at(1)};
        //     double lfMotorPosTarget, lrMotorPosTarget;
        //     kinematics::inverseKinematics(wheelPosTarget, lrMotorPosTarget, lfMotorPosTarget);
        //     RCLCPP_INFO(get_node()->get_logger(), "wTarget(%.2f,%.2f) phi1:%.2f phi4:%.2f",
        //                 wheelPosTarget.x, wheelPosTarget.y, lrMotorPosTarget * rad2deg, lfMotorPosTarget * rad2deg);

        //     command_interfaces_[0].set_value(lfMotorPosTarget - joints_bias_values_[0]);
        //     command_interfaces_[1].set_value(lrMotorPosTarget - joints_bias_values_[1]);
        // }
        if (debug_)
        {
            rightLegSameWithLeft_ = get_node()->get_parameter("right.same_with_left").as_bool();
            leftLegLengthPID.setParams(get_node()->get_parameter("left.leg_length_P").as_double(),
                                       get_node()->get_parameter("left.leg_length_I").as_double(),
                                       get_node()->get_parameter("left.leg_length_D").as_double());
            leftLegLengthTarget_ = get_node()->get_parameter("left.leg_length").as_double();
            if (rightLegSameWithLeft_)
            {
                rightLegLengthPID.setParams(get_node()->get_parameter("left.leg_length_P").as_double(),
                                            get_node()->get_parameter("left.leg_length_I").as_double(),
                                            get_node()->get_parameter("left.leg_length_D").as_double());
                rightLegLengthTarget_ = leftLegLengthTarget_;
            }
            else
            {
                rightLegLengthPID.setParams(get_node()->get_parameter("right.leg_length_P").as_double(),
                                            get_node()->get_parameter("right.leg_length_I").as_double(),
                                            get_node()->get_parameter("right.leg_length_D").as_double());
                rightLegLengthTarget_ = get_node()->get_parameter("right.leg_length").as_double();
            }

            deltaPhi0PID.setParams(get_node()->get_parameter("delta_phi0_P").as_double(),
                                   get_node()->get_parameter("delta_phi0_I").as_double(),
                                   get_node()->get_parameter("delta_phi0_D").as_double());
            angularVelPID.setParams(get_node()->get_parameter("angular_vel_P").as_double(),
                                    get_node()->get_parameter("angular_vel_I").as_double(),
                                    get_node()->get_parameter("angular_vel_D").as_double());
            rollErrPID.setParams(get_node()->get_parameter("roll_error_P").as_double(),
                                 get_node()->get_parameter("roll_error_I").as_double(),
                                 get_node()->get_parameter("roll_error_D").as_double());
        }

        kinematics::fwdKinematicsResult leftFKResult = kinematics::forwardKinematics(lrMotorStates_.position, lfMotorStates_.position);
        kinematics::fwdKinematicsResult rightFKResult = kinematics::forwardKinematics(rrMotorStates_.position, rfMotorStates_.position);

        static double leftThetalast = 0.0;
        double leftTheta = leftFKResult.phi0 + INS.Pitch * deg2rad - M_PI_2;
        double leftThetaDot = (leftTheta - leftThetalast) / dt;
        leftThetalast = leftTheta;
        static double leftWheelx = 0.0;
        leftWheelx += leftWheelVel * dt;
        static double rightThetalast = 0.0;
        double rightTheta = rightFKResult.phi0 + INS.Pitch * deg2rad - M_PI_2;
        double rightThetaDot = (rightTheta - rightThetalast) / dt;
        rightThetalast = rightTheta;
        static double rightWheelx = 0.0;
        rightWheelx += rightWheelVel * dt;

        if (recCmdVel_.linear.x != 0.0 || recCmdVel_.angular.z != 0.0)
        {
            leftWheelx = 0.0;
            rightWheelx = 0.0;
        }
        // RCLCPP_INFO(get_node()->get_logger(), "\nL:theta:%.2f thetaDot:%.2f x:%.2f xDot:%.2f phi:%.2f phiDot:%.2f\nR:theta:%.2f thetaDot:%.2f x:%.2f xDot:%.2f phi:%.2f phiDot:%.2f",
        //             leftTheta, leftThetaDot,
        //             leftWheelx, leftWheelVel,
        //             -INS.Pitch * deg2rad, -INS.Gyro[1],
        //             rightTheta * rad2deg, rightThetaDot * rad2deg,
        //             rightWheelx, rightWheelVel,
        //             -INS.Pitch, -INS.Gyro[1] * rad2deg);

        // double robotLinearVel = (leftWheelVel + rightWheelVel) * 0.5;
        // double robotAngularVel = (rightWheelVel - leftWheelVel) / wheelSeparation;
        double robotAngularVel = INS.Gyro[2];

        double angularVel_T = angularVelPID.compute(recCmdVel_.angular.z, robotAngularVel, dt);
        // RCLCPP_INFO(get_node()->get_logger(), "angularTarget:%.3f now:%.3f GyroZ:%.3f T:%.3f",
        //             recCmdVel_.angular.z, robotAngularVel, INS.Gyro[2], angularVel_T);

        double rollCompensation = rollErrPID.compute(0.0, INS.Roll * deg2rad, dt);
        if (abs(INS.Pitch) > 10.0)
        {
            rollCompensation = rollErrPID.clear();
        }
        // RCLCPP_INFO(get_node()->get_logger(), "(deg)R:%.3f P:%.3f Y:%.3f rollCompensation:%.3f", INS.Roll, INS.Pitch, INS.Yaw, rollCompensation);
        double leftLegLengthCpstTarget = clamp(leftLegLengthTarget_ + rollCompensation, legLengthMin, legLengthMax);
        double rightLegLengthCpstTarget = clamp(rightLegLengthTarget_ - rollCompensation, legLengthMin, legLengthMax);
        double leftVMC_F = leftLegLengthPID.compute(leftLegLengthCpstTarget, leftFKResult.L0, dt);
        double rightVMC_F = rightLegLengthPID.compute(rightLegLengthCpstTarget, rightFKResult.L0, dt);

        double deltaPhi0 = rightFKResult.phi0 - leftFKResult.phi0;
        double deltaPhi0_Tp = deltaPhi0PID.compute(0.0, deltaPhi0, dt);
        // RCLCPP_INFO(get_node()->get_logger(), "deltaPhi0:%.3f deltaPhi0_Tp:%.3f", deltaPhi0, deltaPhi0_Tp);
        std_msgs::msg::Float64MultiArray testMsg;
        testMsg.data.push_back(angularVel_T);
        testMsg.data.push_back(robotAngularVel);
        testMsg.data.push_back(deltaPhi0_Tp);
        testMsg.data.push_back(deltaPhi0);
        testInfoPub_->publish(testMsg);

        // 左腿LQR
        double left_T_target = 0.0, left_Tp_target = 0.0;
        LQR::calKmat(leftFKResult.L0);
        LQR::cal_LQR_u(leftTheta,
                       leftThetaDot,
                       0 * leftWheelx,
                       leftWheelVel - recCmdVel_.linear.x + recCmdVel_.angular.z * wheelSeparation,
                       -INS.Pitch * deg2rad,
                       -INS.Gyro[1],
                       left_T_target, left_Tp_target);
        // RCLCPP_INFO(get_node()->get_logger(), "leftLQR:T:%.2f Tp:%.2f", left_T_target, left_Tp_target);
        // 右腿LQR
        double right_T_target = 0.0, right_Tp_target = 0.0;
        LQR::calKmat(rightFKResult.L0);
        LQR::cal_LQR_u(rightTheta,
                       rightThetaDot,
                       0 * rightWheelx,
                       rightWheelVel - recCmdVel_.linear.x - recCmdVel_.angular.z * wheelSeparation,
                       -INS.Pitch * deg2rad,
                       -INS.Gyro[1],
                       right_T_target,
                       right_Tp_target);

        double leftVMC_T1, leftVMC_T2;
        double rightVMC_T1, rightVMC_T2;

        // cal_VMC(leftVMC_T1, leftVMC_T2, leftFKResult, leftVMC_F, left_Tp_target - deltaPhi0_Tp);
        // cal_VMC(rightVMC_T1, rightVMC_T2, rightFKResult, rightVMC_F, right_Tp_target + deltaPhi0_Tp);

        cal_VMC(leftVMC_T1, leftVMC_T2, leftFKResult, leftVMC_F + rollCompensation, left_Tp_target - deltaPhi0_Tp);
        cal_VMC(rightVMC_T1, rightVMC_T2, rightFKResult, rightVMC_F - rollCompensation, right_Tp_target + deltaPhi0_Tp);

        // RCLCPP_INFO(get_node()->get_logger(), "Tar:%.4f Now:%.4f T1:%.4f T2:%.4f",
        //             leftLegLengthTarget_, leftFKResult.L0, leftVMC_T1, leftVMC_T2);

        command_interfaces_[0].set_value(leftVMC_T2);
        command_interfaces_[1].set_value(leftVMC_T1);
        command_interfaces_[2].set_value(rightVMC_T2);
        command_interfaces_[3].set_value(rightVMC_T1);
        command_interfaces_[4].set_value(left_T_target - angularVel_T);
        command_interfaces_[5].set_value(right_T_target + angularVel_T);

        // RCLCPP_INFO(get_node()->get_logger(), "LW x: %.2f y:%.2f L0:%.2f phi0:%.2f",
        //             leftFKResult.wheelPos.x, leftFKResult.wheelPos.y,
        //             leftFKResult.L0, leftFKResult.phi0);

        // RCLCPP_INFO(get_node()->get_logger(), "LW x: %.2f y:%.2f|RW x: %.2f y:%.2f",
        //             leftFKResult.wheelPos.x, leftFKResult.wheelPos.y,
        //             rightFKResult.wheelPos.x, rightFKResult.wheelPos.y);

        // update函数运行频率测试
        // static auto lastTime = std::chrono::system_clock::now();
        // auto now = std::chrono::system_clock::now();
        // auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(now - lastTime).count();
        // RCLCPP_INFO(get_node()->get_logger(), "duration: %ldms rate:%.2fhz", duration,
        //             1.0 / (duration / 1000.0));
        // lastTime = std::chrono::system_clock::now();
        // RCLCPP_INFO(get_node()->get_logger(), "time:%.3f rate:%.1fhz",
        //             time.seconds(),
        //             1.0 / dt);

        return controller_interface::return_type::OK;
    }

    void WheeledBipedalController::joyCB(sensor_msgs::msg::Joy::SharedPtr msg)
    {
        // const int axeJoystickLeftLRIdx = 0;
        const int axeJoystickLeftUDIdx = 1;
        const int axeJoystickRightLRIdx = 2;
        const int axeJoystickRightUDIdx = 3;
        // const int axeTriggerRightIdx = 4;
        // const int axeTriggerLeftIdx = 5;
        // const int axeArrowLRIdx = 6;
        // const int axeArrowUDIdx = 7;
        const int buttonAIdx = 0;
        const int buttonBIdx = 1;
        // const int buttonXIdx = 3;
        // const int buttonYIdx = 4;
        const int buttonLBIdx = 6;
        const int buttonRBIdx = 7;

        static uint8_t lastButtonA = 0;
        static uint8_t lastButtonB = 0;
        static bool spin = false;
        static int spinDir = 1;

        // 右摇杆上下
        if (abs(msg->axes.at(axeJoystickRightUDIdx)) < 0.1)
            recCmdVel_.linear.x = 0.0;
        else if (msg->buttons.at(buttonLBIdx) == 0) // LB按下时锁定线速度
            recCmdVel_.linear.x = lowPassFilter(msg->axes.at(3), recCmdVel_.linear.x, 0.5);
        // 左摇杆上下
        if (abs(msg->axes.at(axeJoystickLeftUDIdx)) > 0.1)
        {
            double legLengthIncrement = msg->axes.at(1) * 0.05;
            double newLegLength = get_node()->get_parameter("left.leg_length").as_double() + legLengthIncrement;
            newLegLength = clamp(newLegLength, legLengthMin, legLengthMax);
            get_node()->set_parameter(rclcpp::Parameter("left.leg_length", newLegLength));
        }
        // 按键A
        if (lastButtonA == 1 && msg->buttons.at(buttonAIdx) == 0)
            spin = !spin;
        lastButtonA = msg->buttons.at(buttonAIdx);
        // 按键B
        if (lastButtonB == 1 && msg->buttons.at(buttonBIdx) == 0)
            spinDir = -spinDir;
        lastButtonB = msg->buttons.at(buttonBIdx);

        if (spin)
            recCmdVel_.angular.z = 5.0 * spinDir;
        else
        {
            // 右摇杆左右
            if (abs(msg->axes.at(axeJoystickRightLRIdx)) < 0.2)
                recCmdVel_.angular.z = 0.0;
            else if (msg->buttons.at(buttonRBIdx) == 0) // RB按下时锁定旋转速度
                recCmdVel_.angular.z = lowPassFilter(msg->axes.at(2) * 5.0, recCmdVel_.angular.z, 0.1);
        }
    }

    void WheeledBipedalController::loadStates()
    {
        // --- 1. 读取电机数据 ---
        lfMotorStates_.position = state_interfaces_[lf_motor_state_indices_[0]].get_value() + joints_bias_values_[0];
        lfMotorStates_.velocity = state_interfaces_[lf_motor_state_indices_[1]].get_value();
        lfMotorStates_.effort = state_interfaces_[lf_motor_state_indices_[2]].get_value();

        lrMotorStates_.position = state_interfaces_[lr_motor_state_indices_[0]].get_value() + joints_bias_values_[1];
        lrMotorStates_.velocity = state_interfaces_[lr_motor_state_indices_[1]].get_value();
        lrMotorStates_.effort = state_interfaces_[lr_motor_state_indices_[2]].get_value();

        rfMotorStates_.position = state_interfaces_[rf_motor_state_indices_[0]].get_value() + joints_bias_values_[2];
        rfMotorStates_.velocity = state_interfaces_[rf_motor_state_indices_[1]].get_value();
        rfMotorStates_.effort = state_interfaces_[rf_motor_state_indices_[2]].get_value();

        rrMotorStates_.position = state_interfaces_[rr_motor_state_indices_[0]].get_value() + joints_bias_values_[3];
        rrMotorStates_.velocity = state_interfaces_[rr_motor_state_indices_[1]].get_value();
        rrMotorStates_.effort = state_interfaces_[rr_motor_state_indices_[2]].get_value();

        lwMotorStates_.position = state_interfaces_[lw_motor_state_indices_[0]].get_value();
        lwMotorStates_.velocity = state_interfaces_[lw_motor_state_indices_[1]].get_value();
        lwMotorStates_.effort = state_interfaces_[lw_motor_state_indices_[2]].get_value();

        rwMotorStates_.position = state_interfaces_[rw_motor_state_indices_[0]].get_value();
        rwMotorStates_.velocity = state_interfaces_[rw_motor_state_indices_[1]].get_value();
        rwMotorStates_.effort = state_interfaces_[rw_motor_state_indices_[2]].get_value();

        // --- 2. 读取 IMU 数据 ---
        imuStates_.ang_vel_x = state_interfaces_[imu_state_indices_[0]].get_value();
        imuStates_.ang_vel_y = state_interfaces_[imu_state_indices_[1]].get_value();
        imuStates_.ang_vel_z = state_interfaces_[imu_state_indices_[2]].get_value();
        imuStates_.lin_acc_x = state_interfaces_[imu_state_indices_[3]].get_value();
        imuStates_.lin_acc_y = state_interfaces_[imu_state_indices_[4]].get_value();
        imuStates_.lin_acc_z = state_interfaces_[imu_state_indices_[5]].get_value();
    }

} // namespace wheeled_bipedal_controller

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(wheeled_bipedal_controller::WheeledBipedalController, controller_interface::ControllerInterface)