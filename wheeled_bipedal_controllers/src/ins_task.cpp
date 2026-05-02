#include "wheeled_bipedal_controllers/ins_task.h"
#include <chrono>

INS_t INS;

// static double lastTime = 0; // ms

const double xb[3] = {1, 0, 0};
const double yb[3] = {0, 1, 0};
const double zb[3] = {0, 0, 1};

static constexpr int X = 0;
static constexpr int Y = 1;
static constexpr int Z = 2;

void INS_Init()
{
    IMU_QuaternionEKF_Init(10, 0.001, 10000000, 1, 0);
    INS.AccelLPF = 0.0085;
}

void INS_Task(double aX, double aY, double aZ, double gX, double gY, double gZ, double dt)
{
    static int QEKF_INS_Q_Nan_count = 0;
    const double gravity[3] = {0, 0, 9.81};

    // auto now = std::chrono::high_resolution_clock::now();

    // // 转换为自 Unix epoch（1970-01-01 00:00:00 UTC）以来的纳秒数
    // auto ns = std::chrono::time_point_cast<std::chrono::nanoseconds>(now);
    // auto timestamp_ns = ns.time_since_epoch().count();

    // double currentTime = static_cast<double>(timestamp_ns) * 1e-9; // s
    
    // if (lastTime != 0.0)                         // 非首次运行
    //     dt = (currentTime - lastTime);
    // else // 防止首次启动dt计算值出错
    // {
    //     lastTime = currentTime;
    //     return;
    // }
    // lastTime = currentTime;
    // std::cout << "dt: " << dt << " s" << std::endl;
    if (dt <= 0)
    {
        // dt = 0.001; // 防止异常值
        // printf("currentTime:%.15f %s lastTime:%.15f\n", currentTime, (currentTime == lastTime) ? "==" : "!=", lastTime);
        RCLCPP_ERROR(rclcpp::get_logger("IMU_EKF"),"dt:%.3f <= 0 !!!!!", dt);
    }

    // ins update
    INS.Accel[X] = aX;
    INS.Accel[Y] = aY;
    INS.Accel[Z] = aZ;
    INS.Gyro[X] = gX;
    INS.Gyro[Y] = gY;
    INS.Gyro[Z] = gZ;

    // 核心函数,EKF更新四元数
    IMU_QuaternionEKF_Update(INS.Gyro[X], INS.Gyro[Y], INS.Gyro[Z], INS.Accel[X], INS.Accel[Y], INS.Accel[Z], dt);
    // std::cout << "q0: " << QEKF_INS.q[0]
    //           << " q1: " << QEKF_INS.q[1]
    //           << " q2: " << QEKF_INS.q[2]
    //           << " q3: " << QEKF_INS.q[3]
    //           << std::endl;
    if (std::isnan(QEKF_INS.q[0]) || std::isnan(QEKF_INS.q[1]) || std::isnan(QEKF_INS.q[2]) || std::isnan(QEKF_INS.q[3]))
    {
        QEKF_INS_Q_Nan_count++;
        printf("q0:%.3f \tq1:%.3f \tq2:%.3f \tq3:%.3f \tdt:%.6f\n", QEKF_INS.q[0], QEKF_INS.q[1], QEKF_INS.q[2], QEKF_INS.q[3], dt);
        // std::cerr << "Quaternion is NaN updateCount:" << QEKF_INS.UpdateCount << std::endl;
        exit(1);
    }

    memcpy(INS.q, QEKF_INS.q, sizeof(QEKF_INS.q));

    // 机体系基向量转换到导航坐标系，本例选取惯性系为导航系
    BodyFrameToEarthFrame(xb, INS.xn, INS.q);
    BodyFrameToEarthFrame(yb, INS.yn, INS.q);
    BodyFrameToEarthFrame(zb, INS.zn, INS.q);

    // 将重力从导航坐标系n转换到机体系b,随后根据加速度计数据计算运动加速度
    double gravity_b[3];
    EarthFrameToBodyFrame(gravity, gravity_b, INS.q);
    for (uint8_t i = 0; i < 3; ++i) // 同样过一个低通滤波
    {
        INS.MotionAccel_b[i] = (INS.Accel[i] - gravity_b[i]) * dt / (INS.AccelLPF + dt) + INS.MotionAccel_b[i] * INS.AccelLPF / (INS.AccelLPF + dt);
    }
    BodyFrameToEarthFrame(INS.MotionAccel_b, INS.MotionAccel_n, INS.q); // 转换回导航系n

    // 获取最终数据 degree
    INS.Yaw = QEKF_INS.Yaw;
    INS.Pitch = QEKF_INS.Pitch;
    INS.Roll = QEKF_INS.Roll;
    INS.YawTotalAngle = QEKF_INS.YawTotalAngle;

    // printf("Roll:%.3f \tPitch:%.3f \tYaw:%.3f \tNanCount:%d\n", INS.Roll, INS.Pitch, INS.Yaw, QEKF_INS_Q_Nan_count);
    // RCLCPP_INFO(rclcpp::get_logger("IMU_EKF"),"Roll:%.3f \tPitch:%.3f \tYaw:%.3f \tNanCount:%d\n", INS.Roll, INS.Pitch, INS.Yaw, QEKF_INS_Q_Nan_count);
}

/**
 * @brief          Transform 3dvector from BodyFrame to EarthFrame
 * @param[1]       vector in BodyFrame
 * @param[2]       vector in EarthFrame
 * @param[3]       quaternion
 */
void BodyFrameToEarthFrame(const double *vecBF, double *vecEF, double *q)
{
    vecEF[0] = 2.0 * ((0.5 - q[2] * q[2] - q[3] * q[3]) * vecBF[0] +
                       (q[1] * q[2] - q[0] * q[3]) * vecBF[1] +
                       (q[1] * q[3] + q[0] * q[2]) * vecBF[2]);

    vecEF[1] = 2.0 * ((q[1] * q[2] + q[0] * q[3]) * vecBF[0] +
                       (0.5 - q[1] * q[1] - q[3] * q[3]) * vecBF[1] +
                       (q[2] * q[3] - q[0] * q[1]) * vecBF[2]);

    vecEF[2] = 2.0 * ((q[1] * q[3] - q[0] * q[2]) * vecBF[0] +
                       (q[2] * q[3] + q[0] * q[1]) * vecBF[1] +
                       (0.5 - q[1] * q[1] - q[2] * q[2]) * vecBF[2]);
}

/**
 * @brief          Transform 3dvector from EarthFrame to BodyFrame
 * @param[1]       vector in EarthFrame
 * @param[2]       vector in BodyFrame
 * @param[3]       quaternion
 */
void EarthFrameToBodyFrame(const double *vecEF, double *vecBF, double *q)
{
    vecBF[0] = 2.0 * ((0.5 - q[2] * q[2] - q[3] * q[3]) * vecEF[0] +
                       (q[1] * q[2] + q[0] * q[3]) * vecEF[1] +
                       (q[1] * q[3] - q[0] * q[2]) * vecEF[2]);

    vecBF[1] = 2.0 * ((q[1] * q[2] - q[0] * q[3]) * vecEF[0] +
                       (0.5 - q[1] * q[1] - q[3] * q[3]) * vecEF[1] +
                       (q[2] * q[3] + q[0] * q[1]) * vecEF[2]);

    vecBF[2] = 2.0 * ((q[1] * q[3] + q[0] * q[2]) * vecEF[0] +
                       (q[2] * q[3] - q[0] * q[1]) * vecEF[1] +
                       (0.5 - q[1] * q[1] - q[2] * q[2]) * vecEF[2]);
}