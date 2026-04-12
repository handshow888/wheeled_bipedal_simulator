#include "wheeled_bipedal_controllers/QuaternionEKF.h"

QEKF_INS_t QEKF_INS;

const double IMU_QuaternionEKF_F[36] = {1, 0, 0, 0, 0, 0,
                                       0, 1, 0, 0, 0, 0,
                                       0, 0, 1, 0, 0, 0,
                                       0, 0, 0, 1, 0, 0,
                                       0, 0, 0, 0, 1, 0,
                                       0, 0, 0, 0, 0, 1};
double IMU_QuaternionEKF_P[36] = {100000, 0.1, 0.1, 0.1, 0.1, 0.1,
                                 0.1, 100000, 0.1, 0.1, 0.1, 0.1,
                                 0.1, 0.1, 100000, 0.1, 0.1, 0.1,
                                 0.1, 0.1, 0.1, 100000, 0.1, 0.1,
                                 0.1, 0.1, 0.1, 0.1, 100, 0.1,
                                 0.1, 0.1, 0.1, 0.1, 0.1, 100};
double IMU_QuaternionEKF_K[18];
double IMU_QuaternionEKF_H[18];

static void IMU_QuaternionEKF_Observe(KalmanFilter_t *kf);
static void IMU_QuaternionEKF_F_Linearization_P_Fading(KalmanFilter_t *kf);
static void IMU_QuaternionEKF_SetH(KalmanFilter_t *kf);
static void IMU_QuaternionEKF_xhatUpdate(KalmanFilter_t *kf);

/**
 * @brief Quaternion EKF initialization and some reference value
 * @param[in] process_noise1 quaternion process noise    10
 * @param[in] process_noise2 gyro bias process noise     0.001
 * @param[in] measure_noise  accel measure noise         1000000
 * @param[in] lambda         fading coefficient          0.9996
 * @param[in] lpf            lowpass filter coefficient  0
 */
void IMU_QuaternionEKF_Init(double process_noise1, double process_noise2, double measure_noise, double lambda, double lpf)
{
    QEKF_INS.Initialized = 1;
    QEKF_INS.Q1 = process_noise1;
    QEKF_INS.Q2 = process_noise2;
    QEKF_INS.R = measure_noise;
    QEKF_INS.ChiSquareTestThreshold = 1e-8;
    QEKF_INS.ConvergeFlag = 0;
    QEKF_INS.ErrorCount = 0;
    QEKF_INS.UpdateCount = 0;
    if (lambda > 1)
    {
        lambda = 1;
    }
    QEKF_INS.lambda = lambda;
    QEKF_INS.accLPFcoef = lpf;

    // 初始化矩阵维度信息
    Kalman_Filter_Init(&QEKF_INS.IMU_QuaternionEKF);

    // 姿态初始化
    QEKF_INS.IMU_QuaternionEKF.xhat(0, 0) = 1;
    QEKF_INS.IMU_QuaternionEKF.xhat(1, 0) = 0;
    QEKF_INS.IMU_QuaternionEKF.xhat(2, 0) = 0;
    QEKF_INS.IMU_QuaternionEKF.xhat(3, 0) = 0;

    // 自定义函数初始化,用于扩展或增加kf的基础功能
    QEKF_INS.IMU_QuaternionEKF.User_Func0_f = IMU_QuaternionEKF_Observe;
    QEKF_INS.IMU_QuaternionEKF.User_Func1_f = IMU_QuaternionEKF_F_Linearization_P_Fading;
    QEKF_INS.IMU_QuaternionEKF.User_Func2_f = IMU_QuaternionEKF_SetH;
    QEKF_INS.IMU_QuaternionEKF.User_Func3_f = IMU_QuaternionEKF_xhatUpdate;

    // 设定标志位,用自定函数替换kf标准步骤中的SetK(计算增益)以及xhatupdate(后验估计/融合)
    QEKF_INS.IMU_QuaternionEKF.SkipEq3 = 1;
    QEKF_INS.IMU_QuaternionEKF.SkipEq4 = 1;

    QEKF_INS.IMU_QuaternionEKF.F = Eigen::Map<const Eigen::Matrix<double, 6, 6, Eigen::RowMajor>>(IMU_QuaternionEKF_F);
    QEKF_INS.IMU_QuaternionEKF.P = Eigen::Map<const Eigen::Matrix<double, 6, 6, Eigen::RowMajor>>(IMU_QuaternionEKF_P);
    // std::cout << "QEKF_INS.IMU_QuaternionEKF.F: " << std::endl
    //           << QEKF_INS.IMU_QuaternionEKF.F << std::endl;
}

/**
 * @brief Quaternion EKF update
 * @param[in]       gyro x y z in rad/s
 * @param[in]       accel x y z in m/s²
 * @param[in]       update period in s
 */
void IMU_QuaternionEKF_Update(double gx, double gy, double gz, double ax, double ay, double az, double dt)
{
    // 0.5(Ohm-Ohm^bias)*deltaT,用于更新工作点处的状态转移F矩阵
    static double halfgxdt, halfgydt, halfgzdt;
    static double accelInvNorm;
    if (!QEKF_INS.Initialized)
    {
        IMU_QuaternionEKF_Init(10, 0.001, 1000000 * 10, 0.9996 * 0 + 1, 0);
    }

    /*   F, number with * represent vals to be set
     0      1*     2*     3*     4     5
     6*     7      8*     9*    10    11
    12*    13*    14     15*    16    17
    18*    19*    20*    21     22    23
    24     25     26     27     28    29
    30     31     32     33     34    35
    */
    QEKF_INS.dt = dt;

    QEKF_INS.Gyro[0] = gx - QEKF_INS.GyroBias[0];
    QEKF_INS.Gyro[1] = gy - QEKF_INS.GyroBias[1];
    QEKF_INS.Gyro[2] = gz - QEKF_INS.GyroBias[2];

    // set F
    halfgxdt = 0.5 * QEKF_INS.Gyro[0] * dt;
    halfgydt = 0.5 * QEKF_INS.Gyro[1] * dt;
    halfgzdt = 0.5 * QEKF_INS.Gyro[2] * dt;
    // std::cout << "half gx dt: " << halfgxdt << " half gy dt: " << halfgydt << " half gz dt: " << halfgzdt << std::endl;

    // 此部分设定状态转移矩阵F的左上角部分 4x4子矩阵,即0.5(Ohm-Ohm^bias)*deltaT,右下角有一个2x2单位阵已经初始化好了
    // 注意在predict步F的右上角是4x2的零矩阵,因此每次predict的时候都会调用memcpy用单位阵覆盖前一轮线性化后的矩阵
    QEKF_INS.IMU_QuaternionEKF.F = Eigen::Map<const Eigen::Matrix<double, 6, 6, Eigen::RowMajor>>(IMU_QuaternionEKF_F);

    QEKF_INS.IMU_QuaternionEKF.F(0, 1) = -halfgxdt;
    QEKF_INS.IMU_QuaternionEKF.F(0, 2) = -halfgydt;
    QEKF_INS.IMU_QuaternionEKF.F(0, 3) = -halfgzdt;

    QEKF_INS.IMU_QuaternionEKF.F(1, 0) = halfgxdt;
    QEKF_INS.IMU_QuaternionEKF.F(1, 2) = halfgzdt;
    QEKF_INS.IMU_QuaternionEKF.F(1, 3) = -halfgydt;

    QEKF_INS.IMU_QuaternionEKF.F(2, 0) = halfgydt;
    QEKF_INS.IMU_QuaternionEKF.F(2, 1) = -halfgzdt;
    QEKF_INS.IMU_QuaternionEKF.F(2, 3) = halfgxdt;

    QEKF_INS.IMU_QuaternionEKF.F(3, 0) = halfgzdt;
    QEKF_INS.IMU_QuaternionEKF.F(3, 1) = halfgydt;
    QEKF_INS.IMU_QuaternionEKF.F(3, 2) = -halfgxdt;

    // accel low pass filter,加速度过一下低通滤波平滑数据,降低撞击和异常的影响
    if (QEKF_INS.UpdateCount == 0) // 如果是第一次进入,需要初始化低通滤波
    {
        QEKF_INS.Accel[0] = ax;
        QEKF_INS.Accel[1] = ay;
        QEKF_INS.Accel[2] = az;
    }
    QEKF_INS.Accel[0] = QEKF_INS.Accel[0] * QEKF_INS.accLPFcoef / (QEKF_INS.dt + QEKF_INS.accLPFcoef) + ax * QEKF_INS.dt / (QEKF_INS.dt + QEKF_INS.accLPFcoef);
    QEKF_INS.Accel[1] = QEKF_INS.Accel[1] * QEKF_INS.accLPFcoef / (QEKF_INS.dt + QEKF_INS.accLPFcoef) + ay * QEKF_INS.dt / (QEKF_INS.dt + QEKF_INS.accLPFcoef);
    QEKF_INS.Accel[2] = QEKF_INS.Accel[2] * QEKF_INS.accLPFcoef / (QEKF_INS.dt + QEKF_INS.accLPFcoef) + az * QEKF_INS.dt / (QEKF_INS.dt + QEKF_INS.accLPFcoef);

    // set z,单位化重力加速度向量
    accelInvNorm = 1.0 / sqrt(QEKF_INS.Accel[0] * QEKF_INS.Accel[0] + QEKF_INS.Accel[1] * QEKF_INS.Accel[1] + QEKF_INS.Accel[2] * QEKF_INS.Accel[2]);
    for (uint8_t i = 0; i < 3; ++i)
    {
        QEKF_INS.IMU_QuaternionEKF.MeasuredVector[i] = QEKF_INS.Accel[i] * accelInvNorm; // 用加速度向量更新量测值
    }

    // get body state
    QEKF_INS.gyro_norm = sqrt(QEKF_INS.Gyro[0] * QEKF_INS.Gyro[0] +
                                        QEKF_INS.Gyro[1] * QEKF_INS.Gyro[1] +
                                        QEKF_INS.Gyro[2] * QEKF_INS.Gyro[2]);
    QEKF_INS.accl_norm = 1.0 / accelInvNorm;

    // 如果角速度小于阈值且加速度处于设定范围内,认为运动稳定,加速度可以用于修正角速度
    // 稍后在最后的姿态更新部分会利用StableFlag来确定
    if (QEKF_INS.gyro_norm < 0.3 && QEKF_INS.accl_norm > 9.8 - 0.5 && QEKF_INS.accl_norm < 9.8 + 0.5)
    {
        QEKF_INS.StableFlag = 1;
    }
    else
    {
        QEKF_INS.StableFlag = 0;
    }

    // set Q R,过程噪声和观测噪声矩阵
    QEKF_INS.IMU_QuaternionEKF.Q(0, 0) = QEKF_INS.Q1 * QEKF_INS.dt;
    QEKF_INS.IMU_QuaternionEKF.Q(1, 1) = QEKF_INS.Q1 * QEKF_INS.dt;
    QEKF_INS.IMU_QuaternionEKF.Q(2, 2) = QEKF_INS.Q1 * QEKF_INS.dt;
    QEKF_INS.IMU_QuaternionEKF.Q(3, 3) = QEKF_INS.Q1 * QEKF_INS.dt;
    QEKF_INS.IMU_QuaternionEKF.Q(4, 4) = QEKF_INS.Q2 * QEKF_INS.dt;
    QEKF_INS.IMU_QuaternionEKF.Q(5, 5) = QEKF_INS.Q2 * QEKF_INS.dt;
    QEKF_INS.IMU_QuaternionEKF.R(0, 0) = QEKF_INS.R;
    QEKF_INS.IMU_QuaternionEKF.R(1, 1) = QEKF_INS.R;
    QEKF_INS.IMU_QuaternionEKF.R(2, 2) = QEKF_INS.R;

    // 调用kalman_filter.c封装好的函数,注意几个User_Funcx_f的调用
    Kalman_Filter_Update(&QEKF_INS.IMU_QuaternionEKF);

    // 归一化四元数 q = [w, x, y, z]
    double norm = sqrt(QEKF_INS.IMU_QuaternionEKF.FilteredValue[0] * QEKF_INS.IMU_QuaternionEKF.FilteredValue[0] + QEKF_INS.IMU_QuaternionEKF.FilteredValue[1] * QEKF_INS.IMU_QuaternionEKF.FilteredValue[1] + QEKF_INS.IMU_QuaternionEKF.FilteredValue[2] * QEKF_INS.IMU_QuaternionEKF.FilteredValue[2] + QEKF_INS.IMU_QuaternionEKF.FilteredValue[3] * QEKF_INS.IMU_QuaternionEKF.FilteredValue[3]);
    if (norm > 1e-6)
    {
        QEKF_INS.IMU_QuaternionEKF.FilteredValue[0] /= norm;
        QEKF_INS.IMU_QuaternionEKF.FilteredValue[1] /= norm;
        QEKF_INS.IMU_QuaternionEKF.FilteredValue[2] /= norm;
        QEKF_INS.IMU_QuaternionEKF.FilteredValue[3] /= norm;
    }

    // 获取融合后的数据,包括四元数和xy零飘值
    QEKF_INS.q[0] = QEKF_INS.IMU_QuaternionEKF.FilteredValue[0];
    QEKF_INS.q[1] = QEKF_INS.IMU_QuaternionEKF.FilteredValue[1];
    QEKF_INS.q[2] = QEKF_INS.IMU_QuaternionEKF.FilteredValue[2];
    QEKF_INS.q[3] = QEKF_INS.IMU_QuaternionEKF.FilteredValue[3];
    QEKF_INS.GyroBias[0] = QEKF_INS.IMU_QuaternionEKF.FilteredValue[4];
    QEKF_INS.GyroBias[1] = QEKF_INS.IMU_QuaternionEKF.FilteredValue[5];
    QEKF_INS.GyroBias[2] = 0; // 大部分时候z轴通天,无法观测yaw的漂移

    // 利用四元数反解欧拉角
    // QEKF_INS.Yaw = atan2(2.0 * (QEKF_INS.q[0] * QEKF_INS.q[3] + QEKF_INS.q[1] * QEKF_INS.q[2]), 2.0 * (QEKF_INS.q[0] * QEKF_INS.q[0] + QEKF_INS.q[1] * QEKF_INS.q[1]) - 1.0) * rad2deg;
    // QEKF_INS.Pitch = atan2(2.0 * (QEKF_INS.q[0] * QEKF_INS.q[1] + QEKF_INS.q[2] * QEKF_INS.q[3]), 2.0 * (QEKF_INS.q[0] * QEKF_INS.q[0] + QEKF_INS.q[3] * QEKF_INS.q[3]) - 1.0) * rad2deg;
    // QEKF_INS.Roll = asinf(-2.0 * (QEKF_INS.q[1] * QEKF_INS.q[3] - QEKF_INS.q[0] * QEKF_INS.q[2])) * rad2deg;
    QEKF_INS.Yaw = atan2(2.0 * (QEKF_INS.q[0] * QEKF_INS.q[3] + QEKF_INS.q[1] * QEKF_INS.q[2]), 1.0 - 2.0 * (QEKF_INS.q[2] * QEKF_INS.q[2] + QEKF_INS.q[3] * QEKF_INS.q[3])) * rad2deg;
    double sinp = 2.0 * (QEKF_INS.q[0] * QEKF_INS.q[2] - QEKF_INS.q[3] * QEKF_INS.q[1]);
    sinp = clamp(sinp, -1.0, 1.0);
    // QEKF_INS.Pitch = asinf(2.0 * (QEKF_INS.q[0] * QEKF_INS.q[2] - QEKF_INS.q[3] * QEKF_INS.q[1])) * rad2deg;
    QEKF_INS.Pitch = asinf(sinp) * rad2deg;
    QEKF_INS.Roll = atan2(2.0 * (QEKF_INS.q[0] * QEKF_INS.q[1] + QEKF_INS.q[2] * QEKF_INS.q[3]), 1.0 - 2.0 * (QEKF_INS.q[1] * QEKF_INS.q[1] + QEKF_INS.q[2] * QEKF_INS.q[2])) * rad2deg;

    // get Yaw total, yaw数据可能会超过360,处理一下方便其他功能使用(如小陀螺)
    if (QEKF_INS.Yaw - QEKF_INS.YawAngleLast > 180.0)
    {
        QEKF_INS.YawRoundCount--;
    }
    else if (QEKF_INS.Yaw - QEKF_INS.YawAngleLast < -180.0)
    {
        QEKF_INS.YawRoundCount++;
    }
    QEKF_INS.YawTotalAngle = 360.0 * QEKF_INS.YawRoundCount + QEKF_INS.Yaw;
    QEKF_INS.YawAngleLast = QEKF_INS.Yaw;
    QEKF_INS.UpdateCount++; // 初始化低通滤波用,计数测试用
}

/**
 * @brief 用于更新线性化后的状态转移矩阵F右上角的一个4x2分块矩阵,稍后用于协方差矩阵P的更新;
 *        并对零漂的方差进行限制,防止过度收敛并限幅防止发散
 *
 * @param kf
 */
static void IMU_QuaternionEKF_F_Linearization_P_Fading(KalmanFilter_t *kf)
{
    static double q0, q1, q2, q3;
    static double qInvNorm;

    q0 = kf->xhatminus(0, 0);
    q1 = kf->xhatminus(1, 0);
    q2 = kf->xhatminus(2, 0);
    q3 = kf->xhatminus(3, 0);
    // quaternion normalize
    qInvNorm = 1.0 / sqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    for (uint8_t i = 0; i < 4; i++)
    {
        kf->xhatminus(i, 0) *= qInvNorm;
    }
    /*  F, number with * represent vals to be set
     0     1     2     3     4*     5*
     6     7     8     9    10*    11*
    12    13    14    15    16*    17*
    18    19    20    21    22*    23*
    24    25    26    27    28     29
    30    31    32    33    34     35
    */
    // set F
    kf->F(0, 4) = q1 * QEKF_INS.dt / 2;
    kf->F(0, 5) = q2 * QEKF_INS.dt / 2;

    kf->F(1, 4) = -q0 * QEKF_INS.dt / 2;
    kf->F(1, 5) = q3 * QEKF_INS.dt / 2;

    kf->F(2, 4) = -q3 * QEKF_INS.dt / 2;
    kf->F(2, 5) = -q0 * QEKF_INS.dt / 2;

    kf->F(3, 4) = q2 * QEKF_INS.dt / 2;
    kf->F(3, 5) = -q1 * QEKF_INS.dt / 2;

    // fading filter,防止零飘参数过度收敛
    kf->F(4, 4) /= QEKF_INS.lambda;
    kf->F(5, 5) /= QEKF_INS.lambda;

    // 限幅,防止发散
    if (kf->P(4, 4) > 10000)
    {
        kf->P(4, 4) = 10000;
    }
    if (kf->P(5, 5) > 10000)
    {
        kf->P(5, 5) = 10000;
    }
}

/**
 * @brief 在工作点处计算观测函数h(x)的Jacobi矩阵H
 *
 * @param kf
 */
static void IMU_QuaternionEKF_SetH(KalmanFilter_t *kf)
{
    // std::cout << "Setting H matrix" << std::endl;
    static double doubleq0, doubleq1, doubleq2, doubleq3;
    /* H
     0     1     2     3     4     5
     6     7     8     9    10    11
    12    13    14    15    16    17
    last two cols are zero
    */
    // set H
    doubleq0 = 2 * kf->xhatminus(0, 0);
    doubleq1 = 2 * kf->xhatminus(1, 0);
    doubleq2 = 2 * kf->xhatminus(2, 0);
    doubleq3 = 2 * kf->xhatminus(3, 0);

    kf->H = Eigen::Matrix<double, Zsize, Xsize>::Zero(Zsize, Xsize);

    kf->H(0, 0) = -doubleq2;
    kf->H(0, 1) = doubleq3;
    kf->H(0, 2) = -doubleq0;
    kf->H(0, 3) = doubleq1;

    kf->H(1, 0) = doubleq1;
    kf->H(1, 1) = doubleq0;
    kf->H(1, 2) = doubleq3;
    kf->H(1, 3) = doubleq2;

    kf->H(2, 0) = doubleq0;
    kf->H(2, 1) = -doubleq1;
    kf->H(2, 2) = -doubleq2;
    kf->H(2, 3) = doubleq3;
}

/**
 * @brief 利用观测值和先验估计得到最优的后验估计
 *        加入了卡方检验以判断融合加速度的条件是否满足
 *        同时引入发散保护保证恶劣工况下的必要量测更新
 *
 * @param kf
 */
static void IMU_QuaternionEKF_xhatUpdate(KalmanFilter_t *kf)
{
    static double q0, q1, q2, q3;

    kf->HT = kf->H.transpose();                          // z|x => x|z
    Eigen::Matrix<double, Zsize, Xsize> temp_matrix = kf->H * kf->Pminus;               // temp_matrix = H·P'(k)
    Eigen::Matrix<double, Zsize, Zsize> temp_matrix1 = temp_matrix * kf->HT; // temp_matrix1 = H·P'(k)·HT
    kf->S = temp_matrix1 + kf->R;                        // S = H P'(k) HT + R
    temp_matrix1 = kf->S.inverse();                      // temp_matrix1 = inv(H·P'(k)·HT + R)

    q0 = kf->xhatminus(0, 0);
    q1 = kf->xhatminus(1, 0);
    q2 = kf->xhatminus(2, 0);
    q3 = kf->xhatminus(3, 0);

    // 计算预测得到的重力加速度方向(通过姿态获取的)
    Eigen::Matrix<double, Zsize, 1> temp_vector = Eigen::Matrix<double, Zsize, 1>::Zero(Zsize, 1);
    temp_vector(0, 0) = 2 * (q1 * q3 - q0 * q2);
    temp_vector(1, 0) = 2 * (q0 * q1 + q2 * q3);
    temp_vector(2, 0) = q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3; // temp_vector = h(xhat'(k))

    // 计算预测值和各个轴的方向余弦
    QEKF_INS.OrientationCosine[0] = acosf(fabsf(temp_vector(0, 0)));
    QEKF_INS.OrientationCosine[1] = acosf(fabsf(temp_vector(1, 0)));
    QEKF_INS.OrientationCosine[2] = acosf(fabsf(temp_vector(2, 0)));

    // 利用加速度计数据修正
    Eigen::Matrix<double, Zsize, 1> temp_vector1 = kf->z - temp_vector; // temp_vector1 = z(k) - h(xhat'(k))

    // chi-square test,卡方检验
    Eigen::Matrix<double, Zsize, 1> temp_matrix2 = temp_matrix1 * temp_vector1; // temp_matrix2 = inv(H·P'(k)·HT + R)·(z(k) - h(xhat'(k)))
    Eigen::Matrix<double, 1, Zsize> temp_vector1T = temp_vector1.transpose();   // temp_vector1T = z(k) - h(xhat'(k))'
    QEKF_INS.ChiSquare = temp_vector1T * temp_matrix2;

    // rk is small,filter converged/converging
    if (QEKF_INS.ChiSquare(0, 0) < 0.5 * QEKF_INS.ChiSquareTestThreshold)
    {
        QEKF_INS.ConvergeFlag = 1;
    }
    // rk is bigger than thre but once converged
    if (QEKF_INS.ChiSquare(0, 0) > QEKF_INS.ChiSquareTestThreshold && QEKF_INS.ConvergeFlag)
    {
        if (QEKF_INS.StableFlag)
        {
            QEKF_INS.ErrorCount++; // 载体静止时仍无法通过卡方检验
        }
        else
        {
            QEKF_INS.ErrorCount = 0;
        }

        if (QEKF_INS.ErrorCount > 50)
        {
            // 滤波器发散
            QEKF_INS.ConvergeFlag = 0;
            kf->SkipEq5 = 0; // step-5 is cov mat P updating
        }
        else
        {
            //  残差未通过卡方检验 仅预测
            //  xhat(k) = xhat'(k)
            //  P(k) = P'(k)
            kf->xhat = kf->xhatminus;
            kf->P = kf->Pminus;
            kf->SkipEq5 = 1; // part5 is P updating
            return;
        }
    }
    else // if divergent or rk is not that big/acceptable,use adaptive gain
    {
        // scale adaptive,rk越小则增益越大,否则更相信预测值
        if (QEKF_INS.ChiSquare(0, 0) > 0.1 * QEKF_INS.ChiSquareTestThreshold && QEKF_INS.ConvergeFlag)
        {
            QEKF_INS.AdaptiveGainScale = (QEKF_INS.ChiSquareTestThreshold - QEKF_INS.ChiSquare(0, 0)) / (0.9 * QEKF_INS.ChiSquareTestThreshold);
        }
        else
        {
            QEKF_INS.AdaptiveGainScale = 1;
        }
        QEKF_INS.ErrorCount = 0;
        kf->SkipEq5 = 0;
    }

    // cal kf-gain K
    Eigen::Matrix<double, Xsize, Zsize> temp_matrix3 = kf->Pminus * kf->HT; // temp_matrix3 = P'(k)·HT
    kf->K = temp_matrix3 * temp_matrix1;

    // implement adaptive
    kf->K *= QEKF_INS.AdaptiveGainScale;

    for (uint8_t i = 0; i < 3; ++i)
    {
        kf->K(4, i) *= QEKF_INS.OrientationCosine[0] / M_PI_2; // 1 rad
        kf->K(5, i) *= QEKF_INS.OrientationCosine[1] / M_PI_2; // 1 rad
    }

    Eigen::Matrix<double, Xsize, 1> temp_vector4 = kf->K * temp_vector1; // temp_vector4 = K(k)·(z(k) - H·xhat'(k))

    // 零漂修正限幅,一般不会有过大的漂移
    if (QEKF_INS.ConvergeFlag)
    {
        for (uint8_t i = 4; i < 6; i++)
        {
            if (temp_vector4(i, 0) > 1e-2 * QEKF_INS.dt)
            {
                temp_vector4(i, 0) = 1e-2 * QEKF_INS.dt;
            }
            if (temp_vector4(i, 0) < -1e-2 * QEKF_INS.dt)
            {
                temp_vector4(i, 0) = -1e-2 * QEKF_INS.dt;
            }
        }
    }

    // 不修正yaw轴数据
    temp_vector4(3, 0) = 0;
    kf->xhat = kf->xhatminus + temp_vector4;
}

/**
 * @brief EKF观测环节,其实就是把数据复制一下
 *
 * @param kf kf类型定义
 */
static void IMU_QuaternionEKF_Observe(KalmanFilter_t *kf)
{
    matrixToRowMajorFixedArray<Xsize, Xsize>(kf->P, IMU_QuaternionEKF_P);
    matrixToRowMajorFixedArray<Xsize, Zsize>(kf->K, IMU_QuaternionEKF_K);
    matrixToRowMajorFixedArray<Zsize, Xsize>(kf->H, IMU_QuaternionEKF_H);
}