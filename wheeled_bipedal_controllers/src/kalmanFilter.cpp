#include "wheeled_bipedal_controllers/kalmanFilter.h"

/**
 * @brief 初始化矩阵维度信息并为矩阵分配空间
 *
 * @param kf kf类型定义
 */
void Kalman_Filter_Init(KalmanFilter_t *kf)
{
    memset(kf->StateMinVariance, 0, sizeof(kf->StateMinVariance));

    kf->SkipEq1 = kf->SkipEq2 = kf->SkipEq3 = kf->SkipEq4 = kf->SkipEq5 = 0;
    return;
}

void Kalman_Filter_Measure(KalmanFilter_t *kf)
{
    kf->z = arrayToMatrixXd(kf->MeasuredVector, Zsize, 1);

    memset(kf->MeasuredVector, 0, sizeof(kf->MeasuredVector));

    // if (Usize > 0)
    //     kf->u = arrayToMatrixXd(kf->ControlVector, Usize, 1);
}

void Kalman_Filter_xhatMinusUpdate(KalmanFilter_t *kf)
{
    if (!kf->SkipEq1)
    {
        if (Usize > 0)
        {
            auto temp_vector = kf->F * kf->xhat;
            auto temp_vector1 = kf->B * kf->u;
            kf->xhatminus = temp_vector + temp_vector1;
        }
        else
        {
            kf->xhatminus = kf->F * kf->xhat;
        }
    }
}

void Kalman_Filter_PminusUpdate(KalmanFilter_t *kf)
{
    if (!kf->SkipEq2)
    {
        kf->FT = kf->F.transpose(); // 转置
        kf->Pminus = kf->F * kf->P;
        auto temp_matrix = kf->Pminus * kf->FT; // temp_matrix = F P(k-1) FT
        kf->Pminus = temp_matrix + kf->Q;
    }
}
void Kalman_Filter_SetK(KalmanFilter_t *kf)
{
    if (!kf->SkipEq3)
    {
        kf->HT = kf->H.transpose();                          // z|x => x|z
        auto temp_matrix = kf->H * kf->Pminus;    // temp_matrix = H·P'(k)
        Eigen::Matrix3d temp_matrix1 = temp_matrix * kf->HT; // temp_matrix1 = H·P'(k)·HT
        kf->S = temp_matrix1 + kf->R;             // S = H P'(k) HT + R
        temp_matrix1 = kf->S.inverse();             // temp_matrix1 = inv(H·P'(k)·HT + R)
        auto temp_matrix2 = kf->Pminus * kf->HT;  // temp_matrix2 = P'(k)·HT
        kf->K = temp_matrix2 * temp_matrix1;
    }
}
void Kalman_Filter_xhatUpdate(KalmanFilter_t *kf)
{
    if (!kf->SkipEq4)
    {
        auto temp_vector = kf->H * kf->xhatminus; // temp_vector = H xhat'(k)
        auto temp_vector1 = kf->z - temp_vector;  // temp_vector1 = z(k) - H·xhat'(k)
        auto temp_vector2 = kf->K * temp_vector1; // temp_vector2 = K(k)·(z(k) - H·xhat'(k))
        kf->xhat = kf->xhatminus + temp_vector2;
    }
}
void Kalman_Filter_P_Update(KalmanFilter_t *kf)
{
    if (!kf->SkipEq5)
    {
        auto temp_matrix = kf->K * kf->H;             // temp_matrix = K(k)·H
        auto temp_matrix1 = temp_matrix * kf->Pminus; // temp_matrix1 = K(k)·H·P'(k)
        kf->P = kf->Pminus - temp_matrix1;
    }
}

/**
 * @brief 执行卡尔曼滤波黄金五式,提供了用户定义函数,可以替代五个中的任意一个环节,方便自行扩展为EKF/UKF/ESKF/AUKF等
 *
 * @param kf kf类型定义
 * @return float* 返回滤波值
 */
float *Kalman_Filter_Update(KalmanFilter_t *kf)
{
    // 0. 获取量测信息
    Kalman_Filter_Measure(kf);
    if (kf->User_Func0_f != NULL)
        kf->User_Func0_f(kf);
    // std::cout << "KF Update 0 finished." << std::endl;

    // 先验估计
    // 1. xhat'(k)= A·xhat(k-1) + B·u
    Kalman_Filter_xhatMinusUpdate(kf);
    if (kf->User_Func1_f != NULL)
        kf->User_Func1_f(kf);
    // std::cout << "KF Update 1 finished." << std::endl;

    // 预测更新
    // 2. P'(k) = A·P(k-1)·AT + Q
    Kalman_Filter_PminusUpdate(kf);
    if (kf->User_Func2_f != NULL)
        kf->User_Func2_f(kf);
    // std::cout << "KF Update 2 finished." << std::endl;

    // 量测更新
    // 3. K(k) = P'(k)·HT / (H·P'(k)·HT + R)
    Kalman_Filter_SetK(kf);

    if (kf->User_Func3_f != NULL)
        kf->User_Func3_f(kf);
    // std::cout << "KF Update 3 finished." << std::endl;

    // 融合
    // 4. xhat(k) = xhat'(k) + K(k)·(z(k) - H·xhat'(k))
    Kalman_Filter_xhatUpdate(kf);

    if (kf->User_Func4_f != NULL)
        kf->User_Func4_f(kf);
    // std::cout << "KF Update 4 finished." << std::endl;

    // 修正方差
    // 5. P(k) = (1-K(k)·H)·P'(k) ==> P(k) = P'(k)-K(k)·H·P'(k)
    Kalman_Filter_P_Update(kf);

    // 自定义函数,可以提供后处理等
    if (kf->User_Func5_f != NULL)
        kf->User_Func5_f(kf);
    // std::cout << "KF Update 5 finished." << std::endl;

    // 避免滤波器过度收敛
    // suppress filter excessive convergence
    for (int i = 0; i < Xsize; ++i)
    {
        if (kf->P(i, i) < kf->StateMinVariance[i])
        {
            kf->P(i, i) = kf->StateMinVariance[i];
        }
    }
    // std::cout << "KF Update 6 finished." << std::endl;
    
    matrixToFloatArray(kf->xhat, kf->FilteredValue);
    
    // std::cout << "KF Update 7 finished." << std::endl;

    if (kf->User_Func6_f != NULL)
        kf->User_Func6_f(kf);

    return kf->FilteredValue;
}