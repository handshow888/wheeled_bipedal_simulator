#include "wheeled_bipedal_controllers/utils.h"

float lowPassFilter(float currentValue, float previousValue, float alpha)
{
  return alpha * currentValue + (1 - alpha) * previousValue;
}

/**
 * @brief 自定义1/sqrt(x),速度更快
 *
 * @param x x
 * @return float
 */
float invSqrt(float x)
{
  float halfx = 0.5f * x;
  float y = x;
  uint32_t i;
  memcpy(&i, &y, sizeof(y)); // 安全地将 float 位模式转为 uint32_t
  i = 0x5f375a86U - (i >> 1);
  memcpy(&y, &i, sizeof(y)); // 转回 float
  y = y * (1.5f - (halfx * y * y));
  return y;
}

/**
 * @brief        Convert eular angle to quaternion
 */
void EularAngleToQuaternion(float Yaw, float Pitch, float Roll, float *q)
{
  float cosPitch, cosYaw, cosRoll, sinPitch, sinYaw, sinRoll;
  Yaw /= 57.295779513f;
  Pitch /= 57.295779513f;
  Roll /= 57.295779513f;
  cosPitch = cos(Pitch / 2);
  cosYaw = cos(Yaw / 2);
  cosRoll = cos(Roll / 2);
  sinPitch = sin(Pitch / 2);
  sinYaw = sin(Yaw / 2);
  sinRoll = sin(Roll / 2);
  q[0] = cosPitch * cosRoll * cosYaw + sinPitch * sinRoll * sinYaw;
  q[1] = sinPitch * cosRoll * cosYaw - cosPitch * sinRoll * sinYaw;
  q[2] = sinPitch * cosRoll * sinYaw + cosPitch * sinRoll * cosYaw;
  q[3] = cosPitch * cosRoll * sinYaw - sinPitch * sinRoll * cosYaw;
}

/**
 * @brief        Convert quaternion to eular angle
 */
void QuaternionToEularAngle(float *q, float *Yaw, float *Pitch, float *Roll)
{
  *Yaw = atan2f(2.0f * (q[0] * q[3] + q[1] * q[2]), 2.0f * (q[0] * q[0] + q[1] * q[1]) - 1.0f) * 57.295779513f;
  *Pitch = atan2f(2.0f * (q[0] * q[1] + q[2] * q[3]), 2.0f * (q[0] * q[0] + q[3] * q[3]) - 1.0f) * 57.295779513f;
  *Roll = asinf(2.0f * (q[0] * q[2] - q[1] * q[3])) * 57.295779513f;
}


Eigen::MatrixXd arrayToMatrixXd(const float* data, int rows, int cols, bool rowMajor) {
    if (rowMajor) {
        return Eigen::Map<const Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>(
            data, rows, cols).cast<double>();
    } else {
        return Eigen::Map<const Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic, Eigen::ColMajor>>(
            data, rows, cols).cast<double>();
    }
}

// 写入预分配的 float 数组（调用者负责内存）
void matrixToFloatArray(const Eigen::MatrixXd& vec, float* output) {
    if (vec.cols() != 1 || output == nullptr) {
        throw std::invalid_argument("Invalid input: must be Mx1 and output != nullptr.");
    }
    for (int i = 0; i < vec.rows(); ++i) {
        output[i] = static_cast<float>(vec(i, 0));
    }
}

void matrixToRowMajorFloatArray(const Eigen::MatrixXd& mat, float* output) {
    if (output == nullptr) {
        throw std::invalid_argument("Output buffer is null.");
    }
    if (mat.size() == 0) return;

    const int rows = mat.rows();
    const int cols = mat.cols();

    for (int i = 0; i < rows; ++i) {
        for (int j = 0; j < cols; ++j) {
            output[i * cols + j] = static_cast<float>(mat(i, j));
        }
    }
}