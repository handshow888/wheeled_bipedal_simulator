#include "wheeled_bipedal_controllers/utils.h"

double lowPassFilter(double currentValue, double previousValue, double alpha)
{
    return alpha * currentValue + (1 - alpha) * previousValue;
}

/**
 * @brief        Convert eular angle to quaternion
 */
void EularAngleToQuaternion(double Yaw, double Pitch, double Roll, double *q)
{
    double cosPitch, cosYaw, cosRoll, sinPitch, sinYaw, sinRoll;
    Yaw /= rad2deg;
    Pitch /= rad2deg;
    Roll /= rad2deg;
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
void QuaternionToEularAngle(double *q, double *Yaw, double *Pitch, double *Roll)
{
    *Yaw = atan2(2.0 * (q[0] * q[3] + q[1] * q[2]), 2.0 * (q[0] * q[0] + q[1] * q[1]) - 1.0) * rad2deg;
    *Pitch = atan2(2.0 * (q[0] * q[1] + q[2] * q[3]), 2.0 * (q[0] * q[0] + q[3] * q[3]) - 1.0) * rad2deg;
    *Roll = asinf(2.0 * (q[0] * q[2] - q[1] * q[3])) * rad2deg;
}