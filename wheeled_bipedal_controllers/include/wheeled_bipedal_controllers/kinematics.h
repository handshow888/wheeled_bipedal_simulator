#pragma once
#include "utils.h"

// 连杆长度参数
#define L1 0.150 // 后大腿长
#define L2 0.250 // 后小腿长
#define L3 0.250 // 前小腿长
#define L4 0.150 // 前大腿长
#define L5 0.120 // 关节电机水平距离

typedef struct
{
    float x, y;
} Point;

Point forwardKinematics(float phi1, float phi4);

inline float wrapTo2Pi(float radian);