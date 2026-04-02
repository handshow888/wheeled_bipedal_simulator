#ifndef STRUCTURAL_PARAMS_H
#define STRUCTURAL_PARAMS_H

/* 记录结构上的参数 */

#include <vector>

extern std::vector<double> rodLengths;
extern double wheelRadius, wheelSeparation;
extern double legLengthMin, legLengthMax;
extern double wheelMass;

// 连杆长度参数
#define L1 rodLengths[0] // 后大腿长
#define L2 rodLengths[1] // 后小腿长
#define L3 rodLengths[2] // 前小腿长
#define L4 rodLengths[3] // 前大腿长
#define L5 rodLengths[4] // 关节电机水平距离

// #define L1 0.150 // 后大腿长
// #define L2 0.250 // 后小腿长
// #define L3 0.250 // 前小腿长
// #define L4 0.150 // 前大腿长
// #define L5 0.120 // 关节电机水平距离

#endif