#ifndef PACKAGES_HPP
#define PACKAGES_HPP

#include <cstdint>
#include <vector>

struct ReceivePackage
{
    uint8_t header = 0x5A;
    float ax;
    float ay;
    float az;
    float gx;
    float gy;
    float gz;
    uint16_t crc16 = 0xFFFF;
} __attribute__((packed));

struct ReceivePackage2
{
    uint8_t header = 0x5B;
    uint8_t motorID; // 左轮5 右轮6
    // float motorPos; // rad [-4pi, 4pi]
    float motorVel; // rad/s [-45.0, 45.0]
    // float motorTor; // Nm
    uint16_t crc16 = 0xFFFF;
} __attribute__((packed));

struct ReceivePackage3
{
    uint8_t header = 0x5C;
    uint8_t jointMotorState = 0; // 0失能 1使能
    uint16_t crc16 = 0xFFFF;
} __attribute__((packed));

struct SendPackage
{
    uint8_t header = 0xA5;
    float motors_effort[2] = {0.0}; // LW RW
    uint16_t crc16 = 0xFFFF;
} __attribute__((packed));

// struct SendPackage2
// {
//     uint8_t header = 0xA6;
//     float aaa;
//     // others
//     uint16_t crc16 = 0xFFFF;
// } __attribute__((packed));


#endif