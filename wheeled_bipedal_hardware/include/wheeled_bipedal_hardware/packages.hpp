#pragma once
#include <cstdint>
#include <vector>

struct ReceivePackage
{
    uint8_t header = 0x5A;
    float aaa;
    // others
    uint16_t crc16 = 0xFFFF; // crc16校验
} __attribute__((packed));

struct SendPackage
{
    uint8_t header = 0xA5;
    float aaa;
    // others
    uint16_t crc16 = 0xFFFF; // crc16校验
} __attribute__((packed));
