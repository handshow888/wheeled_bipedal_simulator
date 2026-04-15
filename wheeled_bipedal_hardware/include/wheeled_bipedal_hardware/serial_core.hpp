#pragma once
#include <atomic>
#include <cstdint>
#include <fcntl.h>
#include <functional>
#include <future>
#include <iostream>
#include <memory>
#include <mutex>
#include <string_view>
#include <termios.h>
#include <thread>
#include <unistd.h>
#include <errno.h>
#include <stdio.h>
#include <string.h>
#include <cstdio>
#include <cstring>

#include "crc.hpp"
#include "packages.hpp"

speed_t baudRateToSpeed(int baud_rate);

// 定义一个通用的包处理器类型
using ReceiveCallback = std::function<void(const uint8_t *data)>;

class USBSerial
{
private:
    int serial_port_;
    termios tty_old;
    uint8_t buffer_[512];
    std::thread receive_thread_;
    std::mutex send_mutex_;
    std::mutex receive_mutex_;
    std::atomic<bool> running_{false}; // 添加运行状态标志

public:
    USBSerial(const std::string &port_name)
    {
        initializeSerial(port_name, B115200);
    }

    USBSerial(const std::string &port_name, int baud_rate)
    {
        speed_t speed = baudRateToSpeed(baud_rate);
        initializeSerial(port_name, speed);
    }

    void initializeSerial(const std::string &port_name, speed_t baud_rate)
    {
        serial_port_ = open(port_name.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
        if (serial_port_ < 0)
        {
            std::cerr << "1Error: " << strerror(errno) << std::endl;
            throw std::runtime_error("Failed to open serial port");
        }
        termios tty;
        std::memset(&tty, 0, sizeof(tty));
        if (tcgetattr(serial_port_, &tty) != 0)
        {
            std::cerr << "2Error: " << strerror(errno) << std::endl;
            throw std::runtime_error("Failed to get serial port attributes");
        }
        tty_old = tty;

        cfsetospeed(&tty, baud_rate);
        cfsetispeed(&tty, baud_rate);

        tty.c_cflag |= (CLOCAL | CREAD);                  // 忽略调制解调器状态，允许从串口接收数据
        tty.c_iflag &= ~(INPCK | ISTRIP);                 // 禁用奇偶校验，保留8位数据位
        tty.c_cflag &= ~(PARENB | PARODD);                // 禁用奇偶校验
        tty.c_cflag &= ~CSTOPB;                           // 设置为1个停止位
        tty.c_iflag &= ~(IXON | IXOFF | IXANY);           // 禁用软件流控
        tty.c_cflag &= ~CRTSCTS;                          // 禁用硬件流控
        tty.c_oflag &= ~OPOST;                            // 禁用输出处理
        tty.c_iflag &= ~(INLCR | IGNCR | ICRNL | IGNBRK); // 禁用输入处理
        tty.c_cflag |= CS8;                               // 设置数据位为8位
        tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ECHOK | ECHONL | ISIG |
                         IEXTEN); // 设置为原始模式
        tty.c_cc[VMIN] = 0;       // 无最小字节数
        tty.c_cc[VTIME] = 0;      // 无超时

        if (tcsetattr(serial_port_, TCSANOW, &tty) != 0)
        {
            std::cerr << "3Error: " << strerror(errno) << std::endl;
            throw std::runtime_error("Failed to set serial port attributes");
        }
    }

    ~USBSerial()
    {
        running_ = false;
        if (receive_thread_.joinable())
        {
            receive_thread_.join();
        }
        tcsetattr(serial_port_, TCSANOW, &tty_old);
        close(serial_port_);
    }

    // 通用发送接口：直接发送原始字节数组（调用方负责组包和 CRC）
    void send_raw(const uint8_t *data, size_t len)
    {
        std::ignore = std::async(std::launch::async, [this, data, len]()
                                 {
            try {
                std::lock_guard<std::mutex> lock(send_mutex_);
                ssize_t ret = write(serial_port_, data, len);
                if (ret < 0 && errno != EAGAIN && errno != EWOULDBLOCK) {
                    std::cerr << "Write error: " << strerror(errno) << std::endl;
                } 
            } catch (const std::exception& e) {
                std::cerr << "Exception in send: " << e.what() << std::endl;
            } });
    }

    void start_receive_thread(ReceiveCallback callback, const std::unordered_map<uint8_t, size_t> &header_to_length)
    {
        if (header_to_length.empty())
        {
            throw std::invalid_argument("header_to_length map cannot be empty");
        }
        running_ = true;
        receive_thread_ = std::thread(&USBSerial::receiveLoop, this,
                                      std::move(callback), header_to_length);
    }

    void receiveLoop(ReceiveCallback callback, const std::unordered_map<uint8_t, size_t> &header_to_length)
    {
        // 预先计算最小包长度（避免每次循环都遍历 map）
        size_t min_packet_len = std::min_element(header_to_length.begin(), header_to_length.end(),
                                                 [](const auto &a, const auto &b)
                                                 { return a.second < b.second; })
                                    ->second;

        std::vector<uint8_t> buffer;
        uint8_t temp_buffer[1024];

        // 错误计数器
        size_t crc_errors = 0;
        size_t read_errors = 0;
        size_t sync_errors = 0;
        size_t last_logged_crc = 0;

        while (running_)
        {
            ssize_t len = read(serial_port_, temp_buffer, sizeof(temp_buffer));
            if (len > 0)
            {
                buffer.insert(buffer.end(), temp_buffer, temp_buffer + len);

                // 仅当缓冲区数据量 ≥ 最小包长度时，才尝试解析
                while (buffer.size() >= min_packet_len)
                {
                    uint8_t header = buffer[0];
                    auto it = header_to_length.find(header); // 寻找匹配的帧头

                    if (it != header_to_length.end()) // 如果找到帧头
                    {
                        size_t required_len = it->second; // 记录结构体长度
                        if (buffer.size() >= required_len)
                        {
                            // 检查 CRC
                            if (Verify_CRC16_Check_Sum(buffer.data(), required_len))
                            {
                                // 有效帧：交给上层
                                callback(buffer.data());
                                // 移除该帧
                                buffer.erase(buffer.begin(), buffer.begin() + required_len);
                                continue; // 继续尝试解析下一帧
                            }
                            else // CRC校验不通过
                            {
                                ++crc_errors;
                            }
                        }
                        else // 数据不够，等待下一次读取
                        {
                            break;
                        }
                    }

                    // 帧头未识别 或 CRC 校验失败：丢弃第一个字节，滑动窗口
                    buffer.erase(buffer.begin());
                }
            }
            else if (len < 0)
            {
                // 处理读取错误
                if (errno != EAGAIN && errno != EWOULDBLOCK)
                {
                    ++read_errors;
                    if (read_errors % 100 == 0)
                    {
                        std::cerr << "Read error: " << strerror(errno)
                                  << " (count: " << read_errors << ")" << std::endl;
                    }
                }
            }

            // 控制日志输出频率
            if (crc_errors > last_logged_crc + 50)
            {
                std::cerr << "CRC errors: " << crc_errors
                          << ", Sync errors: " << sync_errors << std::endl;
                last_logged_crc = crc_errors;
            }

            // 避免 CPU 满载（根据实际数据速率调整）
            // std::this_thread::sleep_for(std::chrono::microseconds(1));
        }
    }
};

speed_t baudRateToSpeed(int baud_rate)
{
    switch (baud_rate)
    {
    case 0:
        return B0;
    case 50:
        return B50;
    case 75:
        return B75;
    case 110:
        return B110;
    case 134:
        return B134;
    case 150:
        return B150;
    case 200:
        return B200;
    case 300:
        return B300;
    case 600:
        return B600;
    case 1200:
        return B1200;
    case 1800:
        return B1800;
    case 2400:
        return B2400;
    case 4800:
        return B4800;
    case 9600:
        return B9600;
    case 19200:
        return B19200;
    case 38400:
        return B38400;
    case 57600:
        return B57600;
    case 115200:
        return B115200;
    case 230400:
        return B230400;
    case 460800:
        return B460800;
    case 500000:
        return B500000;
    case 576000:
        return B576000;
    case 921600:
        return B921600;
    case 1000000:
        return B1000000;
    case 1152000:
        return B1152000;
    case 1500000:
        return B1500000;
    case 2000000:
        return B2000000;
    case 2500000:
        return B2500000;
    case 3000000:
        return B3000000;
    case 3500000:
        return B3500000;
    case 4000000:
        return B4000000;
    default:
        throw std::invalid_argument("Unsupported baud rate: " + std::to_string(baud_rate));
    }
}