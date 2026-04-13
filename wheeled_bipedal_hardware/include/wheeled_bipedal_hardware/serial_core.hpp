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
  USBSerial(const std::string &port_name) noexcept
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

    cfsetospeed(&tty, B115200);
    cfsetispeed(&tty, B115200);

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

  void send(const SendPackage &package)
  {
    std::async(std::launch::async, [this, package]()
               {
            try {
                uint8_t packet[sizeof(SendPackage)];
                std::copy(reinterpret_cast<const uint8_t *>(&package),
                         reinterpret_cast<const uint8_t *>(&package) + 
                         sizeof(SendPackage),
                         packet);
                Append_CRC16_Check_Sum(packet, sizeof(SendPackage));
                std::lock_guard<std::mutex> lock(send_mutex_);
                ssize_t len = write(serial_port_, &packet, sizeof(SendPackage));
                if (len < 0) {
                    if (errno != EAGAIN && errno != EWOULDBLOCK) {
                        std::cerr << "Write error: " << strerror(errno) << std::endl;
                    }
                }
            } catch (const std::exception& e) {
                std::cerr << "Exception in send: " << e.what() << std::endl;
            } });
  }

  void start_receive_thread(std::function<void(const ReceivePackage &)> callback)
  {
    running_ = true;
    receive_thread_ = std::thread([this, callback]()
                                  {
            constexpr size_t package_size = sizeof(ReceivePackage);
            std::vector<uint8_t> buffer;
            size_t buffer_size = 0;
            uint8_t temp_buffer[512];
            
            // 错误计数器
            size_t crc_errors = 0;
            size_t read_errors = 0;
            size_t incomplete_errors = 0;
            size_t sync_errors = 0;
            size_t last_logged_crc = 0;

            while (running_) {
                // 非阻塞读取
                ssize_t len = read(serial_port_, temp_buffer, sizeof(temp_buffer));
                
                if (len > 0) {
                    // 添加到缓冲区
                    buffer.insert(buffer.end(), temp_buffer, temp_buffer + len);
                    buffer_size = buffer.size();
                    
                    // 处理缓冲区中的完整包
                    while (buffer_size >= package_size) {
                        // 查找可能的帧头（根据协议实现）
                        size_t start_index = 0;
                        bool found_package = false;
                        
                        for (; start_index <= buffer_size - package_size; ++start_index) {
                            // 检查CRC校验
                            if (Verify_CRC16_Check_Sum(buffer.data() + start_index, package_size)) {
                                found_package = true;
                                break;
                            }
                        }
                        
                        if (found_package) {
                            // 处理有效包
                            callback(*reinterpret_cast<ReceivePackage*>(buffer.data() + start_index));
                            
                            // 移除已处理数据
                            size_t bytes_to_remove = start_index + package_size;
                            buffer.erase(buffer.begin(), buffer.begin() + bytes_to_remove);
                            buffer_size = buffer.size();
                        } else {
                            // 未找到有效包，丢弃无效数据
                            if (buffer_size > package_size) {
                                buffer.erase(buffer.begin(), buffer.begin() + (buffer_size - package_size + 1));
                                buffer_size = buffer.size();
                                sync_errors++;
                            } else {
                                break; // 保留数据等待更多输入
                            }
                        }
                    }
                } else if (len < 0) {
                    // 处理读取错误
                    if (errno != EAGAIN && errno != EWOULDBLOCK) {
                        read_errors++;
                        if (read_errors % 100 == 0) {
                            std::cerr << "Read error: " << strerror(errno) 
                                      << " (count: " << read_errors << ")" << std::endl;
                        }
                    }
                }
                
                // 控制日志输出频率
                if (crc_errors > last_logged_crc + 50) {
                    std::cerr << "CRC errors: " << crc_errors 
                              << ", Sync errors: " << sync_errors 
                              << ", Incomplete: " << incomplete_errors << std::endl;
                    last_logged_crc = crc_errors;
                }
                
                // 避免忙等待
                // std::this_thread::sleep_for(std::chrono::milliseconds(1));
            } });
  }
};
