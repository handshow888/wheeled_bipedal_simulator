/*
 * Copyright 2025 handshow
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "wheeled_bipedal_hardware/CanSerialCore.hpp"

CanSerial::CanSerial(const std::string &interface)
    : can_interface_(interface),
      stream_(io_service_)
{
}

CanSerial::~CanSerial()
{
    running_ = false;
    tx_cv_.notify_all();
    if (tx_thread_.joinable())
        tx_thread_.join();

    if (sock_ >= 0)
        close(sock_);
    io_service_.stop();
    if (io_thread_.joinable())
    {
        io_thread_.join();
        std::cout << "Boost.Asio线程已停止" << std::endl;
    }
}

void CanSerial::init()
{
    // Socket初始化， 创建CAN套接字
    sock_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (sock_ < 0)
        throw std::runtime_error("socket failed");

    int loopback = 0; // 本地回环模式
    setsockopt(sock_, SOL_CAN_RAW, CAN_RAW_LOOPBACK, &loopback, sizeof(loopback));

    // 绑定接口
    ifreq ifr{};
    strncpy(ifr.ifr_name, can_interface_.c_str(), IFNAMSIZ);
    if (ioctl(sock_, SIOCGIFINDEX, &ifr) < 0)
        throw std::runtime_error("获取接口索引失败: " + std::string(strerror(errno)));

    sockaddr_can addr{};
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;
    if (bind(sock_, reinterpret_cast<sockaddr *>(&addr), sizeof(addr)) < 0)
        throw std::runtime_error("绑定失败: " + std::string(strerror(errno)));

    // 分配到Boost流
    stream_.assign(sock_);

    // 增加发送缓冲区
    int sndbuf_size = 1024 * 1024; // 1 MB
    setsockopt(sock_, SOL_SOCKET, SO_SNDBUF, &sndbuf_size, sizeof(sndbuf_size));

    // 设置非阻塞模式
    stream_.non_blocking(true); // boost::asio 非阻塞模式
}

void CanSerial::async_read()
{
    stream_.async_read_some(
        boost::asio::buffer(&recv_frame_, sizeof(can_frame)),
        [this](auto &&...args)
        { handle_received(args...); });
}

void CanSerial::handle_received(const boost::system::error_code &ec, size_t bytes)
{
    if (!ec && bytes == sizeof(can_frame))
    {
        if (frame_callback_)
            frame_callback_(recv_frame_);
        async_read();
    }
    else if (ec)
    {
        std::cout << "读取错误: " << ec.message() << std::endl;
    }
}

void CanSerial::start_io_service()
{
    io_thread_ = std::thread([this]()
                             {  
        std::cout << "启动Boost.Asio事件循环..." << std::endl;
        io_service_.run();  // 阻塞，直到io_service_.stop()被调用
        std::cout << "Boost.Asio事件循环已停止" << std::endl; });

    // 新增发送线程
    tx_thread_ = std::thread(&CanSerial::tx_loop, this);
    std::cout << "发送线程已启动" << std::endl;
}

void CanSerial::tx_loop()
{
    while (running_)
    {
        std::unique_lock<std::mutex> lock(tx_mutex_);
        tx_cv_.wait(lock, [this]
                    { return !tx_queue_.empty() || !running_; });
        if (!running_)
            break;

        // 取出队首帧
        can_frame frame = tx_queue_.front();
        tx_queue_.pop();
        lock.unlock();

        // 实际发送（阻塞但只阻塞发送线程）
        boost::system::error_code ec;
        while (running_)
        {
            size_t len = stream_.write_some(boost::asio::buffer(&frame, sizeof(frame)), ec);
            (void)len;
            if (!ec)
            {
                break; // 发送成功
            }
            else if (ec == boost::asio::error::would_block ||
                     ec == boost::asio::error::try_again ||
                     ec.value() == ENOBUFS)
            {
                // 缓冲区短暂满，等待一小段时间再试
                std::this_thread::sleep_for(std::chrono::microseconds(100));
            }
            else
            {
                std::cerr << "[CanSerial] Send error: " << ec.message() << std::endl;
                break;
            }
        }
    }
}

// 新的 send_frame：只负责入队
void CanSerial::send_frame(const can_frame &frame)
{
    if (!running_)
        return;
    std::lock_guard<std::mutex> lock(tx_mutex_);
    // 限制队列大小，超出时丢弃最旧的帧（保证实时性）
    const size_t max_queue = 10;
    if (tx_queue_.size() >= max_queue)
    {
        tx_queue_.pop(); // 丢旧帧
    }
    tx_queue_.push(frame);
    tx_cv_.notify_one();
}

void CanSerial::send_frame_sync(const can_frame &frame)
{
    for (int retry = 0; retry < 100; ++retry)
    {
        ssize_t n = ::write(sock_, &frame, sizeof(frame));
        if (n == sizeof(frame))
        {
            return; // 成功
        }
        if (n < 0)
        {
            if (errno == ENOBUFS || errno == EAGAIN || errno == EWOULDBLOCK)
            {
                // 缓冲区暂满，等待 50us 后重试
                std::this_thread::sleep_for(std::chrono::microseconds(50));
                continue;
            }
            else
            {
                std::cerr << "[CanSerial] Sync send fatal error: " << strerror(errno) << std::endl;
                break;
            }
        }
    }
    std::cerr << "[CanSerial] Sync send failed after retries" << std::endl;
}

void CanSerial::set_frame_callback(FrameCallback callback)
{
    frame_callback_ = std::move(callback);
}