#include "wheeled_bipedal_hardware/serial_core.hpp"

class SerialTest
{
public:
    std::unique_ptr<USBSerial> serial_core_;
    std::mutex state_mutex_;
    std::mutex cmd_mutex_;

    SerialTest()
    {
        // 定义帧头到包长度的映射
        std::unordered_map<uint8_t, size_t> header_map;
        header_map[0x5A] = sizeof(ReceivePackage);
        header_map[0x5B] = sizeof(ReceivePackage2);

        serial_core_ = std::make_unique<USBSerial>("/dev/ttyS0", 921600);
        serial_core_->start_receive_thread(
            [this](const uint8_t *data)
            { this->receiveCallback(data); },
            header_map);
    }

    void receiveCallback(const uint8_t *data)
    {
        // 此时回调的数据已通过 CRC 校验，长度与帧头匹配
        uint8_t header = data[0];

        // 根据帧头安全解析
        switch (header)
        {
        case 0x5A:
        {
            std::lock_guard<std::mutex> lock(state_mutex_);
            auto pkg = reinterpret_cast<const ReceivePackage *>(data);
            std::cout << "received pkg with header: " << std::hex << std::showbase << pkg->header << std::endl;
            break;
        }
        case 0x5B:
        {
            std::lock_guard<std::mutex> lock(state_mutex_);
            auto pkg = reinterpret_cast<const ReceivePackage2 *>(data);
            std::cout << "received pkg with header: " << std::hex << std::showbase << pkg->header << std::endl;
            break;
        }
        default:
            break;
        }
    }

    void sendCommands()
    {
        // 发送关节1位置命令
        if (1)
        {
            SendPackage pkg;
            pkg.header = 0xA5;
            {
                std::lock_guard<std::mutex> lock(cmd_mutex_);
                pkg.aaa = 1; // 从命令接口读取
            }
            uint8_t buffer[sizeof(SendPackage)];
            std::memcpy(buffer, &pkg, sizeof(SendPackage));
            Append_CRC16_Check_Sum(buffer, sizeof(SendPackage));
            serial_core_->send_raw(buffer, sizeof(SendPackage));
        }

        // 发送关节2速度命令
        if (1)
        {
            SendPackage2 pkg;
            pkg.header = 0xA6;
            {
                std::lock_guard<std::mutex> lock(cmd_mutex_);
                pkg.aaa = 2;
            }
            uint8_t buffer[sizeof(SendPackage2)];
            std::memcpy(buffer, &pkg, sizeof(SendPackage2));
            Append_CRC16_Check_Sum(buffer, sizeof(SendPackage2));
            serial_core_->send_raw(buffer, sizeof(SendPackage2));
        }
    }
};

int main()
{
    SerialTest serialTest;
    return 0;
}