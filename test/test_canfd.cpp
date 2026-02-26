/**
 * @Author: ZZSZSZSZZ
 * @CreateTime: 2026-02-20
 * @Description:
 * @Version: 1.0
 */

#include <iostream>
#include <chrono>
#include <thread>
#include <iomanip>

#include "can/can_socket_impl.hpp"
#include "can/can_frame_router.hpp"
#include "can/can_receiver.hpp"
#include "can/can_coding.hpp"
#include "common/common.hpp"

using namespace robot::can;

// 自定义日志回调（带颜色）
void logCallback(const std::string &level, const std::string &msg) {
    static std::mutex cout_mutex;
    std::lock_guard<std::mutex> lock(cout_mutex);
    if (level == "ERROR")
        std::cout << "\033[31m[" << level << "]\033[0m " << msg << std::endl;
    else if (level == "WARN")
        std::cout << "\033[33m[" << level << "]\033[0m " << msg << std::endl;
    else if (level == "INFO")
        std::cout << "\033[32m[" << level << "]\033[0m " << msg << std::endl;
    else
        std::cout << "[" << level << "] " << msg << std::endl;
}

// 辅助函数：字节数组转十六进制字符串
std::string bytesToHex(const std::vector<uint8_t> &data) {
    std::ostringstream oss;
    oss << std::hex << std::setfill('0');
    for (uint8_t b: data) {
        oss << std::setw(2) << static_cast<int>(b) << " ";
    }
    return oss.str();
}

// 通用设备类，支持配置是否扩展ID、是否FD
class GenericDevice : public CANDevice {
public:
    GenericDevice(uint32_t id, const std::string &name, bool extended, bool fd)
        : recv_id_(id), name_(name), extended_(extended), fd_(fd) {
    }

    void onFrameReceived(const CANFrame &frame) override {
        std::string typeStr;
        if (frame.format.type == CANFrameType::CanFd)
            typeStr = "CAN FD";
        else if (frame.format.isExtendedId)
            typeStr = "Extended";
        else
            typeStr = "Standard";

        Logger::info(name_ + " received [" + typeStr + "] frame: ID=0x" +
                     toHex(frame.id) + ", DLC=" + std::to_string(frame.format.dlc) +
                     ", data=" + bytesToHex(frame.data) +
                     (frame.format.bitRateSwitch ? " BRS=1" : ""));
    }

    std::string toHex(uint32_t val) {
        std::stringstream ss;
        ss << std::hex << val;
        return ss.str();
    }

    bool isExtendedId() const override { return extended_; }
    uint32_t getReceiveId() const override { return recv_id_; }
    std::string getName() const override { return name_; }

    CANDeviceFrameRequirement getFrameRequirement() const override {
        if (fd_) {
            // CAN FD 设备：首选 FD，扩展ID取决于参数，最大64字节
            return {CANFrameType::CanFd, extended_, 64, true};
        } else {
            // 非 FD 设备：如果是扩展ID则用 Extended，否则 Standard，最大8字节
            return {
                extended_ ? CANFrameType::Extended : CANFrameType::Standard,
                extended_, 8, false
            };
        }
    }

private:
    uint32_t recv_id_;
    std::string name_;
    bool extended_;
    bool fd_;
};

int main() {
    // 设置日志回调
    Logger::setCallback(logCallback);

    // 1. 创建并打开 CAN 套接字（启用 FD 支持，若硬件不支持则后面会失败）
    auto socket = std::make_shared<LinuxCANSocket>();
    Result<void> openRes = socket->open("can0", true); // 启用 CAN FD
    if (openRes.isError()) {
        Logger::error("Failed to open CAN socket: " + openRes.error().toString());
        Logger::warn("If using vcan0, ensure it's created and try again.");
        return -1;
    }
    Logger::info("CAN socket opened on " + socket->getInterfaceName() +
                 ", FD support: " + std::string(socket->supportsCanFd() ? "yes" : "no"));

    // 2. 创建路由器
    auto router = std::make_shared<CANFrameRouter>();

    // 3. 创建设备（分别对应标准、扩展、CAN FD）
    auto dev_std = std::make_shared<GenericDevice>(0x123, "StdDevice", false, false);
    auto dev_ext = std::make_shared<GenericDevice>(0x12ABCDE, "ExtDevice", true, false);
    auto dev_std_fd = std::make_shared<GenericDevice>(0x456, "StdFDDevice", false, true);
    auto dev_ext_fd = std::make_shared<GenericDevice>(0x89ABCDE, "ExtFDDevice", true, true);

    router->registerDevice(dev_std);
    router->registerDevice(dev_ext);
    router->registerDevice(dev_std_fd);
    router->registerDevice(dev_ext_fd);
    Logger::info("Devices registered, total: " + std::to_string(router->deviceCount()));

    // 4. 创建接收线程并启动
    auto receiver = std::make_shared<CANReceiver>(socket, router);
    Result<void> start_res = receiver->start();
    if (start_res.isError()) {
        Logger::error("Failed to start receiver: " + start_res.error().toString());
        socket->close();
        return -1;
    }

    // 5. 主线程发送测试帧
    Logger::info("Sending test frames...");

    // 标准帧 (ID 0x123)
    CANFrame std_frame = CANFrame::makeStandard(0x123, {0x11, 0x22, 0x33, 0x44});
    Result<void> write_std = socket->writeFrame(std_frame);
    if (write_std.isError())
        Logger::error("Write standard frame failed: " + write_std.error().toString());
    else
        Logger::info("Standard frame sent");

    // 扩展帧 (ID 0x12ABCDE)
    CANFrame ext_frame = CANFrame::makeExtended(0x12ABCDE, {0xAA, 0xBB, 0xCC, 0xDD});
    Result<void> write_ext = socket->writeFrame(ext_frame);
    if (write_ext.isError())
        Logger::error("Write extended frame failed: " + write_ext.error().toString());
    else
        Logger::info("Extended frame sent");

    // CAN FD 帧 (如果套接字支持FD)
    if (socket->supportsCanFd()) {
        // 构造50字节数据（超过8字节，触发FD）
        std::vector<uint8_t> fdData(50);
        for (size_t i = 0; i < fdData.size(); ++i)
            fdData[i] = static_cast<uint8_t>(i);

        // 标准FD帧 (ID 0x456)
        CANFrame std_fd_frame = CANFrame::makeCanFd(0x456, fdData, false, true); // 扩展ID, 启用BRS
        Result<void> write_std_fd = socket->writeFrame(std_fd_frame);
        if (write_std_fd.isError())
            Logger::error("Write standard CAN FD frame failed: " + write_std_fd.error().toString());
        else
            Logger::info("standard CAN FD frame sent (" + std::to_string(fdData.size()) + " bytes)");

        // 扩展帧 (ID 0x89ABCDE)
        CANFrame ext_fd_frame = CANFrame::makeCanFd(0x89ABCDE, fdData, true, true); // 扩展ID, 启用BRS
        Result<void> write_ext_fd = socket->writeFrame(ext_fd_frame);
        if (write_ext_fd.isError())
            Logger::error("Write CAN FD frame failed: " + write_ext_fd.error().toString());
        else
            Logger::info("CAN FD frame sent (" + std::to_string(fdData.size()) + " bytes)");
    } else {
        Logger::warn("Socket does not support CAN FD, skipping FD frame test.");
    }

    // 等待接收线程处理（通常100ms足够）
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    // 6. 测试编码器/解码器（可选）
    Logger::info("Testing FrameEncoder/Decoder...");
    std::vector<uint8_t> payload = {0x01, 0x02, 0x03, 0x04, 0x05};
    CANDeviceFrameRequirement req = dev_std->getFrameRequirement();
    CANFrame encoded = CANFrameEncoder::encode(payload, 0x123, req);
    Logger::info("Encoded frame DLC: " + std::to_string(encoded.format.dlc) +
                 ", data size: " + std::to_string(encoded.data.size()));

    bool valid = CANFrameDecoder::validate(encoded, req);
    Logger::info("Frame validation: " + std::string(valid ? "OK" : "FAIL"));

    std::vector<uint8_t> extracted = CANFrameDecoder::extractPayload(encoded);
    Logger::info("Extracted payload size: " + std::to_string(extracted.size()));

    // 7. 停止接收并关闭
    receiver->stop();
    socket->close();
    Logger::info("Test finished.");

    return 0;
}
