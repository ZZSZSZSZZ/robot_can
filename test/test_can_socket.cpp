/**
 * @Author: ZZSZSZSZZ
 * @CreateTime: 2026-02-21
 * @Description: 测试 CAN Socket 底层通讯
 * @Version: 1.0
 */

#include <memory>

#include "can/can_socket_impl.hpp"
#include "common/logger.hpp"

using namespace robot::can;
using namespace robot::common;

int main() {
    // 创建 socket
    auto socket = std::make_shared<LinuxCANSocket>();
    auto res = socket->open("can0", false);
    if (res.isError()) {
        Logger::error("Open failed: " + res.error().toString());
        return -1;
    }
    Logger::info("Socket opened");

    // 发送一帧
    CANFrame frame = CANFrame::makeStandard(0x123, {0x11, 0x22, 0x33, 0x44});
    auto writeRes = socket->writeFrame(frame);
    if (writeRes.isError()) {
        Logger::error("Write failed: " + writeRes.error().toString());
    } else {
        Logger::info("Frame sent");
    }

    // 直接接收
    Logger::info("Waiting to receive...");
    auto recvRes = socket->receiveFrame(-1); // 阻塞
    if (recvRes.isSuccess()) {
        Logger::info("Received a frame");
    } else {
        Logger::error("Receive failed: " + recvRes.error().toString());
    }

    socket->close();
    return 0;
}
