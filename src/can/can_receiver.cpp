/**
 * @Author: ZZSZSZSZZ
 * @CreateTime: 2026-02-18
 * @Description: CAN 帧接收线程
 * @Version: 1.0
 */

#include "can/can_receiver.hpp"

namespace robot::can {
    Result<void> CANReceiver::start() {
        // 检查是否运行
        if (running_) return ErrorCode::success();

        try {
            // 设置运行状态标志位为 true
            running_ = true;
            // 创建接收线程
            thread_ = std::thread(&CANReceiver::receiveLoop, this);
            Logger::info("CAN receiver thread started");
            return ErrorCode::success();
        } catch (...) {
            running_ = false;
            return ErrorCode::system(common::SystemError::ThreadCreationFailed, 0);
        }
    }

    void CANReceiver::stop() {
        // 如果已经停止则直接返回
        if (bool expected = true; !running_.compare_exchange_strong(expected, false)) {
            return;
        }
        // 检测线程是否有效, 如果有效, 等待线程停止
        if (thread_.joinable()) {
            thread_.join();
        }
        Logger::info("CAN receiver thread stopped");
    }

    void CANReceiver::receiveLoop() const {
        while (running_) {
            // 接收一帧
            auto result = socket_->receiveFrame(100);
            if (result.isSuccess()) {
                // 将接收到的 CAN 帧路由到对应的设备
                auto routeResult = router_->route(result.value());
                if (routeResult.isError()) {
                    Logger::warn("Route failed: " + routeResult.error().toString());
                }
            } else {
                if (result.error() != ErrorCode::transport(common::TransportError::Timeout, 0)) {
                    Logger::error("receiveFrame failed: " + result.error().toString());
                }
            }
        }
    }
}
