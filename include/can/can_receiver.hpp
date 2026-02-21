/**
 * @Author: ZZSZSZSZZ
 * @CreateTime: 2026-02-18
 * @Description: CAN 帧接收线程
 * @Version: 1.0
 */

#pragma once

#include <atomic>
#include <thread>

#include "can/can_socket.hpp"
#include "can/can_frame_router.hpp"

namespace robot::can {
    // CAN 帧接收线程
    // 负责异步接收 CAN 帧, 运行一个独立的后台线程, 持续从 CANSocket 读取帧, 并分发给已注册的设备
    class CANReceiver {
    public:
        /// CANReceiver 构造函数
        /// @param socket CANSocket 对象
        /// @param router CANFrameRouter 对象
        CANReceiver(const std::shared_ptr<CANSocket> &socket, const std::shared_ptr<CANFrameRouter> &router)
            : socket_(socket), router_(router), running_(false) {
        }

        ~CANReceiver() { stop(); }

        /// 启动接收线程
        Result<void> start();

        /// 停止接收线程
        void stop();

        /// 接收线程是否运行
        bool isRunning() const { return running_; }

        std::thread& getThread() { return thread_; }

    private:
        /// 线程主循环
        void receiveLoop() const;

        // CANSocket 对象
        std::shared_ptr<CANSocket> socket_;
        // CANDeviceDispatcher 对象
        std::shared_ptr<CANFrameRouter> router_;
        // 接收线程的运行状态 (true 运行, false 停止)
        std::atomic<bool> running_;
        // 后台接收线程
        std::thread thread_;
    };
}
