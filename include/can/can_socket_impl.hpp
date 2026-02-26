/**
 * @Author: ZZSZSZSZZ
 * @CreateTime: 2026-02-17
 * @Description: Linux CAN 套接字具体实现
 * @Version: 1.0
 */

#pragma once

#include "can/can_socket.hpp"

namespace robot::can {
    // Linux CAN 套接字具体实现
    // 基于 Linux CAN 套接字实现所有虚函数，负责底层硬件通信
    class LinuxCANSocket : public CANSocket {
    public:
        LinuxCANSocket() = default;

        ~LinuxCANSocket() override { LinuxCANSocket::close(); }

        /// 打开指定名称的 CAN 网络接口
        /// @param interface CAN 网络接口 如 "can0"
        /// @param enable_can_fd
        /// @return Result<void> 表示成功或错误
        Result<void> open(const std::string &interface, bool enable_can_fd) override;

        ///关闭当前打开的 CAN 套接字
        void close() override;

        /// 检查套接字是否已打开
        bool isOpen() const override { return socket_fd_ >= 0; }

        bool supportsCanFd() const override { return can_fd_enabled_; }

        bool supportsExtendedId() const override { return true; }  // SocketCAN总是支持

        /// 发送一个 CAN 帧
        /// @param frame CAN 帧
        /// @return Result<void> 表示成功或错误
        Result<void> writeFrame(const CANFrame& frame) override;

        /// 读取一个 CAN 帧
        /// @param timeout_ms 超时时间 (毫秒) -1 表示阻塞等待，0 表示立即返回
        /// @return 成功时返回 Result<CANPacket>, 包含接收到的帧
        Result<CANFrame> receiveFrame(int timeout_ms) override;

        /// 检查在指定微秒超时内是否有数据可读
        /// @param timeout_us 超时时间 (微秒)
        /// @return 返回 Result<bool>, true 表示有数据
        Result<bool> isDataAvailable(int timeout_us) override;

        /// 返回当前打开的接口名称
        std::string getInterfaceName() const override { return interface_; }

        int getSocket() const override { return socket_fd_; }

    private:
        // CAN 设备文件描述符
        int socket_fd_ = -1;
        // CAN 接口名
        std::string interface_;

        bool can_fd_enabled_ = false;
        bool is_open_ = false;
    };
}
