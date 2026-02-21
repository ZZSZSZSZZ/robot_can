/**
 * @Author: ZZSZSZSZZ
 * @CreateTime: 2026-02-17
 * @Description: CAN 套接字抽象基类
 * @Version: 1.0
 */

#pragma once

#include "can_frame.hpp"
#include "common/common.hpp"

namespace robot::can {
    // CAN 套接字抽象类
    // 声明 CAN 套接字的通用接口
    class CANSocket {
    public:
        virtual ~CANSocket() = default;

        /// 打开指定名称的 CAN 网络接口
        /// @param interface CAN 网络接口 如 "can0"
        /// @param enableCanFd 启用 CAN FD
        /// @return Result<void> 表示成功或错误
        virtual Result<void> open(const std::string &interface, bool enableCanFd = false) = 0;

        ///关闭当前打开的 CAN 套接字
        virtual void close() = 0;

        /// 检查套接字是否已打开
        virtual bool isOpen() const = 0;

        /// 检查是否支持 CAN FD 帧
        virtual bool supportsCanFd() const = 0;

        /// 发送前检查是否支持拓展 CAN 帧
        virtual bool supportsExtendedId() const = 0;

        /// 发送一个 CAN 帧
        /// @param frame CAN 帧
        /// @return Result<void> 表示成功或错误
        virtual Result<void> writeFrame(const CANFrame &frame) = 0;

        /// 接收一个 CAN 帧
        /// @param timeout_ms 超时时间 (毫秒) -1 表示阻塞等待，0 表示立即返回
        /// @return 成功时返回 Result<CANPacket>, 包含接收到的帧
        virtual Result<CANFrame> receiveFrame(int timeout_ms = -1) = 0;

        /// 检查在指定微秒超时内是否有数据可读
        /// @param timeout_us 超时时间 (微秒)
        /// @return 返回 Result<bool>, true 表示有数据
        virtual Result<bool> isDataAvailable(int timeout_us) = 0;

        /// 返回当前打开的接口名称
        virtual std::string getInterfaceName() const = 0;

        virtual int getSocket() const = 0;
    };
}
