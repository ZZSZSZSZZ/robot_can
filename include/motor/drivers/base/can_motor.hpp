/**
 * @Author: ZZSZSZSZZ
 * @CreateTime: 2026-03-06
 * @Description: CAN电机基础抽象类 - 提供CAN通信通用功能
 * @Version: 2.0
 */

#pragma once

#include <atomic>
#include <vector>

#include "base_motor.hpp"
#include "can/can_frame.hpp"
#include "can/can_device.hpp"
#include "can/can_coding.hpp"

namespace robot::motor {
    // CAN帧编码器接口
    // 定义如何将电机命令编码为CAN帧
    class CANCommandEncoder {
    public:
        virtual ~CANCommandEncoder() = default;

        /// 将命令编码为CAN帧
        /// @param motor_id 电机ID
        /// @return 编码后的CAN帧列表
        virtual std::vector<can::CANFrame> encode(uint32_t motor_id) const = 0;
    };

    // CAN帧解码器接口
    // 定义如何从CAN帧解码电机状态
    class CANStateDecoder {
    public:
        virtual ~CANStateDecoder() = default;

        /// 解码CAN帧到电机状态
        /// @param frame 接收到的CAN帧
        /// @param out_state 输出的通用状态
        /// @return 是否成功解码
        virtual bool decode(const can::CANFrame &frame, MotorState &out_state) = 0;

        /// 检查是否可以处理该帧
        /// @param frame 接收到的CAN帧
        /// @return 是否可以处理
        virtual bool canDecode(const can::CANFrame &frame) const = 0;
    };

    // CAN电机抽象基类
    // 继承BaseMotor, 添加CAN通信相关的通用功能
    class CANMotor : public BaseMotor {
    public:
        explicit CANMotor(MotorConfig config) : BaseMotor(std::move(config)) {
        }

        ~CANMotor() override = default;

        // ========== CAN设备接口 ==========

        /// 接收CAN帧回调 (由MotorManager调用)
        /// @param frame 接收到的CAN帧
        void onCANFrameReceived(const can::CANFrame &frame) override = 0;

        /// 获取发送帧格式要求
        /// @return 帧格式要求
        virtual can::CANDeviceFrameRequirement getFrameRequirement() const {
            can::CANDeviceFrameRequirement req{};
            req.preferredType = config_.tx_format.type;
            req.requireExtendedId = config_.tx_format.isExtendedId;
            req.maxDataLength = config_.tx_format.dlc;
            req.requireCanFd = config_.tx_format.type == can::CANFrameType::CanFd;
            return req;
        }

        /// 获取接收CAN ID
        /// @return 接收CAN ID
        virtual uint32_t getReceiveCanId() const {
            return config_.rx_can_id != 0 ? config_.rx_can_id : config_.id;
        }

        /// 获取发送CAN ID
        /// @return 发送CAN ID
        virtual uint32_t getTransmitCanId() const {
            return config_.tx_can_id != 0 ? config_.tx_can_id : config_.id;
        }

        /// 是否使用扩展ID
        /// @return 是否使用扩展ID
        virtual bool isExtendedId() const {
            return config_.rx_format.isExtendedId;
        }

        /// 状态轮询回调 (返回需要发送的查询帧)
        /// @return 轮询帧列表
        std::vector<can::CANFrame> onStatusPoll() override { return {}; }

    protected:
        mutable std::mutex pending_mutex_; // 待发送帧队列锁
        std::vector<can::CANFrame> pending_frames_; // 待发送帧队列
        std::atomic<bool> has_pending_frames_{false}; // 是否有待发送帧

        /// 编码通用CAN帧
        /// @param data 帧数据内容
        /// @param can_id CAN ID
        /// @return 编码后的CAN帧
        can::CANFrame encodeFrame(const std::vector<uint8_t> &data, uint32_t can_id) const {
            return can::CANFrameEncoder::encode(data, can_id, getFrameRequirement());
        }

        /// 编码命令并加入发送队列
        /// @param encoder 命令编码器
        void enqueueCommand(const CANCommandEncoder &encoder) {
            auto frames = encoder.encode(getTransmitCanId());
            enqueueFrames(std::move(frames));
        }

        /// 将帧加入发送队列
        /// @param frames 要发送的帧列表
        void enqueueFrames(std::vector<can::CANFrame> frames) {
            if (frames.empty()) return;
            std::lock_guard lock(pending_mutex_);
            pending_frames_.insert(pending_frames_.end(),
                                   std::make_move_iterator(frames.begin()),
                                   std::make_move_iterator(frames.end()));
            has_pending_frames_.store(true, std::memory_order_release);
        }

        /// 将单帧加入发送队列
        /// @param frame 要发送的帧
        void enqueueFrame(can::CANFrame frame) {
            std::lock_guard lock(pending_mutex_);
            pending_frames_.push_back(std::move(frame));
            has_pending_frames_.store(true, std::memory_order_release);
        }

        /// 提取待发送的帧
        /// @return 待发送帧列表
        std::vector<can::CANFrame> extractPendingFrames() {
            std::lock_guard lock(pending_mutex_);
            if (!has_pending_frames_.load(std::memory_order_acquire)) {
                return {};
            }
            std::vector<can::CANFrame> result = std::move(pending_frames_);
            pending_frames_.clear();
            has_pending_frames_.store(false, std::memory_order_release);
            return result;
        }

        /// 检查是否有待发送帧
        /// @return 是否有待发送帧
        bool hasPendingFrames() const {
            return has_pending_frames_.load(std::memory_order_acquire);
        }
    };
} // namespace robot::motor
