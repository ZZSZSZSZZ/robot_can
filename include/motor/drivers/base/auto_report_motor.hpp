/**
 * @Author: ZZSZSZSZZ
 * @CreateTime: 2026-03-12
 * @Description: 自动上报型电机抽象基类 - 适用于自动上报状态的电机
 * @Version: 1.0
 */

#pragma once

#include "can_motor.hpp"

namespace robot::motor {
    /**
     * @brief 自动上报型电机抽象基类
     *
     * 适用于电机主动周期性上报状态的通信模式。
     * 与PollingMotor不同，此类不需要主动轮询获取状态，
     * 电机在收到控制命令后会自动回复状态帧，或周期性主动上报。
     *
     * 子类需要实现:
     * - onCANFrameReceived(): 处理接收到的状态帧
     * - generateKeepAliveFrame(): 生成保活/心跳帧(如需要)
     */
    class AutoReportMotor : public CANMotor {
    public:
        explicit AutoReportMotor(MotorConfig config)
            : CANMotor(std::move(config)),
              auto_report_enabled_(false),
              report_timeout_ms_(100) {
        }

        ~AutoReportMotor() override = default;

        // ========== 自动上报控制 ==========

        /// 设置自动上报使能状态
        /// @param enabled 是否使能自动上报
        void setAutoReportEnabled(bool enabled) {
            auto_report_enabled_ = enabled;
        }

        /// 检查自动上报是否使能
        /// @return 是否使能
        bool isAutoReportEnabled() const {
            return auto_report_enabled_;
        }

        /// 设置上报超时时间
        /// @param timeout_ms 超时时间(毫秒)
        void setReportTimeout(uint32_t timeout_ms) {
            report_timeout_ms_ = timeout_ms;
        }

        /// 获取上报超时时间
        /// @return 超时时间(毫秒)
        uint32_t getReportTimeout() const {
            return report_timeout_ms_;
        }

        /// 更新最后收到上报的时间
        void updateLastReportTime() {
            last_report_time_ = std::chrono::steady_clock::now();
        }

        /// 检查是否上报超时
        /// @return 是否超时
        bool isReportTimeout() const {
            auto now = std::chrono::steady_clock::now();
            auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
                now - last_report_time_).count();
            return elapsed > static_cast<int64_t>(report_timeout_ms_);
        }

        // ========== CANMotor 接口实现 ==========

        /**
         * @brief 状态轮询回调 - 对于自动上报电机，通常不需要生成查询帧
         * @return 轮询帧列表(默认返回控制命令队列中的帧)
         */
        std::vector<can::CANFrame> onStatusPoll() override {
            // 自动上报电机不需要主动轮询状态
            // 只返回待发送的控制命令帧
            return extractPendingFrames();
        }

        /**
         * @brief 只查询状态回调 - 对于自动上报电机，返回空
         * @return 空列表
         */
        std::vector<can::CANFrame> onStateOnlyPoll() override {
            // 自动上报电机不需要查询状态
            return {};
        }

    protected:
        std::atomic<bool> auto_report_enabled_{false};
        std::atomic<uint32_t> report_timeout_ms_{100};
        std::chrono::steady_clock::time_point last_report_time_;

        /**
         * @brief 生成保活帧(可选) - 子类可重写
         *
         * 某些自动上报电机需要定期发送保活帧来维持连接或触发状态上报。
         * 默认返回空列表。
         *
         * @return 保活帧列表
         */
        virtual std::vector<can::CANFrame> generateKeepAliveFrame() {
            return {};
        }

        /**
         * @brief 检查是否需要发送保活帧
         * @return 是否需要发送
         */
        bool shouldSendKeepAlive() const {
            return isReportTimeout();
        }
    };

} // namespace robot::motor
