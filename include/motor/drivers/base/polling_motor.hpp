/**
 * @Author: ZZSZSZSZZ
 * @CreateTime: 2026-03-06
 * @Description: 轮询型电机抽象基类 - 提供状态轮询管理
 * @Version: 2.0
 */

#pragma once

#include <atomic>
#include <cstdint>

#include "can_motor.hpp"

namespace robot::motor {
    // 轮询策略配置
    struct PollingPolicy {
        uint32_t full_poll_interval; // 全量轮询间隔(轮询计数)
        uint32_t position_poll_divisor; // 位置轮询分频(每N次轮询)
        uint32_t velocity_poll_divisor; // 速度轮询分频
        uint32_t current_poll_divisor; // 电流轮询分频
        uint32_t temperature_poll_divisor; // 温度轮询分频

        static PollingPolicy balanced() {
            return {10, 1, 2, 2, 10};
        }

        static PollingPolicy high_frequency() {
            return {5, 1, 1, 1, 5};
        }

        static PollingPolicy low_frequency() {
            return {20, 1, 5, 5, 20};
        }
    };

    // 轮询型电机抽象基类
    // 继承CANMotor, 添加轮询管理功能
    class PollingMotor : public CANMotor {
    public:
        explicit PollingMotor(MotorConfig config,
                              PollingPolicy policy = PollingPolicy::balanced())
            : CANMotor(std::move(config)), polling_policy_(policy) {
        }

        ~PollingMotor() override = default;

        // ========== 轮询策略设置 ==========

        /// 设置轮询策略
        /// @param policy 轮询策略
        void setPollingPolicy(const PollingPolicy &policy) {
            polling_policy_ = policy;
        }

        /// 获取当前轮询策略
        /// @return 轮询策略
        PollingPolicy getPollingPolicy() const {
            return polling_policy_;
        }

        /// 设置轮询分频器
        /// @param position_div 位置分频
        /// @param velocity_div 速度分频
        /// @param current_div 电流分频
        void setPollDivisors(uint32_t position_div, uint32_t velocity_div, uint32_t current_div) {
            polling_policy_.position_poll_divisor = position_div;
            polling_policy_.velocity_poll_divisor = velocity_div;
            polling_policy_.current_poll_divisor = current_div;
        }

        // ========== CANMotor 接口实现 ==========

        std::vector<can::CANFrame> onStatusPoll() override {
            // 1. 提取待发送的帧（控制命令优先）
            auto frames = extractPendingFrames();
            frames.reserve(frames.size() + 8);

            // 2. 生成状态查询帧
            generatePollFrames(frames);

            return frames;
        }

        std::vector<can::CANFrame> onStateOnlyPoll() override {
            // 1. 提取待发送的帧（控制命令优先）
            auto frames = extractPendingFrames();
            frames.reserve(frames.size() + 8);

            // 2. 生成状态查询帧
            generateStateOnlyPollFrames(frames);

            return frames;
        }

    protected:
        std::atomic<uint32_t> poll_counter_{0};
        PollingPolicy polling_policy_;

        /**
         * @brief 生成轮询帧 (子类必须实现)
         * @param out_frames 输出帧列表
         */
        virtual void generatePollFrames(std::vector<can::CANFrame> &out_frames) = 0;

        virtual void generateStateOnlyPollFrames(std::vector<can::CANFrame> &out_frames) = 0;

        /**
         * @brief 获取当前轮询计数
         * @return 轮询计数
         */
        uint32_t getPollCount() const {
            return poll_counter_.load(std::memory_order_relaxed);
        }

        /**
         * @brief 增加轮询计数
         * @return 增加前的计数值
         */
        uint32_t incrementPollCount() {
            return poll_counter_.fetch_add(1, std::memory_order_relaxed);
        }

        /**
         * @brief 检查是否应轮询（基于分频）
         * @param divisor 分频系数
         * @return 是否应该轮询
         */
        bool shouldPoll(uint32_t divisor) const {
            if (divisor == 0) return false;
            return getPollCount() % divisor == 0;
        }

        /**
         * @brief 检查是否应进行全量轮询
         * @return 是否应该全量轮询
         */
        bool shouldFullPoll() const {
            return shouldPoll(polling_policy_.full_poll_interval);
        }

        /**
         * @brief 检查是否应轮询位置
         * @return 是否应该轮询位置
         */
        bool shouldPollPosition() const {
            return shouldPoll(polling_policy_.position_poll_divisor);
        }

        /**
         * @brief 检查是否应轮询速度
         * @return 是否应该轮询速度
         */
        bool shouldPollVelocity() const {
            return shouldPoll(polling_policy_.velocity_poll_divisor);
        }

        /**
         * @brief 检查是否应轮询电流
         * @return 是否应该轮询电流
         */
        bool shouldPollCurrent() const {
            return shouldPoll(polling_policy_.current_poll_divisor);
        }

        /**
         * @brief 检查是否应轮询温度
         * @return 是否应该轮询温度
         */
        bool shouldPollTemperature() const {
            return shouldPoll(polling_policy_.temperature_poll_divisor);
        }
    };
} // namespace robot::motor
