/**
 * @Author: ZZSZSZSZZ
 * @CreateTime: 2026-03-06
 * @Description: 电机基础抽象基类 - 提供通用状态管理和配置管理
 * @Version: 2.0
 */

#pragma once

#include <condition_variable>
#include <memory>
#include <mutex>

#include "motor/motor_interface.hpp"

namespace robot::motor {
    /**
     * @brief 电机基础抽象类
     * 
     * 提供通用的状态管理、配置管理和线程安全的状态更新机制。
     * 所有具体电机驱动应继承此类，并专注于实现硬件特定的功能。
     */
    class BaseMotor : public Motor {
    public:
        explicit BaseMotor(MotorConfig config) : config_(std::move(config)) {
            current_state_.timestamp = std::chrono::steady_clock::now();
        }

        ~BaseMotor() override = default;

        // ========== 基本信息 (通用实现) ==========
        uint32_t id() const override { return config_.id; }
        const std::string &name() const override { return config_.name; }

        // ========== 状态管理 (通用实现) ==========
        MotorState getState() const override {
            std::lock_guard lock(state_mutex_);
            return current_state_;
        }

        bool waitForStateUpdate(MotorState &state, uint32_t timeout_ms) override {
            std::unique_lock lock(state_mutex_);
            auto last = current_state_.timestamp;
            bool ok = state_cv_.wait_for(lock, std::chrono::milliseconds(timeout_ms), [&] {
                return current_state_.timestamp != last;
            });
            if (ok) state = current_state_;
            return ok;
        }

        // ========== 配置管理 (通用实现) ==========
        bool configure(const MotorConfig &config) override {
            std::lock_guard lock(state_mutex_);
            config_ = config;
            return true;
        }

        MotorConfig getConfig() const override {
            std::lock_guard lock(state_mutex_);
            return config_;
        }

        // ========== 回调设置 (通用实现) ==========
        void setStateCallback(std::function<void(const MotorState &)> cb) override {
            std::lock_guard lock(state_mutex_);
            state_callback_ = std::move(cb);
        }

    protected:
        mutable std::mutex state_mutex_; // 状态保护互斥锁
        MotorConfig config_; // 电机配置
        MotorState current_state_; // 当前状态
        std::function<void(const MotorState &)> state_callback_; // 状态回调
        std::condition_variable state_cv_; // 状态更新条件变量

        // /**
        //  * @brief 更新电机状态 (线程安全)
        //  * @param new_state 新状态
        //  */


        void updateState(const MotorState &new_state) {
            std::lock_guard lock(state_mutex_);
            current_state_ = new_state;
            current_state_.timestamp = std::chrono::steady_clock::now();

            if (state_callback_) {
                state_callback_(current_state_);
            }
            state_cv_.notify_all();
        }

        /// 更新状态的部分字段
        /// @param updater 更新函数
        template<typename Updater>
        void updateStatePartial(Updater &&updater) {
            std::lock_guard lock(state_mutex_);
            updater(current_state_);
            current_state_.timestamp = std::chrono::steady_clock::now();

            if (state_callback_) {
                state_callback_(current_state_);
            }
            state_cv_.notify_all();
        }

        /**
         * @brief 获取当前状态的副本（线程安全）
         * @return 当前状态的副本
         */
        MotorState getCurrentStateSnapshot() const {
            std::lock_guard lock(state_mutex_);
            return current_state_;
        }

        // ========== 静态类型转换工具模板 ==========

        /**
         * @brief 从 Motor 智能指针转换到派生类型
         * @tparam Derived 派生电机类型
         * @param motor 电机智能指针
         * @return 转换后的智能指针，失败返回 nullptr
         */
        template<typename Derived>
        static std::shared_ptr<Derived> ptrFrom(std::shared_ptr<Motor> motor) {
            return std::dynamic_pointer_cast<Derived>(motor);
        }

        /**
         * @brief 从 Motor 智能指针强制转换到派生类型
         * @tparam Derived 派生电机类型
         * @param motor 电机智能指针
         * @return 转换后的智能指针
         * @throws std::bad_cast 转换失败时抛出
         */
        template<typename Derived>
        static std::shared_ptr<Derived> ptrCast(std::shared_ptr<Motor> motor) {
            auto result = ptrFrom<Derived>(motor);
            if (!result) {
                throw std::bad_cast();
            }
            return result;
        }

        /**
         * @brief 检查 Motor 智能指针是否为指定派生类型
         * @tparam Derived 派生电机类型
         * @param motor 电机智能指针
         * @return 是否匹配
         */
        template<typename Derived>
        static bool ptrIs(std::shared_ptr<Motor> motor) {
            return ptrFrom<Derived>(motor) != nullptr;
        }
    };
} // namespace robot::motor
