/**
* @Author: ZZSZSZSZZ
 * @CreateTime: 2026-02-22
 * @Description:
 * @Version: 1.0
 */

#pragma once

#include <condition_variable>
#include <memory>
#include <mutex>
#include <utility>

#include "motor_type.hpp"
#include "can/can_frame.hpp"

using robot::can::CANFrame;

namespace robot::motor {
    class MotorCommand {
    public:
        virtual ~MotorCommand() = default;
    };

    // 电机抽象类
    class Motor {
    public:
        virtual ~Motor() = default;

        // 基本信息
        /// 电机 ID
        /// @return 电机 ID 字符串
        virtual uint32_t id() const = 0;

        /// 电机名称
        /// @return 电机名称字符串
        virtual const std::string &name() const = 0;

        /// 获取电机型号
        /// @return 电机型号字符串
        virtual const std::string &type() const = 0;

        // 电机状态
        /// 获取电机状态
        /// @return 电机状态
        virtual MotorState getState() const = 0;

        /// 设置电机状态
        /// @param state 电机状态
        // virtual void setState(const MotorState& state) = 0;

        virtual bool waitForStateUpdate(MotorState &state, uint32_t timeout_ms) = 0;

        virtual uint32_t getFaultCode() const { return 0; }
        virtual std::vector<std::string> getFaultDescriptions() const { return {}; }

        // 基础控制
        virtual bool enable() = 0;

        virtual bool disable() = 0;

        virtual bool emergencyStop() = 0;

        virtual bool clearFault() = 0;

        virtual bool setZeroPosition() = 0;

        // 运动控制
        virtual bool command(const MotorCommand& cmd) = 0;

        virtual bool setPosition(double position_rad, double max_vel, double max_torque) = 0;

        virtual bool setVelocity(double velocity_rad_s, double max_current) = 0;

        virtual bool setTorque(double torque_nm) = 0;

        virtual bool setCurrent(double current_a) = 0;

        // 配置
        virtual bool configure(const MotorConfig &config) = 0;

        virtual MotorConfig getConfig() const = 0;

        virtual bool saveConfig() { return false; }

        // 内部接口（由Manager调用）
        virtual void onCANFrameReceived(const CANFrame &frame) = 0;

        // 状态轮询回调（用于主动查询型电机）
        virtual std::vector<CANFrame> onStatusPoll() { return {}; }

        virtual void setStateCallback(std::function<void(const MotorState &)> cb) = 0;
    };

    /// 电机基类
    class BaseMotor : public Motor {
    public:
        explicit BaseMotor(MotorConfig config) : config_(std::move(config)) {
            current_state_.timestamp = std::chrono::steady_clock::now();
        }

        uint32_t id() const override { return config_.id; }
        const std::string &name() const override { return config_.name; }

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

        void setStateCallback(std::function<void(const MotorState &)> cb) override {
            std::lock_guard lock(state_mutex_);
            state_callback_ = cb;
        }

    protected:
        MotorConfig config_;
        MotorState current_state_;
        mutable std::mutex state_mutex_;
        std::function<void(const MotorState &)> state_callback_;
        std::condition_variable state_cv_;

        void updateState(const MotorState &new_state) {
            std::lock_guard lock(state_mutex_);
            current_state_ = new_state;
            current_state_.timestamp = std::chrono::steady_clock::now();

            if (state_callback_) {
                state_callback_(current_state_);
            }
            state_cv_.notify_all();
        }
    };
}
