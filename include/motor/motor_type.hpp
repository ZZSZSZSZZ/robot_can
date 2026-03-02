/**
 * @Author: ZZSZSZSZZ
 * @CreateTime: 2026-02-22
 * @Description: 
 * @Version: 1.0
 */

#pragma once

#include <chrono>
#include <string>

#include "can/can_frame.hpp"

using robot::can::CANFrameFormat;

namespace robot::motor {
    enum class MotorCapability : uint64_t {
        None = 0,

        // 基础控制
        Enable = 1ULL << 0,
        Disable = 1ULL << 1,
        EmergencyStop = 1ULL << 2,
        ClearFault = 1ULL << 3,
        SetZero = 1ULL << 4,

        // 控制模式
        PositionControl = 1ULL << 10,
        VelocityControl = 1ULL << 11,
        TorqueControl = 1ULL << 12,
        CurrentControl = 1ULL << 13,
        ProfilePosition = 1ULL << 14,
        ProfileVelocity = 1ULL << 15,
        MITMode = 1ULL << 16,
        ImpedanceControl = 1ULL << 17,

        // 反馈数据
        FeedbackPosition = 1ULL << 20,
        FeedbackVelocity = 1ULL << 21,
        FeedbackTorque = 1ULL << 22,
        FeedbackCurrent = 1ULL << 23,
        FeedbackTemperature = 1ULL << 24,
        FeedbackVoltage = 1ULL << 25,

        // 状态获取方式
        AutoStatusFeedback = 1ULL << 30, // 电机自动发送状态
        PollStatus = 1ULL << 31, // 需要主动查询

        // 配置功能
        ConfigurablePID = 1ULL << 40,
        ConfigurableLimit = 1ULL << 41,
        ConfigurableCanId = 1ULL << 42,
        ConfigurableBaudrate = 1ULL << 43,
        SaveToFlash = 1ULL << 44,

        All = 0xFFFFFFFFFFFFFFFF
    };

    inline MotorCapability operator|(MotorCapability a, MotorCapability b) {
        return static_cast<MotorCapability>(static_cast<uint64_t>(a) | static_cast<uint64_t>(b));
    }

    inline MotorCapability operator&(MotorCapability a, MotorCapability b) {
        return static_cast<MotorCapability>(static_cast<uint64_t>(a) & static_cast<uint64_t>(b));
    }

    inline bool hasCapability(MotorCapability caps, MotorCapability check) {
        return (static_cast<uint64_t>(caps) & static_cast<uint64_t>(check)) != 0;
    }

    // 控制模式
    enum class ControlMode {
        Idle,
        Position, // 位置模式
        Velocity, // 速度模式
        Torque, // 力矩模式
        ProfilePosition, // 轮廓位置模式
        MIT, // MIT模式
        Impedance // 阻抗控制
    };

    struct MotorState {
        std::chrono::steady_clock::time_point timestamp;

        bool enabled = false;
        bool fault = false;
        ControlMode current_mode = ControlMode::Idle;

        double position = 0.0; // 当前实际位置
        double velocity = 0.0; // 当前实际速度
        double torque = 0.0; // 当前实际力矩
        double current = 0.0; // 当前实际电流
        double voltage = 0.0;
        double temperature = 0.0; // 温度

        std::vector<uint8_t> raw_data;

        bool isValid() const {
            return timestamp.time_since_epoch().count() > 0;
        }
    };

    struct MotorConfig {
        uint32_t id; // 电机逻辑ID
        std::string name; // 电机名称
        std::string type; // 电机类型

        uint32_t tx_can_id; // 发送CAN ID
        uint32_t rx_can_id; // 接收CAN ID

        CANFrameFormat tx_format; // 发送帧格式
        CANFrameFormat rx_format; // 接收帧格式 (期望)

        double position_min = -3.14159;
        double position_max = 3.14159;
        double velocity_max = 30.0;
        double torque_max = 10.0;

        double default_velocity = 5.0;
        double default_acceleration = 10.0;
        double default_torque_limit = 5.0;

        bool enable_auto_status = true; // 状态获取配置
        uint32_t status_poll_interval_ms = 10;

        std::function<void(const MotorState &)> on_state_update;

        void useStandardFrame(uint8_t dlc = 8) {
            tx_format = CANFrameFormat::standard(dlc);
            rx_format = CANFrameFormat::standard(dlc);
        }

        void useExtendedFrame(uint8_t dlc = 8) {
            tx_format = CANFrameFormat::extended(dlc);
            rx_format = CANFrameFormat::extended(dlc);
        }
    };

    struct MotorInfo {
        uint32_t id;
        std::string name;
        std::string driver_type;
        MotorCapability capabilities;
        bool enabled;
        std::chrono::steady_clock::time_point last_update;
    };
}
