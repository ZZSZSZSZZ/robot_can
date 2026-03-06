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
    // 控制模式
    enum class ControlMode {
        Position, // 位置模式
        Velocity, // 速度模式
        Torque, // 力矩模式
        ProfilePosition, // 轮廓位置模式
        MIT, // MIT模式
        Impedance // 阻抗控制
    };

    // 电机状态机
    enum class MotorStateMachine : uint8_t {
        Disabled = 0, // 未使能，不查询
        Enabling, // 正在使能，等待确认
        Enabled, // 已使能，正常查询
        Disabling, // 正在失能，等待确认
        Fault // 故障，低频查询
    };

    // 电机状态
    struct MotorState {
        std::chrono::steady_clock::time_point timestamp;
        MotorStateMachine state_machine = MotorStateMachine::Disabled;

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

    // 电机配置
    struct MotorConfig {
        uint32_t id; // 电机ID
        std::string name; // 电机名称
        std::string type; // 电机类型

        uint32_t tx_can_id = 0; // 发送CAN ID (默认电机ID)
        uint32_t rx_can_id = 0; // 接收CAN ID (默认电机ID)

        CANFrameFormat tx_format; // 发送帧格式
        CANFrameFormat rx_format; // 接收帧格式 (期望)

        double position_min = -3.14159;
        double position_max = 3.14159;
        double velocity_max = 30.0;
        double torque_max = 10.0;

        double default_velocity = 10.0;
        double default_acceleration = 20.0;
        double default_torque = 5.0;

        bool enable_auto_status = true; // 状态获取配置

        std::function<void(const MotorState &)> on_state_update;

        void useStandardFrame(const uint8_t data_len = 8) {
            tx_format = CANFrameFormat::standard(data_len);
            rx_format = CANFrameFormat::standard(data_len);
        }

        void useExtendedFrame(const uint8_t data_len = 8) {
            tx_format = CANFrameFormat::extended(data_len);
            rx_format = CANFrameFormat::extended(data_len);
        }

        void useStandardCanFDFrame(const uint8_t data_len = 64) {
            tx_format = CANFrameFormat::canFd(false, data_len);
            rx_format = CANFrameFormat::canFd(false, data_len);
        }

        void useExtendedCanFDFrame(const uint8_t data_len = 64) {
            tx_format = CANFrameFormat::canFd(true, data_len);
            rx_format = CANFrameFormat::canFd(true, data_len);
        }
    };

    // 电机信息
    struct MotorInfo {
        uint32_t id;
        std::string name;
        std::string driver_type;
        bool enabled;
        std::chrono::steady_clock::time_point last_update;
    };
}
