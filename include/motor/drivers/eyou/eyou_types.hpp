/**
 * @Author: ZZSZSZSZZ
 * @CreateTime: 2026-02-26
 * @Description: 意优电机命令类
 * @Version: 1.0
 */

#pragma once

#include <memory>
#include <utility>
#include <vector>
#include <string>

#include "eyou_specification.hpp"
#include "eyou_units.hpp"
#include "motor/motor_interface.hpp"
#include "can/can_frame.hpp"

namespace robot::motor::eyou {
    // 意优特定状态
    struct EYOUMotorState {
        uint32_t alarm_code = 0;
        double bus_voltage = 0.0;
        int32_t raw_position = 0;
        int32_t raw_velocity = 0;
        int32_t raw_current = 0;
        double estimated_torque = 0.0;

        bool hasAlarm(uint32_t alarm_flag) const { return (alarm_code & alarm_flag) != 0; }

        std::vector<std::string> getAlarmDescriptions() const;
    };

    // 意优命令基类
    class EYOUCommand : public MotorCommand {
    public:
        virtual std::vector<CANFrame> encode(uint32_t motor_id) const = 0;
    };

    // 使能/失能
    class EYOUEnableCmd : public EYOUCommand {
    public:
        explicit EYOUEnableCmd(const bool enable) : enable_(enable) {
        }

        ControlMode getMode() const override { return ControlMode::Idle; }

        std::vector<CANFrame> encode(uint32_t motor_id) const override;

    private:
        bool enable_;
    };

    // 轮廓位置命令
    class EYOUProfilePositionCmd : public EYOUCommand {
    public:
        EYOUProfilePositionCmd(double pos, double vel, double torque, double acc, double dec, EYOUMotorSpec spec)
            : position_(pos), velocity_(vel), torque_(torque), accel_(acc), decel_(dec), spec_(std::move(spec)) {
        }

        ControlMode getMode() const override { return ControlMode::ProfilePosition; }

        std::vector<CANFrame> encode(uint32_t motor_id) const override;

    private:
        double position_;
        double velocity_;
        double accel_;
        double decel_;
        double torque_;
        EYOUMotorSpec spec_;
    };

    // 速度命令
    class EYOUVelocityCmd : public EYOUCommand {
    public:
        EYOUVelocityCmd(double vel, double max_current) : velocity_(vel), max_current_(max_current) {
        }

        ControlMode getMode() const override { return ControlMode::Velocity; }

        std::vector<CANFrame> encode(uint32_t motor_id) const override;

    private:
        double velocity_;
        double max_current_;
    };

    // 扭矩命令
    class EYOUTorqueCmd : public EYOUCommand {
    public:
        EYOUTorqueCmd(double torque_nm, EYOUMotorSpec spec) : torque_nm_(torque_nm), spec_(std::move(spec)) {
        }

        ControlMode getMode() const override { return ControlMode::Torque; }

        std::vector<CANFrame> encode(uint32_t motor_id) const override;

        double getTorque() const { return torque_nm_; }

    private:
        double torque_nm_;
        EYOUMotorSpec spec_;
    };

    // 紧急停止
    class EYOUEmergencyStopCmd : public EYOUCommand {
    public:
        ControlMode getMode() const override { return ControlMode::Idle; }

        std::vector<CANFrame> encode(uint32_t motor_id) const override;
    };

    // 清除故障
    class EYOUClearFaultCmd : public EYOUCommand {
    public:
        ControlMode getMode() const override { return ControlMode::Idle; }

        std::vector<CANFrame> encode(uint32_t motor_id) const override;
    };

    // 设置零点
    class EYOUSetZeroCmd : public EYOUCommand {
    public:
        ControlMode getMode() const override { return ControlMode::Idle; }

        std::vector<CANFrame> encode(uint32_t motor_id) const override;
    };
}
