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
#include "motor/drivers/base/can_motor.hpp"

namespace robot::motor::eyou {
    using robot::can::CANFrame;

    // 意优特定状态
    struct EYOUMotorState {
        uint32_t alarm_code = 0;
        int32_t raw_position = 0;
        int32_t raw_velocity = 0;
        int32_t raw_current = 0;
        double estimated_torque = 0.0;

        bool hasAlarm(uint32_t alarm_flag) const { return (alarm_code & alarm_flag) != 0; }

        std::vector<std::string> getAlarmDescriptions() const;
    };

    // 意优命令基类
    class EYOUCommand : public MotorCommand, public CANCommandEncoder {
    public:
        std::vector<CANFrame> encode(uint32_t motor_id) const override = 0;
    };

    // 使能/失能
    class EYOUEnableCmd : public EYOUCommand {
    public:
        explicit EYOUEnableCmd(const bool enable) : enable_(enable) {
        }

        std::vector<CANFrame> encode(uint32_t motor_id) const override;

    private:
        bool enable_;
    };

    // 轮廓位置命令
    class EYOUProfilePositionCmd : public EYOUCommand {
    public:
        EYOUProfilePositionCmd(const double pos, const double vel, const double torque, const double acc,
                               const double dec, EYOUMotorSpec spec)
            : position_(pos), velocity_(vel), torque_(torque), accel_(acc), decel_(dec), spec_(std::move(spec)) {
        }

        std::vector<CANFrame> encode(uint32_t motor_id) const override;

    private:
        double position_;
        double velocity_;
        double torque_;
        double accel_;
        double decel_;
        EYOUMotorSpec spec_;
    };

    // 速度命令
    class EYOUVelocityCmd : public EYOUCommand {
    public:
        EYOUVelocityCmd(double vel, double max_current) : velocity_(vel), max_current_(max_current) {
        }

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

        std::vector<CANFrame> encode(uint32_t motor_id) const override;

        double getTorque() const { return torque_nm_; }

    private:
        double torque_nm_;
        EYOUMotorSpec spec_;
    };

    // 紧急停止
    class EYOUEmergencyStopCmd : public EYOUCommand {
    public:
        std::vector<CANFrame> encode(uint32_t motor_id) const override;
    };

    // 清除故障
    class EYOUClearFaultCmd : public EYOUCommand {
    public:
        std::vector<CANFrame> encode(uint32_t motor_id) const override;
    };

    // 设置零点
    class EYOUSetZeroCmd : public EYOUCommand {
    public:
        std::vector<CANFrame> encode(uint32_t motor_id) const override;
    };

    // 读取寄存器命令（查询命令）
    class EYOUReadCmd : public EYOUCommand {
    public:
        explicit EYOUReadCmd(uint8_t addr) : addr_(addr) {
        }

        std::vector<CANFrame> encode(uint32_t motor_id) const override;

        uint8_t getAddress() const { return addr_; }

    private:
        uint8_t addr_;
    };
}
