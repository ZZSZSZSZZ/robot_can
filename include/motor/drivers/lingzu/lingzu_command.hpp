/**
 * @Author: ZZSZSZSZZ
 * @CreateTime: 2026-03-12
 * @Description: 灵足电机命令类
 * @Version: 1.0
 */

#pragma once

#include <memory>
#include <utility>
#include <vector>

#include "lingzu_specification.hpp"
#include "lingzu_units.hpp"
#include "motor/motor_interface.hpp"
#include "motor/drivers/base/can_motor.hpp"

namespace robot::motor::lingzu {
    using robot::can::CANFrame;

    // 灵足命令基类
    class LingzuCommand : public MotorCommand, public CANCommandEncoder {
    public:
        std::vector<CANFrame> encode(uint32_t motor_id) const override = 0;
    };

    // 运控模式命令 - 灵足电机的主要控制方式
    // 包含位置、速度、Kp、Kd参数，CAN ID中包含力矩信息(bit23-8)
    class LingzuMITCmd : public LingzuCommand {
    public:
        LingzuMITCmd(double pos, double vel, double torque, double kp, double kd)
            : position_(pos), velocity_(vel), torque_(torque), kp_(kp), kd_(kd) {
        }

        std::vector<CANFrame> encode(uint32_t motor_id) const override;

        double getPosition() const { return position_; }
        double getVelocity() const { return velocity_; }
        double getTorque() const { return torque_; }
        double getKp() const { return kp_; }
        double getKd() const { return kd_; }

    private:
        double position_;   // 目标位置(弧度)
        double velocity_;   // 目标角速度(rad/s)
        double torque_;     // 力矩限制(-5.5~5.5 Nm)，编码到CAN ID的bit23-8
        double kp_;         // 位置增益
        double kd_;         // 速度增益
    };

    // 使能/失能命令
    class LingzuEnableCmd : public LingzuCommand {
    public:
        explicit LingzuEnableCmd(bool enable) : enable_(enable) {
        }

        std::vector<CANFrame> encode(uint32_t motor_id) const override;

    private:
        bool enable_;
    };

    // 停止命令
    class LingzuStopCmd : public LingzuCommand {
    public:
        // clear_fault: 是否同时清除故障
        explicit LingzuStopCmd(bool clear_fault = false) : clear_fault_(clear_fault) {
        }

        std::vector<CANFrame> encode(uint32_t motor_id) const override;

    private:
        bool clear_fault_;
    };

    // 设置机械零位命令
    class LingzuSetZeroCmd : public LingzuCommand {
    public:
        std::vector<CANFrame> encode(uint32_t motor_id) const override;
    };

    // 设置CAN ID命令
    class LingzuSetCanIdCmd : public LingzuCommand {
    public:
        explicit LingzuSetCanIdCmd(uint8_t new_can_id) : new_can_id_(new_can_id) {
        }

        std::vector<CANFrame> encode(uint32_t motor_id) const override;

    private:
        uint8_t new_can_id_;
    };

    // 保存数据命令
    class LingzuSaveDataCmd : public LingzuCommand {
    public:
        std::vector<CANFrame> encode(uint32_t motor_id) const override;
    };

    // 主动上报控制命令
    class LingzuAutoReportCmd : public LingzuCommand {
    public:
        explicit LingzuAutoReportCmd(bool enable): enable_(enable) {
        }

        std::vector<CANFrame> encode(uint32_t motor_id) const override;

    private:
        bool enable_;
        uint8_t interval_ms_;
    };

    // 单个参数写入命令 (用于高级配置)
    class LingzuWriteParamCmd : public LingzuCommand {
    public:
        LingzuWriteParamCmd(uint16_t index, int32_t value)
            : index_(index), value_(value) {
        }

        std::vector<CANFrame> encode(uint32_t motor_id) const override;

    private:
        uint16_t index_;    // 参数索引
        int32_t value_;     // 参数值
    };

} // namespace robot::motor::lingzu
