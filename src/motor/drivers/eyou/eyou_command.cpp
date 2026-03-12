/**
 * @Author: ZZSZSZSZZ
 * @CreateTime: 2026-02-26
 * @Description: 
 * @Version: 1.0
 */

#include "motor/drivers/eyou/eyou_command.hpp"
#include "motor/drivers/eyou/eyou_protocol_constants.hpp"
#include "motor/drivers/eyou/eyou_units.hpp"
#include "can/can_coding.hpp"

namespace robot::motor::eyou {
    // 辅助函数 - 使用基类静态方法进行帧编码
    static CANFrame makeFrame(uint32_t id, uint8_t cmd, uint8_t addr, int32_t data) {
        std::vector<uint8_t> payload(8, 0x00);
        payload[0] = cmd;
        payload[1] = addr;
        // 使用基类的静态方法编码32位整数
        CANMotor::encodeInt32BE(data, payload, 2);

        // 使用标准CAN设备需求
        can::CANDeviceFrameRequirement req{};
        req.preferredType = can::CANFrameType::Standard;
        req.requireExtendedId = false;
        req.maxDataLength = 8;
        req.requireCanFd = false;

        return can::CANFrameEncoder::encode(payload, id, req);
    }

    // 命令实现
    std::vector<CANFrame> EYOUEnableCmd::encode(uint32_t motor_id) const {
        return {makeFrame(motor_id, Cmd::WRITE, Addr::ENABLE_STATE, enable_ ? 1 : 0)};
    }

    std::vector<CANFrame> EYOUProfilePositionCmd::encode(uint32_t motor_id) const {
        const double current_ma = EYOUUnits::torqueToCurrent(torque_, spec_);

        return {
            makeFrame(motor_id, Cmd::FAST_WRITE, Addr::WORK_MODE, static_cast<int32_t>(WorkMode::ProfilePosition)),
            makeFrame(motor_id, Cmd::FAST_WRITE, Addr::TARGET_VELOCITY, EYOUUnits::radPerSecToPulses(velocity_)),
            makeFrame(motor_id, Cmd::FAST_WRITE, Addr::TARGET_CURRENT, static_cast<int32_t>(current_ma)),
            makeFrame(motor_id, Cmd::FAST_WRITE, Addr::TARGET_ACCEL, EYOUUnits::radPerSecToPulses(accel_)),
            makeFrame(motor_id, Cmd::FAST_WRITE, Addr::TARGET_DECEL, EYOUUnits::radPerSecToPulses(decel_)),
            makeFrame(motor_id, Cmd::FAST_WRITE, Addr::TARGET_POSITION, EYOUUnits::radiansToPulses(position_)),
        };
    }

    std::vector<CANFrame> EYOUVelocityCmd::encode(uint32_t motor_id) const {
        return {
            makeFrame(motor_id, Cmd::FAST_WRITE, Addr::WORK_MODE, static_cast<int32_t>(WorkMode::Velocity)),
            makeFrame(motor_id, Cmd::FAST_WRITE, Addr::TARGET_VELOCITY, EYOUUnits::rpmToPulsesPerSec(velocity_))
        };
    }

    std::vector<CANFrame> EYOUTorqueCmd::encode(uint32_t motor_id) const {
        double current_ma = EYOUUnits::torqueToCurrent(torque_nm_, spec_);

        return {
            makeFrame(motor_id, Cmd::FAST_WRITE, Addr::WORK_MODE, static_cast<int32_t>(WorkMode::Current)),
            makeFrame(motor_id, Cmd::FAST_WRITE, Addr::TARGET_CURRENT, static_cast<int32_t>(current_ma))
        };
    }

    std::vector<CANFrame> EYOUEmergencyStopCmd::encode(uint32_t motor_id) const {
        return {makeFrame(motor_id, Cmd::WRITE, Addr::STOP_STATE, 1)};
    }

    std::vector<CANFrame> EYOUClearFaultCmd::encode(uint32_t motor_id) const {
        return {makeFrame(motor_id, Cmd::WRITE, Addr::ALARM_STATUS, 0)};
    }

    std::vector<CANFrame> EYOUSetZeroCmd::encode(uint32_t motor_id) const {
        return {makeFrame(motor_id, Cmd::WRITE, Addr::POS_OFFSET, 0)};
    }

    std::vector<CANFrame> EYOUReadCmd::encode(uint32_t motor_id) const {
        return {makeFrame(motor_id, Cmd::READ, addr_, 0)};
    }
}
