/**
 * @Author: ZZSZSZSZZ
 * @CreateTime: 2026-03-06
 * @Description: 意优电机驱动实现 - 重构版
 * @Version: 2.0
 */

#include "motor/drivers/eyou/eyou_motor.hpp"
#include "motor/drivers/eyou/eyou_protocol_constants.hpp"
#include "motor/drivers/eyou/eyou_units.hpp"
#include "can/can_coding.hpp"

namespace robot::motor::eyou {
    // ========== 构造/析构 ==========

    EYOUMotor::EYOUMotor(const MotorConfig &config)
        : PollingMotor(config) {
        const auto *found = EYOUSpecRegistry::find(config.type);
        spec_ = found ? *found : Specs::EYOU_PP11;
    }

    EYOUMotor::EYOUMotor(const MotorConfig &config, const EYOUMotorSpec &spec)
        : PollingMotor(config), spec_(spec) {
    }

    // ========== 基本信息 ==========

    const std::string &EYOUMotor::type() const {
        static const std::string type_prefix = "EYOU_";
        return spec_.type;
    }

    // ========== 基础控制 ==========

    bool EYOUMotor::enable() {
        // 更新状态机为 Enabling
        updateStatePartial([](MotorState &state) {
            state.state_machine = MotorStateMachine::Enabling;
        });
        enqueueCommand(EYOUEnableCmd(true));
        return true;
    }

    bool EYOUMotor::disable() {
        // 更新状态机为 Disabling
        updateStatePartial([](MotorState &state) {
            state.state_machine = MotorStateMachine::Disabling;
        });
        enqueueCommand(EYOUEnableCmd(false));
        return true;
    }

    bool EYOUMotor::emergencyStop() {
        enqueueCommand(EYOUEmergencyStopCmd());
        return true;
    }

    bool EYOUMotor::clearFault() {
        enqueueCommand(EYOUClearFaultCmd());
        return true;
    }

    bool EYOUMotor::setZeroPosition() {
        enqueueCommand(EYOUSetZeroCmd());
        return true;
    }

    // ========== 运动控制 ==========

    bool EYOUMotor::command(const MotorCommand &cmd) {
        const auto *eyou_cmd = dynamic_cast<const EYOUCommand *>(&cmd);
        if (!eyou_cmd) {
            return false;
        }
        enqueueCommand(*eyou_cmd);
        return true;
    }

    bool EYOUMotor::setPosition(double position_rad, double max_vel, double max_torque) {
        const double vel = max_vel > 0 ? max_vel : config_.default_velocity;
        const double acc = config_.default_acceleration;
        const double torque = max_torque > 0 ? max_torque : config_.default_torque;

        auto cmd = makeProfilePositionCmd(position_rad, vel, torque, acc, acc);
        return command(*cmd);
    }

    bool EYOUMotor::setVelocity(double velocity_rad_s, double max_current) {
        auto cmd = makeVelocityCmd(velocity_rad_s, max_current);
        return command(*cmd);
    }

    bool EYOUMotor::setTorque(double torque_nm) {
        auto cmd = makeTorqueCmd(torque_nm);
        return command(*cmd);
    }

    bool EYOUMotor::setCurrent(double current_a) {
        return setTorque(EYOUUnits::currentToTorque(current_a * 1000.0, spec_));
    }

    // ========== 故障管理 ==========

    uint32_t EYOUMotor::getFaultCode() const {
        return getCurrentStateSnapshot().alarm_code;
    }

    std::vector<std::string> EYOUMotor::getFaultDescriptions() const {
        const uint32_t alarm_code = getCurrentStateSnapshot().alarm_code;
        std::vector<std::string> alarms;
        if (alarm_code & Alarm::OVER_VOLTAGE) alarms.push_back("OVER_VOLTAGE");
        if (alarm_code & Alarm::UNDER_VOLTAGE) alarms.push_back("UNDER_VOLTAGE");
        if (alarm_code & Alarm::OVER_TEMP) alarms.push_back("OVER_TEMP");
        if (alarm_code & Alarm::OVER_CURRENT) alarms.push_back("OVER_CURRENT");
        if (alarm_code & Alarm::OVERLOAD) alarms.push_back("OVERLOAD");
        if (alarm_code & Alarm::MOTOR_LOCK) alarms.push_back("MOTOR_LOCK");
        if (alarm_code & Alarm::PHASE_LOSS) alarms.push_back("PHASE_LOSS");
        if (alarm_code & Alarm::PARAM_ERROR) alarms.push_back("PARAM_ERROR");
        if (alarm_code & Alarm::ENCODER_MAG_ERROR) alarms.push_back("ENCODER_MAG_ERROR");
        if (alarm_code & Alarm::ENCODER_UVLO) alarms.push_back("ENCODER_UVLO");
        if (alarm_code & Alarm::ENCODER_ANGLE_ERROR) alarms.push_back("ENCODER_ANGLE_ERROR");
        return alarms;
    }

    // ========== CAN通信 ==========

    void EYOUMotor::onCANFrameReceived(const can::CANFrame &frame) {
        MotorState state;

        if (decodeFrame(frame, state)) {
            updateState(state);
        }
    }

    void EYOUMotor::generatePollFrames(std::vector<can::CANFrame> &out_frames) {
        const uint32_t can_id = getTransmitCanId();

        // 位置每轮询必查
        out_frames.push_back(EYOUReadCmd(Addr::POSITION_VALUE).encode(can_id)[0]);

        // 速度和电流按分频查询
        if (shouldPollVelocity()) {
            out_frames.push_back(EYOUReadCmd(Addr::VELOCITY_VALUE).encode(can_id)[0]);
            out_frames.push_back(EYOUReadCmd(Addr::CURRENT_VALUE).encode(can_id)[0]);
        }

        // 温度低频查询
        if (shouldPollTemperature()) {
            out_frames.push_back(EYOUReadCmd(Addr::TEMPERATURE).encode(can_id)[0]);
        }

        // 使能状态和故障状态查询（每轮询都查，用于状态机更新）
        out_frames.push_back(EYOUReadCmd(Addr::ENABLE_STATE).encode(can_id)[0]);
        if (getPollCount() % 5 == 0) {  // 故障状态低频查询
            out_frames.push_back(EYOUReadCmd(Addr::ALARM_STATUS).encode(can_id)[0]);
        }
    }

    void EYOUMotor::generateStateOnlyPollFrames(std::vector<can::CANFrame> &out_frames) {
        // 只查询使能状态和故障状态
        const uint32_t can_id = getTransmitCanId();
        out_frames.push_back(EYOUReadCmd(Addr::ENABLE_STATE).encode(can_id)[0]);
        out_frames.push_back(EYOUReadCmd(Addr::ALARM_STATUS).encode(can_id)[0]);
    }

    // ========== EYOUMotor 特有接口 ==========

    std::unique_ptr<EYOUProfilePositionCmd> EYOUMotor::makeProfilePositionCmd(
        double pos, double vel, double torque, double acc, double dec) {
        return std::make_unique<EYOUProfilePositionCmd>(pos, vel, torque, acc, dec, spec_);
    }

    std::unique_ptr<EYOUVelocityCmd> EYOUMotor::makeVelocityCmd(double vel, double max_current) {
        return std::make_unique<EYOUVelocityCmd>(vel, max_current);
    }

    std::unique_ptr<EYOUTorqueCmd> EYOUMotor::makeTorqueCmd(double torque_nm) {
        return std::make_unique<EYOUTorqueCmd>(torque_nm, spec_);
    }

    // ========== 私有方法 ==========

    bool EYOUMotor::decodeFrame(const can::CANFrame &frame, MotorState &state) {
        if (frame.data.size() < 6) return false;

        const uint8_t cmd = frame.data[0];
        const uint8_t addr = frame.data[1];

        if (cmd != Cmd::READ_REPLY && cmd != Cmd::WRITE_REPLY) return false;

        // 使用基类辅助方法解析32位数据
        const int32_t value = extractInt32BE(frame.data, 2);

        // 复制当前状态作为基础（使用基类方法）
        state = getCurrentStateSnapshot();

        state.timestamp = std::chrono::steady_clock::now();

        // 根据寄存器地址解析数据
        switch (addr) {
            case Addr::POSITION_VALUE:
                state.position = EYOUUnits::pulsesToRadians(value);
                break;
            case Addr::VELOCITY_VALUE:
                state.velocity = EYOUUnits::pulsesToRadPerSec(value);
                break;
            case Addr::CURRENT_VALUE:
                state.current = EYOUUnits::milliampsToAmps(value);
                state.torque = EYOUUnits::currentToTorque(value, spec_);
                break;
            case Addr::TEMPERATURE:
                state.temperature = static_cast<double>(value);
                break;
            case Addr::ALARM_STATUS:
                state.alarm_code = static_cast<uint32_t>(value);
                // 根据故障状态更新状态机
                if (value != 0) {
                    state.state_machine = MotorStateMachine::Fault;
                } else if (state.state_machine == MotorStateMachine::Fault) {
                    // 故障清除，恢复到 Disabled 状态
                    state.state_machine = MotorStateMachine::Disabled;
                }
                break;
            case Addr::ENABLE_STATE:
                // 根据使能状态更新状态机
                if (value != 0) {
                    // 电机已使能
                    if (state.state_machine != MotorStateMachine::Enabled) {
                        state.state_machine = MotorStateMachine::Enabled;
                    }
                } else {
                    // 电机未使能
                    if (state.state_machine != MotorStateMachine::Disabled &&
                        state.state_machine != MotorStateMachine::Fault) {
                        state.state_machine = MotorStateMachine::Disabled;
                    }
                }
                break;
            default:
                return false;
        }
        return true;
    }
}
