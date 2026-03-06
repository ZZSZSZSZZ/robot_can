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

    // ========== 配置管理 ==========

    bool EYOUMotor::saveConfig() {
        enqueueFrame(encodeEYOUFrame(Cmd::FAST_WRITE, Addr::SAVE_DATA, 0x00000001));
        return true;
    }

    // ========== 故障管理 ==========

    uint32_t EYOUMotor::getFaultCode() const {
        std::lock_guard lock(eyou_mutex_);
        return eyou_state_.alarm_code;
    }

    std::vector<std::string> EYOUMotor::getFaultDescriptions() const {
        std::lock_guard lock(eyou_mutex_);
        return eyou_state_.getAlarmDescriptions();
    }

    // ========== CAN通信 ==========

    void EYOUMotor::onCANFrameReceived(const can::CANFrame &frame) {
        MotorState state;
        EYOUMotorState eyou_state;

        if (decodeFrame(frame, state, eyou_state)) {
            updateState(state);
            updateEYOUState(eyou_state);
        }
    }

    void EYOUMotor::generatePollFrames(std::vector<can::CANFrame> &out_frames) {
        const uint32_t cnt = incrementPollCount();

        // 位置每轮询必查
        out_frames.push_back(makeReadFrame(Addr::POSITION_VALUE));

        // 速度和电流按分频查询
        if (cnt % polling_policy_.velocity_poll_divisor == 0) {
            out_frames.push_back(makeReadFrame(Addr::VELOCITY_VALUE));
            out_frames.push_back(makeReadFrame(Addr::CURRENT_VALUE));
        }

        // 温度低频查询
        if (cnt % polling_policy_.temperature_poll_divisor == 0) {
            out_frames.push_back(makeReadFrame(Addr::TEMPERATURE));
        }

        // 使能状态和故障状态查询（每轮询都查，用于状态机更新）
        out_frames.push_back(makeReadFrame(Addr::ENABLE_STATE));
        if (cnt % 5 == 0) {  // 故障状态低频查询
            out_frames.push_back(makeReadFrame(Addr::ALARM_STATUS));
        }
    }

    void EYOUMotor::generateStateOnlyPollFrames(std::vector<can::CANFrame> &out_frames) {
        // 只查询使能状态和故障状态
        out_frames.push_back(makeReadFrame(Addr::ENABLE_STATE));
        out_frames.push_back(makeReadFrame(Addr::ALARM_STATUS));
    }

    // ========== EYOUMotor 特有接口 ==========

    EYOUMotorState EYOUMotor::getEYOUState() const {
        std::lock_guard lock(eyou_mutex_);
        return eyou_state_;
    }

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

    std::shared_ptr<EYOUMotor> EYOUMotor::from(std::shared_ptr<Motor> motor) {
        return std::dynamic_pointer_cast<EYOUMotor>(motor);
    }

    std::shared_ptr<EYOUMotor> EYOUMotor::cast(std::shared_ptr<Motor> motor) {
        auto result = from(motor);
        if (!result) {
            throw std::bad_cast();
        }
        return result;
    }

    bool EYOUMotor::is(std::shared_ptr<Motor> motor) {
        return from(motor) != nullptr;
    }

    // ========== 私有方法 ==========

    void EYOUMotor::updateEYOUState(const EYOUMotorState &state) {
        std::lock_guard lock(eyou_mutex_);
        eyou_state_ = state;
    }

    can::CANFrame EYOUMotor::encodeEYOUFrame(uint8_t cmd, uint8_t addr, int32_t data) const {
        std::vector<uint8_t> payload = {
            cmd, addr,
            static_cast<uint8_t>((data >> 24) & 0xFF),
            static_cast<uint8_t>((data >> 16) & 0xFF),
            static_cast<uint8_t>((data >> 8) & 0xFF),
            static_cast<uint8_t>(data & 0xFF),
            0x00, 0x00
        };
        return encodeFrame(payload, getTransmitCanId());
    }

    bool EYOUMotor::decodeFrame(const can::CANFrame &frame, MotorState &state, EYOUMotorState &eyou) {
        if (frame.data.size() < 6) return false;

        uint8_t cmd = frame.data[0];
        uint8_t addr = frame.data[1];

        if (cmd != Cmd::READ_REPLY && cmd != Cmd::WRITE_REPLY) return false;

        // 解析32位数据
        int32_t value = 0;
        {
            const uint32_t uval = 
                (static_cast<uint32_t>(frame.data[2]) << 24) |
                (static_cast<uint32_t>(frame.data[3]) << 16) |
                (static_cast<uint32_t>(frame.data[4]) << 8) |
                static_cast<uint32_t>(frame.data[5]);
            value = static_cast<int32_t>(uval);
        }

        // 复制当前状态作为基础
        {
            std::lock_guard lock(state_mutex_);
            state = current_state_;
        }

        // 复制上次的意优状态
        {
            std::lock_guard lock(eyou_mutex_);
            eyou = eyou_state_;
        }

        state.timestamp = std::chrono::steady_clock::now();

        // 根据寄存器地址解析数据
        switch (addr) {
            case Addr::POSITION_VALUE:
                eyou.raw_position = value;
                state.position = EYOUUnits::pulsesToRadians(value);
                break;
            case Addr::VELOCITY_VALUE:
                eyou.raw_velocity = value;
                state.velocity = EYOUUnits::pulsesToRadPerSec(value);
                break;
            case Addr::CURRENT_VALUE:
                eyou.raw_current = value;
                state.current = EYOUUnits::milliampsToAmps(value);
                eyou.estimated_torque = EYOUUnits::currentToTorque(value, spec_);
                state.torque = eyou.estimated_torque;
                break;
            case Addr::TEMPERATURE:
                state.temperature = static_cast<double>(value);
                break;
            case Addr::ALARM_STATUS:
                eyou.alarm_code = static_cast<uint32_t>(value);
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
} // namespace robot::motor::eyou
