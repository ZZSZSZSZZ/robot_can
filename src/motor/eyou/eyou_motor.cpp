/**
 * @Author: ZZSZSZSZZ
 * @CreateTime: 2026-02-24
 * @Description: 
 * @Version: 1.0
 */

#include "motor/drivers/eyou/eyou_motor.hpp"
#include "motor/drivers/eyou/eyou_protocol_constants.hpp"
#include "motor/drivers/eyou/eyou_units.hpp"

namespace robot::motor::eyou {
    // 辅助函数
    static CANFrame makeFrame(uint32_t id, uint8_t cmd, uint8_t addr, int32_t data) {
        std::vector<uint8_t> bytes = {
            cmd, addr,
            static_cast<uint8_t>(data >> 24 & 0xFF),
            static_cast<uint8_t>(data >> 16 & 0xFF),
            static_cast<uint8_t>(data >> 8 & 0xFF),
            static_cast<uint8_t>(data & 0xFF),
            0x00, 0x00
        };
        return CANFrame::makeStandard(id, bytes);
    }

    EYOUMotor::EYOUMotor(const MotorConfig &config) : BaseMotor(config) {
        const auto *found = EYOUSpecRegistry::find(config.type);
        spec_ = found ? *found : Specs::EYOU_PP08; // 默认规格
    }

    EYOUMotor::EYOUMotor(const MotorConfig &config, const EYOUMotorSpec &spec) : BaseMotor(config), spec_(spec) {
    }

    const std::string &EYOUMotor::type() const {
        static const std::string type = spec_.type;
        return type;
    }

    MotorCapability EYOUMotor::capabilities() const {
        return MotorCapability::Enable |
               MotorCapability::Disable |
               MotorCapability::EmergencyStop |
               MotorCapability::ClearFault |
               MotorCapability::SetZero |
               MotorCapability::PositionControl |
               MotorCapability::VelocityControl |
               MotorCapability::TorqueControl |
               MotorCapability::CurrentControl |
               MotorCapability::ProfilePosition |
               MotorCapability::ProfileVelocity |
               MotorCapability::FeedbackPosition |
               MotorCapability::FeedbackVelocity |
               MotorCapability::FeedbackTorque |
               MotorCapability::FeedbackCurrent |
               MotorCapability::FeedbackTemperature |
               MotorCapability::FeedbackVoltage |
               MotorCapability::PollStatus |
               MotorCapability::ConfigurableCanId |
               MotorCapability::ConfigurableBaudrate |
               MotorCapability::SaveToFlash;
    }

    bool EYOUMotor::enable() {
        EYOUEnableCmd cmd(true);
        enqueueFrames(cmd.encode(config_.id));
        return true;
    }

    bool EYOUMotor::disable() {
        EYOUEnableCmd cmd(false);
        enqueueFrames(cmd.encode(config_.id));
        return true;
    }

    bool EYOUMotor::emergencyStop() {
        EYOUEmergencyStopCmd cmd;
        enqueueFrames(cmd.encode(config_.id));
        return true;
    }

    bool EYOUMotor::clearFault() {
        EYOUClearFaultCmd cmd;
        enqueueFrames(cmd.encode(config_.id));
        return true;
    }

    bool EYOUMotor::setZeroPosition() {
        EYOUSetZeroCmd cmd;
        enqueueFrames(cmd.encode(config_.id));
        return true;
    }

    bool EYOUMotor::command(const MotorCommand &cmd) {
        const auto eyou_cmd = dynamic_cast<const EYOUCommand *>(&cmd);
        if (!eyou_cmd) {
            return false;
        }
        enqueueFrames(eyou_cmd->encode(config_.id));
        return true;
    }

    bool EYOUMotor::setPosition(double position_rad, double max_vel, double max_torque) {
        double vel = max_vel > 0 ? max_vel : config_.default_velocity;
        double acc = config_.default_acceleration;
        double torque = max_torque > 0 ? max_torque : config_.default_torque_limit;

        auto cmd = makeProfilePositionCmd(position_rad, vel, acc, acc, torque);
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

    bool EYOUMotor::configure(const MotorConfig &config) {
        config_ = config;
        return true;
    }

    MotorConfig EYOUMotor::getConfig() const {
        return config_;
    }

    bool EYOUMotor::saveConfig() {
        enqueueFrames({makeFrame(config_.id, Cmd::FAST_WRITE, Addr::SAVE_DATA, 0x00000001)});
        return true;
    }

    void EYOUMotor::onCANFrameReceived(const CANFrame &frame) {
        MotorState state;
        EYOUMotorState eyou_state;

        if (decodeFrame(frame, state, eyou_state)) {
            updateState(state);
            updateEYOUState(eyou_state);
        }
    }

    std::vector<CANFrame> EYOUMotor::onStatusPoll() {
        std::lock_guard lock(pending_mutex_);

        auto frames = std::move(pending_frames_);
        pending_frames_.clear();

        uint32_t id = config_.id;
        frames.push_back(makeFrame(id, Cmd::READ, Addr::POSITION_VALUE, 0));
        frames.push_back(makeFrame(id, Cmd::READ, Addr::VELOCITY_VALUE, 0));
        frames.push_back(makeFrame(id, Cmd::READ, Addr::CURRENT_VALUE, 0));
        frames.push_back(makeFrame(id, Cmd::READ, Addr::BUS_VOLTAGE, 0));
        frames.push_back(makeFrame(id, Cmd::READ, Addr::TEMPERATURE, 0));
        frames.push_back(makeFrame(id, Cmd::READ, Addr::ALARM_STATUS, 0));
        frames.push_back(makeFrame(id, Cmd::READ, Addr::ENABLE_STATE, 0));

        return frames;
    }

    EYOUMotorState EYOUMotor::getEYOUState() const {
        std::lock_guard lock(eyou_mutex_);
        return eyou_state_;
    }

    std::unique_ptr<EYOUProfilePositionCmd> EYOUMotor::makeProfilePositionCmd(
        double pos, double vel, double acc, double dec, double torque) {
        return std::make_unique<EYOUProfilePositionCmd>(pos, vel, acc, dec, torque);
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

    void EYOUMotor::enqueueFrames(std::vector<CANFrame> frames) {
        std::lock_guard lock(pending_mutex_);
        pending_frames_.insert(pending_frames_.end(), frames.begin(), frames.end());
    }

    void EYOUMotor::updateEYOUState(const EYOUMotorState &state) {
        std::lock_guard lock(eyou_mutex_);
        eyou_state_ = state;
    }

    bool EYOUMotor::decodeFrame(const CANFrame &frame,MotorState &state, EYOUMotorState &eyou) {
        if (frame.data.size() < 6) return false;

        uint8_t cmd = frame.data[0];
        uint8_t addr = frame.data[1];

        if (cmd != Cmd::READ_REPLY && cmd != Cmd::WRITE_REPLY) return false;

        auto readInt32 = [&](size_t offset) -> int32_t {
            return static_cast<int32_t>(frame.data[offset]) << 24 |
                   static_cast<int32_t>(frame.data[offset + 1]) << 16 |
                   static_cast<int32_t>(frame.data[offset + 2]) << 8 |
                   static_cast<int32_t>(frame.data[offset + 3]);
        };

        int32_t value = readInt32(2);
        state.timestamp = std::chrono::steady_clock::now();

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
            case Addr::BUS_VOLTAGE:
                eyou.bus_voltage = value * 0.1;
                state.voltage = eyou.bus_voltage;
                break;
            case Addr::TEMPERATURE:
                state.temperature = static_cast<double>(value);
                break;
            case Addr::ALARM_STATUS:
                eyou.alarm_code = static_cast<uint32_t>(value);
                state.fault = value != 0;
                break;
            case Addr::ENABLE_STATE:
                state.enabled = value != 0;
                break;
            default:
                return false;
        }
        return true;
    }
}
