/**
 * @Author: ZZSZSZSZZ
 * @CreateTime: 2026-02-24
 * @Description: 
 * @Version: 1.0
 */

#include <utility>

#include "motor/drivers/eyou/eyou_motor.hpp"
#include "motor/drivers/eyou/eyou_protocol_constants.hpp"
#include "motor/drivers/eyou/eyou_units.hpp"
#include "can/can_coding.hpp"

namespace robot::motor::eyou {
    EYOUMotor::EYOUMotor(const MotorConfig &config) : BaseMotor(config) {
        const auto *found = EYOUSpecRegistry::find(config.type);
        spec_ = found ? *found : Specs::EYOU_PP11; // 默认规格
    }

    EYOUMotor::EYOUMotor(const MotorConfig &config, const EYOUMotorSpec &spec) : BaseMotor(config), spec_(spec) {
    }

    const std::string &EYOUMotor::type() const {
        static const std::string type = spec_.type;
        return type;
    }

    bool EYOUMotor::enable() {
        const EYOUEnableCmd cmd(true);
        enqueueFrames(cmd.encode(config_.id));
        return true;
    }

    bool EYOUMotor::disable() {
        const EYOUEnableCmd cmd(false);
        enqueueFrames(cmd.encode(config_.id));
        return true;
    }

    bool EYOUMotor::emergencyStop() {
        const EYOUEmergencyStopCmd cmd;
        enqueueFrames(cmd.encode(config_.id));
        return true;
    }

    bool EYOUMotor::clearFault() {
        const EYOUClearFaultCmd cmd;
        enqueueFrames(cmd.encode(config_.id));
        return true;
    }

    bool EYOUMotor::setZeroPosition() {
        const EYOUSetZeroCmd cmd;
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

    bool EYOUMotor::setPosition(const double position_rad, const double max_vel, const double max_torque) {
        const double vel = max_vel > 0 ? max_vel : config_.default_velocity;
        const double acc = config_.default_acceleration;
        const double torque = max_torque > 0 ? max_torque : config_.default_torque;

        const auto cmd = makeProfilePositionCmd(position_rad, vel, torque, acc, acc);
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

    bool EYOUMotor::setCurrent(double current_ma) {
        return setTorque(EYOUUnits::currentToTorque(current_ma, spec_));
    }

    bool EYOUMotor::configure(const MotorConfig &config) {
        config_ = config;
        return true;
    }

    MotorConfig EYOUMotor::getConfig() const {
        return config_;
    }

    bool EYOUMotor::saveConfig() {
        enqueueFrames({encodeFrame(Cmd::FAST_WRITE, Addr::SAVE_DATA, 0x00000001)});
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
        std::vector<CANFrame> frames;
        frames.reserve(64);

        {
            std::lock_guard lock(pending_mutex_);
            if (has_pending_frames_) {
                frames = std::move(pending_frames_);
                pending_frames_.clear();
                has_pending_frames_ = false;
            }
        }

        // frames.push_back(encodeFrame(Cmd::READ, Addr::POSITION_VALUE, 0));
        // frames.push_back(encodeFrame(Cmd::READ, Addr::VELOCITY_VALUE, 0));
        // frames.push_back(encodeFrame(Cmd::READ, Addr::CURRENT_VALUE, 0));
        // frames.push_back(encodeFrame(Cmd::READ, Addr::BUS_VOLTAGE, 0));
        // frames.push_back(encodeFrame(Cmd::READ, Addr::TEMPERATURE, 0));
        // frames.push_back(encodeFrame(Cmd::READ, Addr::ALARM_STATUS, 0));
        // frames.push_back(encodeFrame(Cmd::READ, Addr::ENABLE_STATE, 0));

        uint32_t cnt = poll_counter_.fetch_add(1);
        frames.push_back(encodeFrame(Cmd::READ, Addr::POSITION_VALUE, 0));

        if (cnt % 2 == 0) {
            frames.push_back(encodeFrame(Cmd::READ, Addr::VELOCITY_VALUE, 0));
            frames.push_back(encodeFrame(Cmd::READ, Addr::CURRENT_VALUE, 0));
        }

        return frames;
    }

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

    void EYOUMotor::enqueueFrames(std::vector<CANFrame> frames) {
        std::lock_guard lock(pending_mutex_);
        pending_frames_.insert(pending_frames_.end(), frames.begin(), frames.end());
        has_pending_frames_ = true;
    }

    CANFrame EYOUMotor::encodeFrame(uint8_t cmd, uint8_t addr, int32_t data) const {
        std::vector<uint8_t> payload = {
            cmd, addr,
            static_cast<uint8_t>(data >> 24 & 0xFF),
            static_cast<uint8_t>(data >> 16 & 0xFF),
            static_cast<uint8_t>(data >> 8 & 0xFF),
            static_cast<uint8_t>(data & 0xFF),
            0x00, 0x00
        };
        
        // 使用can_coding进行编码，自动选择最佳格式
        return can::CANFrameEncoder::encode(payload, config_.id, getFrameRequirement());
    }
    
    can::CANDeviceFrameRequirement EYOUMotor::getFrameRequirement() const {
        can::CANDeviceFrameRequirement req;
        req.preferredType = config_.tx_format.type;
        req.requireExtendedId = config_.tx_format.isExtendedId;
        req.maxDataLength = config_.tx_format.dlc;
        req.requireCanFd = config_.tx_format.type == can::CANFrameType::CanFd;
        return req;
    }

    void EYOUMotor::updateEYOUState(const EYOUMotorState &state) {
        std::lock_guard lock(eyou_mutex_);
        eyou_state_ = state;
    }

    bool EYOUMotor::decodeFrame(const CANFrame &frame, MotorState &state, EYOUMotorState &eyou) {
        if (frame.data.size() < 6) return false;

        uint8_t cmd = frame.data[0];
        uint8_t addr = frame.data[1];

        if (cmd != Cmd::READ_REPLY && cmd != Cmd::WRITE_REPLY) return false;

        int32_t value = 0;
        {
            const uint8_t buf[4] = {frame.data[2], frame.data[3], frame.data[4], frame.data[5]};
            const uint32_t uval = (static_cast<uint32_t>(buf[0]) << 24) |
                                  (static_cast<uint32_t>(buf[1]) << 16) |
                                  (static_cast<uint32_t>(buf[2]) << 8) |
                                  static_cast<uint32_t>(buf[3]);
            value = static_cast<int32_t>(uval);
        }

        {
            std::lock_guard lock(state_mutex_);
            state = current_state_;
        }

        eyou = {};
        eyou.raw_position = eyou_state_.raw_position; // 保留上次的有效值
        eyou.raw_velocity = eyou_state_.raw_velocity;
        eyou.raw_current = eyou_state_.raw_current;
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
            case Addr::TEMPERATURE:
                state.temperature = static_cast<double>(value);
                break;
            // case Addr::ALARM_STATUS:
            //     eyou.alarm_code = static_cast<uint32_t>(value);
            //     state.fault = value != 0;
            //     break;
            // case Addr::ENABLE_STATE:
            //     state.enabled = value != 0;
            //     break;
            default:
                return false;
        }
        return true;
    }
}
