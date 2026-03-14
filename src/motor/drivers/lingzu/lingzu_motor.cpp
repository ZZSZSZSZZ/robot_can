/**
 * @Author: ZZSZSZSZZ
 * @CreateTime: 2026-03-12
 * @Description: 灵足电机驱动实现
 * @Version: 1.0
 */

#include "motor/drivers/lingzu/lingzu_motor.hpp"
#include "motor/drivers/lingzu/lingzu_protocol_constants.hpp"
#include "motor/drivers/lingzu/lingzu_units.hpp"

namespace robot::motor::lingzu {

    // ========== 构造/析构 ==========

    LingzuMotor::LingzuMotor(const MotorConfig &config)
            : AutoReportMotor(config) {
        const auto *found = LingzuSpecRegistry::find(config.type);
        spec_ = found ? *found : Specs::LINGZU_RS05;
        // 设置默认上报超时时间(灵足默认10ms上报间隔，设置100ms超时)
        setReportTimeout(100);
    }

    LingzuMotor::LingzuMotor(const MotorConfig &config, const LingzuMotorSpec &spec)
            : AutoReportMotor(config), spec_(spec) {
        setReportTimeout(100);
    }

    // ========== 基本信息 ==========

    const std::string &LingzuMotor::type() const {
        return spec_.type;
    }

    // ========== 基础控制 ==========

    bool LingzuMotor::enable() {
        updateStatePartial([](MotorState &state) {
            state.state_machine = MotorStateMachine::Enabling;
        });
        // 1. 发送使能命令
        enqueueCommand(LingzuEnableCmd(true));
        // 2. 开启主动上报（灵足电机需要此命令才会定期上报状态）
        enqueueCommand(LingzuAutoReportCmd(true));

        updateStatePartial([](MotorState &state) {
            state.state_machine = MotorStateMachine::Enabled;
        });
        return true;
    }

    bool LingzuMotor::disable() {
        updateStatePartial([](MotorState &state) {
            state.state_machine = MotorStateMachine::Disabling;
        });
        // 1. 关闭主动上报
        enqueueCommand(LingzuAutoReportCmd(false));
        // 2. 灵足电机没有专门的失能命令，使用停止命令
        enqueueCommand(LingzuStopCmd(false));

        updateStatePartial([](MotorState &state) {
            state.state_machine = MotorStateMachine::Disabled;
        });
        return true;
    }

    bool LingzuMotor::emergencyStop() {
        enqueueCommand(LingzuStopCmd(false));
        return true;
    }

    bool LingzuMotor::clearFault() {
        enqueueCommand(LingzuStopCmd(true));  // 停止命令带清故障标志
        return true;
    }

    bool LingzuMotor::setZeroPosition() {
        enqueueCommand(LingzuSetZeroCmd());
        return true;
    }

    // ========== 运动控制 ==========

    bool LingzuMotor::command(const MotorCommand &cmd) {
        const auto *lingzu_cmd = dynamic_cast<const LingzuCommand *>(&cmd);
        if (!lingzu_cmd) {
            return false;
        }
        enqueueCommand(*lingzu_cmd);
        return true;
    }

    bool LingzuMotor::setPosition(double position_rad, double max_vel, double max_torque) {
        // 灵足电机使用运控模式，需要Kp、Kd参数
        // 使用配置中的默认值或计算合适的值
        const double kp = spec_.kp_max * 0.5;  // 使用50%的最大Kp
        const double kd = spec_.kd_max * 0.5;  // 使用50%的最大Kd
        const double vel = max_vel > 0 ? max_vel : config_.default_velocity;

        auto cmd = std::make_unique<LingzuMITCmd>(position_rad, vel, 0.0, kp, kd);
        return command(*cmd);
    }

    bool LingzuMotor::setVelocity(double velocity_rad_s, double max_current) {
        // 灵足电机主要通过位置控制实现速度控制
        // 这里发送一个较远的距离配合速度限制来实现
        const double current_pos = getCurrentStateSnapshot().position;
        const double target_pos = current_pos + (velocity_rad_s * 0.1);  // 100ms的提前量

        const double kp = spec_.kp_max * 0.3;  // 速度模式下使用较小的Kp
        const double kd = spec_.kd_max * 0.8;  // 速度模式下使用较大的Kd

        auto cmd = std::make_unique<LingzuMITCmd>(target_pos, velocity_rad_s, 0.0, kp, kd);
        return command(*cmd);
    }

    bool LingzuMotor::setTorque(double torque_nm) {
        // 灵足电机的运控模式通过Kp/Kd间接控制力矩
        // 降低Kp，使系统更像力矩控制
        const double current_pos = getCurrentStateSnapshot().position;
        const double kp = 0.0;  // 无力矩时无位置反馈
        const double kd = 0.0;  // 无力矩时无速度反馈

        // 通过目标位置偏差来产生力矩
        double torque_factor = torque_nm / spec_.rated_torque_nm;
        const double target_pos = current_pos + (torque_factor * 0.1);  // 小偏差产生力矩

        auto cmd = std::make_unique<LingzuMITCmd>(target_pos, 0.0, 0.0, kp, kd);
        return command(*cmd);
    }

    bool LingzuMotor::setCurrent(double current_a) {
        // 转换为力矩后调用setTorque
        double torque = current_a * (spec_.rated_torque_nm / (spec_.rated_current_ma / 1000.0));
        return setTorque(torque);
    }

    bool LingzuMotor::setMIT(double position_rad, double velocity_rad_s, double torque_nm, double kp, double kd) {
        // 灵足电机通过运控模式实现MIT控制
        // 创建运控模式命令，包含力矩参数（编码到CAN ID的bit23-8）
        auto cmd = std::make_unique<LingzuMITCmd>(position_rad, velocity_rad_s, torque_nm, kp, kd);
        return command(*cmd);
    }

    // ========== 故障管理 ==========

    uint32_t LingzuMotor::getFaultCode() const {
        return getCurrentStateSnapshot().alarm_code;
    }

    std::vector<std::string> LingzuMotor::getFaultDescriptions() const {
        const uint32_t fault_code = getCurrentStateSnapshot().alarm_code;
        return decodeFaultCode(static_cast<uint8_t>(fault_code));
    }

    // ========== CAN通信 ==========

    uint32_t LingzuMotor::getReceiveCanId() const {
        // 注册时使用基础CAN ID（bit7-0），不是完整ID
        // Route会根据帧ID提取基础ID来匹配
        return static_cast<uint32_t>(config_.id);
    }

    void LingzuMotor::onCANFrameReceived(const can::CANFrame &frame) {
        MotorState state;

        if (decodeFrame(frame, state)) {
            updateState(state);
            // 更新最后上报时间
            updateLastReportTime();
        }
    }

    // ========== LingzuMotor 特有接口 ==========

    bool LingzuMotor::writeParameter(uint16_t index, int32_t value) {
        enqueueCommand(LingzuWriteParamCmd(index, value));
        return true;
    }

    bool LingzuMotor::saveData() {
        enqueueCommand(LingzuSaveDataCmd());
        return true;
    }

    // ========== 私有方法 ==========

    bool LingzuMotor::decodeFrame(const can::CANFrame &frame, MotorState &state) {
        // 检查帧类型 - 必须是扩展帧
        if (!frame.format.isExtendedId) {
            return false;
        }

        // 检查数据长度
        if (frame.data.size() < 8) {
            return false;
        }

        // 解析CAN ID
        uint32_t can_id = frame.id;
        uint8_t cmd_type = LingzuUnits::parseResponseCmdType(can_id);
        uint8_t mode_state = LingzuUnits::parseResponseModeState(can_id);
        uint8_t fault_code = LingzuUnits::parseResponseFaultCode(can_id);
        uint8_t resp_can_id = LingzuUnits::parseResponseCanId(can_id);

        // 验证CAN ID是否匹配
        if (resp_can_id != static_cast<uint8_t>(getReceiveCanId())) {
            return false;
        }

        // 更新当前模式状态
        current_mode_state_ = mode_state;

        // 复制当前状态作为基础
        state = getCurrentStateSnapshot();
        state.timestamp = std::chrono::steady_clock::now();

        // 根据命令类型处理数据
        if (cmd_type == CmdType::FEEDBACK || cmd_type == CmdType::AUTO_REPORT) {
            // 类型2: 电机反馈数据
            // Byte0~1: 当前角度
            // Byte2~3: 当前角速度
            // Byte4~5: 当前力矩
            // Byte6~7: 当前温度

            uint16_t pos_raw = parseUint16BE(frame.data, 0);
            uint16_t vel_raw = parseUint16BE(frame.data, 2);
            uint16_t torque_raw = parseUint16BE(frame.data, 4);
            uint16_t temp_raw = parseUint16BE(frame.data, 6);

            state.position = LingzuUnits::positionToRadians(pos_raw);
            state.velocity = LingzuUnits::velocityToRadPerSec(vel_raw);
            state.torque = LingzuUnits::rawToTorque(torque_raw);
            state.temperature = LingzuUnits::rawToCelsius(temp_raw);

            // 更新故障码
            state.alarm_code = fault_code;

            // 如果存在故障，更新状态机
            if (fault_code != 0) {
                state.state_machine = MotorStateMachine::Fault;
            }

            return true;
        }

        return false;
    }

    uint16_t LingzuMotor::parseUint16BE(const std::vector<uint8_t> &data, size_t offset) {
        if (data.size() < offset + 2) return 0;
        return (static_cast<uint16_t>(data[offset]) << 8) |
               static_cast<uint16_t>(data[offset + 1]);
    }

    std::vector<std::string> LingzuMotor::decodeFaultCode(uint8_t fault_code) const {
        std::vector<std::string> faults;

        if (fault_code == Fault::NONE) {
            return faults;
        }

        if (fault_code & Fault::UNDEFINED) {
            faults.push_back("UNDEFINED: 电机未标定");
        }
        if (fault_code & Fault::STALL_OVERLOAD) {
            faults.push_back("STALL_OVERLOAD: 堵转过载故障");
        }
        if (fault_code & Fault::ENCODER_FAULT) {
            faults.push_back("ENCODER_FAULT: 磁编码器故障");
        }
        if (fault_code & Fault::OVER_TEMP) {
            faults.push_back("OVER_TEMP: 过温");
        }
        if (fault_code & Fault::DRIVER_FAULT) {
            faults.push_back("DRIVER_FAULT: 驱动故障");
        }
        if (fault_code & Fault::UNDER_VOLTAGE) {
            faults.push_back("UNDER_VOLTAGE: 欠压故障");
        }

        return faults;
    }

} // namespace robot::motor::lingzu
