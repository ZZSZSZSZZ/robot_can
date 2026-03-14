/**
 * @Author: ZZSZSZSZZ
 * @CreateTime: 2026-03-12
 * @Description: 灵足电机驱动类
 * @Version: 1.0
 */

#pragma once

#include <memory>

#include "lingzu_protocol_constants.hpp"
#include "lingzu_specification.hpp"
#include "lingzu_command.hpp"
#include "motor/drivers/base/auto_report_motor.hpp"

namespace robot::motor::lingzu {
    /**
     * @brief 灵足电机驱动类
     *
     * 继承AutoReportMotor，实现灵足协议特定功能：
     * - 协议命令编码（使用LingzuCommand类）
     * - CAN帧解码（灵足协议格式 - 扩展帧）
     * - 自动上报状态处理
     *
     * 灵足协议特点：
     * - 使用CAN 2.0扩展帧格式
     * - 发送帧ID: bit28-24命令类型, bit23-8=力矩/0, bit7-0=目标电机canid
     * - 应答帧ID: bit28-24命令类型, bit22-23模式状态, bit21-16故障代码, bit15-8=当前电机CAN_ID, bit7-0=主机CAN_ID
     * - 运控模式(类型1)是主要控制方式，包含位置、速度、Kp、Kd参数
     * - 电机自动上报状态(类型2)，无需主动轮询
     */
    class LingzuMotor final : public AutoReportMotor {
    public:
        /// 构造 (通过 type 自动查找规格)
        explicit LingzuMotor(const MotorConfig& config);

        /// 构造 (直接指定规格)
        LingzuMotor(const MotorConfig& config, const LingzuMotorSpec& spec);

        ~LingzuMotor() override = default;

        // ========== Motor 接口实现 ==========

        const std::string& type() const override;

        bool enable() override;
        bool disable() override;
        bool emergencyStop() override;
        bool clearFault() override;
        bool setZeroPosition() override;

        bool command(const MotorCommand& cmd) override;

        bool setPosition(double position_rad, double max_vel, double max_torque) override;
        bool setVelocity(double velocity_rad_s, double max_current) override;
        bool setTorque(double torque_nm) override;
        bool setCurrent(double current_a) override;

        /// MIT模式控制 - 灵足电机通过运控模式实现
        /// @param position_rad 目标位置(弧度)
        /// @param velocity_rad_s 目标速度(弧度/秒)
        /// @param kp 位置增益
        /// @param kd 速度增益
        /// @param torque_nm 前馈力矩(牛米)
        /// @return 是否成功
        bool setMIT(double position_rad, double velocity_rad_s, double torque_nm, double kp, double kd) override;

        uint32_t getFaultCode() const override;
        std::vector<std::string> getFaultDescriptions() const override;

        // ========== CANMotor 接口实现 ==========

        void onCANFrameReceived(const can::CANFrame& frame) override;

        /// 获取接收CAN ID - 灵足电机应答帧基础ID
        uint32_t getReceiveCanId() const override;

        // ========== LingzuMotor 特有接口 ==========

        /// 写入参数
        /// @param index 参数索引
        /// @param value 参数值
        /// @return 是否成功
        bool writeParameter(uint16_t index, int32_t value);

        /// 保存数据到EEPROM
        /// @return 是否成功
        bool saveData();

        /// 类型转换辅助函数
        static std::shared_ptr<LingzuMotor> from(std::shared_ptr<Motor> motor) {
            return ptrFrom<LingzuMotor>(motor);
        }
        static std::shared_ptr<LingzuMotor> cast(std::shared_ptr<Motor> motor) {
            return ptrCast<LingzuMotor>(motor);
        }
        static bool is(std::shared_ptr<Motor> motor) {
            return ptrIs<LingzuMotor>(motor);
        }

    private:
        LingzuMotorSpec spec_;
        uint8_t current_mode_state_ = ModeState::RESET;

        // 解码CAN帧
        bool decodeFrame(const can::CANFrame& frame, MotorState& state);

        // 解析16位无符号整数（大端序）
        static uint16_t parseUint16BE(const std::vector<uint8_t>& data, size_t offset);

        // 获取故障描述
        std::vector<std::string> decodeFaultCode(uint8_t fault_code) const;
    };

} // namespace robot::motor::lingzu
