/**
 * @Author: ZZSZSZSZZ
 * @CreateTime: 2026-03-06
 * @Description: 意优电机驱动 - 重构版，继承标准抽象层次
 * @Version: 2.0
 */

#pragma once

#include <memory>
#include <mutex>

#include "eyou_protocol_constants.hpp"
#include "motor/drivers/base/polling_motor.hpp"
#include "eyou_command.hpp"

namespace robot::motor::eyou {
    /**
     * @brief 意优电机驱动类
     * 
     * 继承PollingMotor，专注于实现意优协议特定的功能：
     * - 协议命令编码（使用EYOUCommand类）
     * - CAN帧解码（意优协议格式）
     * - 轮询帧生成（意优寄存器地址）
     */
    class EYOUMotor final : public PollingMotor {
    public:
        /// 构造 (通过 type 自动查找规格)
        /// @param config 电机配置
        explicit EYOUMotor(const MotorConfig &config);

        /// 构造 (直接指定规格)
        /// @param config 电机配置
        /// @param spec 电机额定参数
        EYOUMotor(const MotorConfig &config, const EYOUMotorSpec &spec);

        ~EYOUMotor() override = default;

        // ========== Motor 接口实现 ==========

        const std::string &type() const override;

        bool enable() override;
        bool disable() override;
        bool emergencyStop() override;
        bool clearFault() override;
        bool setZeroPosition() override;

        bool command(const MotorCommand &cmd) override;

        bool setPosition(double position_rad, double max_vel, double max_torque) override;
        bool setVelocity(double velocity_rad_s, double max_current) override;
        bool setTorque(double torque_nm) override;
        bool setCurrent(double current_a) override;

        uint32_t getFaultCode() const override;
        std::vector<std::string> getFaultDescriptions() const override;

        // ========== CANMotor 接口实现 ==========

        void onCANFrameReceived(const can::CANFrame &frame) override;

        // ========== PollingMotor 接口实现 ==========

        void generatePollFrames(std::vector<can::CANFrame> &out_frames) override;

        void generateStateOnlyPollFrames(std::vector<can::CANFrame> &out_frames) override;

        // ========== EYOUMotor 特有接口 ==========

        /// 获取意优特定状态
        /// @return EYOUMotorState
        EYOUMotorState getEYOUState() const;

        /// 创建轮廓位置命令
        /// @param pos 位置(弧度)
        /// @param vel 速度
        /// @param torque 力矩
        /// @param acc 加速度
        /// @param dec 减速度
        /// @return 命令对象
        std::unique_ptr<EYOUProfilePositionCmd> makeProfilePositionCmd(
            double pos, double vel, double torque, double acc, double dec);

        /// 创建速度命令
        /// @param vel 速度
        /// @param max_current 最大电流
        /// @return 命令对象
        static std::unique_ptr<EYOUVelocityCmd> makeVelocityCmd(double vel, double max_current);

        /// 创建力矩命令
        /// @param torque_nm 力矩
        /// @return 命令对象
        std::unique_ptr<EYOUTorqueCmd> makeTorqueCmd(double torque_nm);

        /// 类型转换辅助函数
        static std::shared_ptr<EYOUMotor> from(std::shared_ptr<Motor> motor);
        static std::shared_ptr<EYOUMotor> cast(std::shared_ptr<Motor> motor);
        static bool is(std::shared_ptr<Motor> motor);

    private:
        EYOUMotorSpec spec_;                    ///< 电机规格
        EYOUMotorState eyou_state_;             ///< 意优特定状态
        mutable std::mutex eyou_mutex_;         ///< 意优状态保护锁

        // ========== 协议特定方法 ==========

        /// 解码CAN帧
        /// @param frame 接收到的帧
        /// @param out_state 输出的通用状态
        /// @param out_eyou 输出的意优状态
        /// @return 是否成功解码
        bool decodeFrame(const can::CANFrame &frame, MotorState &out_state, EYOUMotorState &out_eyou);

        /// 更新意优特定状态
        /// @param state 新的意优状态
        void updateEYOUState(const EYOUMotorState &state);
    };
} // namespace robot::motor::eyou
