/**
 * @Author: ZZSZSZSZZ
 * @CreateTime: 2026-02-24
 * @Description: 
 * @Version: 1.0
 */

#pragma once

#include "motor/motor_interface.hpp"
#include "eyou_types.hpp"

namespace robot::motor::eyou {
    class EYOUMotor : public BaseMotor {
    public:
        /// 意优电机构造 (通过 type 自动查找规格)
        /// @param config 电机配置
        explicit EYOUMotor(const MotorConfig &config);

        /// 意优电机构造 (直接指定规格)
        /// @param config 电机配置
        /// @param spec 电机额定参数
        explicit EYOUMotor(const MotorConfig &config, const EYOUMotorSpec &spec);

        ~EYOUMotor() override = default;

        // ========== Motor 抽象接口 ==========

        /// 获取电机型号
        /// @return 电机型号字符串
        const std::string &type() const override;

        /// 电机能力查询
        MotorCapability capabilities() const override;

        /// 使能
        /// @return 是否成功
        bool enable() override;

        /// 失能
        /// @return 是否成功
        bool disable() override;

        /// 急停
        /// @return 是否成功
        bool emergencyStop() override;

        /// 清空报错
        /// @return 是否成功
        bool clearFault() override;

        /// 设置零位
        /// @return 是否成功
        bool setZeroPosition() override;

        /// 控制电机
        /// @param cmd 控制命令
        /// @return 是否成功
        bool command(const MotorCommand &cmd) override;

        /// 位置控制 (轮廓位置模式)
        /// @param position_rad 位置 - 弧度
        /// @param max_vel 最大速度
        /// @param max_torque 最大力矩
        /// @return 是否成功
        bool setPosition(double position_rad, double max_vel, double max_torque) override;

        /// 速度控制 (速度模式)
        /// @param velocity_rad_s 速度 - 弧度每秒
        /// @param max_current 最大电流
        /// @return 是否成功
        bool setVelocity(double velocity_rad_s, double max_current) override;

        /// 力矩控制 (电流模式)
        /// @param torque_nm 力矩 - 牛米
        /// @return 是否成功
        bool setTorque(double torque_nm) override;

        /// 电流控制 (电流模式)
        /// @param current_ma 电流 - 毫安
        /// @return 是否成功
        bool setCurrent(double current_ma) override;

        bool configure(const MotorConfig &config) override;

        MotorConfig getConfig() const override;

        bool saveConfig() override;

        void onCANFrameReceived(const CANFrame &frame) override;

        std::vector<CANFrame> onStatusPoll() override;

        // ========== EYOUMotor 特有接口 ==========

        EYOUMotorState getEYOUState() const;

        std::unique_ptr<EYOUProfilePositionCmd> makeProfilePositionCmd(
            double pos, double vel, double torque, double acc, double dec);

        static std::unique_ptr<EYOUVelocityCmd> makeVelocityCmd(double vel, double max_current);

        std::unique_ptr<EYOUTorqueCmd> makeTorqueCmd(double torque_nm);

        static std::shared_ptr<EYOUMotor> from(std::shared_ptr<Motor> motor);

        static std::shared_ptr<EYOUMotor> cast(std::shared_ptr<Motor> motor);

        static bool is(std::shared_ptr<Motor> motor);

    private:
        EYOUMotorState eyou_state_;
        mutable std::mutex eyou_mutex_;
        std::vector<CANFrame> pending_frames_;
        mutable std::mutex pending_mutex_;
        EYOUMotorSpec spec_;

        void enqueueFrames(std::vector<CANFrame> frames);

        void updateEYOUState(const EYOUMotorState &state);

        bool decodeFrame(const CANFrame &frame, MotorState &state, EYOUMotorState &eyou);
    };
}
