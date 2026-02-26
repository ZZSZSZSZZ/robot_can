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
        // 通过 driver_type 自动查找规格
        explicit EYOUMotor(const MotorConfig &config);

        // 或直接指定规格
        explicit EYOUMotor(const MotorConfig &config, const EYOUMotorSpec &spec);

        const std::string &type() const override;

        MotorCapability capabilities() const override;

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

        bool configure(const MotorConfig &config) override;

        MotorConfig getConfig() const override;

        bool saveConfig() override;

        void onCANFrameReceived(const CANFrame &frame) override;

        std::vector<CANFrame> onStatusPoll() override;

        // ========== EYOUMotor 特有接口 ==========
        EYOUMotorState getEYOUState() const;

        static std::unique_ptr<EYOUProfilePositionCmd> makeProfilePositionCmd(
            double pos, double vel, double acc, double dec, double torque);

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
