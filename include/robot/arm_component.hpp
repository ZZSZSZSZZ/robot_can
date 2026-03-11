/**
 * @Author: ZZSZSZSZZ
 * @CreateTime: 2026-03-02
 * @Description: 
 * @Version: 1.0
 */

#pragma once

#include "base_component.hpp"
#include "motor/motor_factory.hpp"
#include "motor/motor_manager.hpp"
#include "motor/drivers/eyou/eyou_factory.hpp"

namespace robot {
    struct PositionParam {
        double position;
        double velocity;
        double torque;
        double acc;
    };

    class ArmComponent : public BaseComponent {
    public:
        ArmComponent(const std::shared_ptr<motor::MotorManager> &manager,
                     std::string name = "Arm");

        void initialize(const std::vector<motor::MotorConfig> &motor_configs) override;

        void shutdown() override;

        bool enableAll() const override;

        bool disableAll() const override;

        void setPositions(const std::vector<robot::PositionParam> &param) const;

        void setMITPositions(const std::vector<double> &positions,
                             const std::vector<double> &velocity,
                             const std::vector<double> &torque,
                             const std::vector<double> &kp,
                             const std::vector<double> &kd) const;

        std::string getName() const override { return name_; }

        std::shared_ptr<motor::Motor> getMotor(const size_t index) {
            return (index < motors_.size()) ? motors_[index] : nullptr;
        }

        std::vector<motor::MotorState> getStates() const {
            std::vector<motor::MotorState> states;
            for (auto &motor: motors_) states.push_back(motor->getState());
            return states;
        }

        size_t getMotorCount() const { return motors_.size(); }

    private:
        std::string name_;
        std::shared_ptr<motor::MotorManager> manager_;
        std::vector<std::shared_ptr<motor::Motor> > motors_;
    };
}
