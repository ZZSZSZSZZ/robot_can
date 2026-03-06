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
    class ArmComponent : public BaseComponent {
    public:
        ArmComponent(const std::shared_ptr<motor::MotorManager> &manager,
                     std::string name = "Arm");

        void initialize(const std::vector<motor::MotorConfig> &motor_configs) override;

        void shutdown() override;

        std::string getName() const override { return name_; }

        // void enableAll() const { manager_->enableAll(); }
        // void disableAll() const { manager_->disableAll(); }

        void setPositions(const std::vector<double> &positions,
                          const std::vector<double> &velocity,
                          const std::vector<double> &torque) const;

        void setMITPositions(const std::vector<double> &positions,
                             const std::vector<double> &velocity,
                             const std::vector<double> &torque,
                             const std::vector<double> &kp,
                             const std::vector<double> &kd);

        std::vector<motor::MotorState> getStates() const {
            std::vector<motor::MotorState> states;
            for (auto &motor: motors_) states.push_back(motor->getState());
            return states;
        }

        std::shared_ptr<motor::Motor> getMotor(const size_t index) {
            return (index < motors_.size()) ? motors_[index] : nullptr;
        }

        size_t getMotorCount() const { return motors_.size(); }

    private:
        std::string name_;
        std::shared_ptr<motor::MotorManager> manager_;
        std::vector<std::shared_ptr<motor::Motor> > motors_;
    };
}
