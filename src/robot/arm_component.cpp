/**
 * @Author: ZZSZSZSZZ
 * @CreateTime: 2026-03-05
 * @Description: 
 * @Version: 1.0
 */

#include <utility>

#include "robot/arm_component.hpp"

namespace robot {
    ArmComponent::ArmComponent(const std::shared_ptr<motor::MotorManager> &manager, std::string name)
        : name_(std::move(name)), manager_(manager) {
    }

    void ArmComponent::initialize(const std::vector<motor::MotorConfig> &motor_configs) {
        for (const auto &config: motor_configs) {
            auto motor = motor::MotorFactory::create(config);
            manager_->addMotor(motor);
            motors_.push_back(motor);
        }

        manager_->start();

        Logger::info("Initializing " + name_ + " Component");
    }

    void ArmComponent::shutdown() {
        motors_.clear();
    }

    bool ArmComponent::enableAll() const {
        std::vector<uint32_t> motor_ids;
        motor_ids.reserve(motors_.size()); // 预分配空间提高效率
        for (const auto &motor_ptr: motors_) {
            if (motor_ptr) {
                // 可选：检查指针是否为空
                motor_ids.push_back(motor_ptr->id());
            }
        }

        return manager_->enableMotors(motor_ids);
    }

    bool ArmComponent::disableAll() const {
        std::vector<uint32_t> motor_ids;
        motor_ids.reserve(motors_.size()); // 预分配空间提高效率
        for (const auto &motor_ptr: motors_) {
            if (motor_ptr) {
                // 可选：检查指针是否为空
                motor_ids.push_back(motor_ptr->id());
            }
        }

        return manager_->disableMotors(motor_ids);
    }

    void ArmComponent::setPositions(const std::vector<double> &positions,
                                    const std::vector<double> &velocity,
                                    const std::vector<double> &torque) const {
        if (positions.size() != motors_.size() ||
            velocity.size() != motors_.size() ||
            torque.size() != motors_.size()) {
            throw common::ApplicationException(
                common::ApplicationError::InvalidConfiguration, "Position count mismatch", name_);
        }

        for (size_t i = 0; i < motors_.size(); ++i) {
            motors_[i]->setPosition(positions[i], velocity[i], torque[i]);
        }
    }

    void ArmComponent::setMITPositions(const std::vector<double> &positions,
                                       const std::vector<double> &velocity,
                                       const std::vector<double> &torque,
                                       const std::vector<double> &kp,
                                       const std::vector<double> &kd) {
        if (positions.size() != motors_.size()) {
            throw common::ApplicationException(
                common::ApplicationError::InvalidConfiguration, "Position count mismatch", name_);
        }

        for (size_t i = 0; i < motors_.size(); ++i) {
            double c_pos = motors_[i]->getState().position;
            double c_vel = motors_[i]->getState().velocity;
            double mit_t = torque[i] + kp[i] * (positions[i] - c_pos) + kd[i] * (velocity[i] - c_vel);
            motors_[i]->setTorque(mit_t);
        }
    }
}
