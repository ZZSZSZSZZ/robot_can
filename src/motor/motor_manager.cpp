/**
 * @Author: ZZSZSZSZZ
 * @CreateTime: 2026-02-23
 * @Description: 
 * @Version: 1.0
 */

#include "motor/motor_manager.hpp"
#include <mutex>

namespace robot::motor {
    // MotorCANDevice
    MotorCANDevice::MotorCANDevice(const std::shared_ptr<Motor> &motor) : motor_(motor) {
    }

    void MotorCANDevice::onFrameReceived(const CANFrame &frame) {
        motor_->onCANFrameReceived(frame);
    }

    uint32_t MotorCANDevice::getReceiveId() const {
        return motor_->getConfig().rx_can_id;
    }

    bool MotorCANDevice::isExtendedId() const {
        return motor_->getConfig().tx_format.isExtendedId;
    }

    can::CANDeviceFrameRequirement MotorCANDevice::getFrameRequirement() const {
        const auto &fmt = motor_->getConfig().rx_format;
        can::CANDeviceFrameRequirement req;
        req.preferredType = fmt.type;
        req.requireExtendedId = fmt.isExtendedId;
        req.maxDataLength = fmt.dlc;
        req.requireCanFd = fmt.type == can::CANFrameType::CanFd;
        return req;
    }

    MotorManager::MotorManager(const std::shared_ptr<can::CANSocket> &socket,
                               const std::shared_ptr<can::CANFrameRouter> &router,
                               const Options &options)
        : socket_(socket), router_(router), options_(options) {
    }

    MotorManager::~MotorManager() {
        stop();
    }

    bool MotorManager::addMotor(std::shared_ptr<Motor> motor) {
        if (!motor || !router_) return false;

        std::unique_lock lock(motors_mutex_);
        uint32_t id = motor->id();

        if (motors_.count(id)) {
            return false;
        }

        motor->setStateCallback([this, id](const MotorState &state) {
            if (global_state_cb_) {
                global_state_cb_(id, state);
            }
        });

        motors_[id] = motor;
        lock.unlock();

        // 注册到CAN路由器
        auto can_device = std::make_shared<MotorCANDevice>(motor);
        router_->registerDevice(can_device);

        return true;
    }

    void MotorManager::removeMotor(uint32_t motor_id) {
        std::shared_ptr<Motor> motor;
        {
            std::unique_lock lock(motors_mutex_);
            auto it = motors_.find(motor_id);
            if (it == motors_.end()) return;
            motor = it->second;
            motors_.erase(it);
        }

        if (router_ && motor) {
            router_->unregisterDevice(motor->getConfig().rx_can_id,
                                      motor->getConfig().rx_format.isExtendedId);
        }
    }

    std::shared_ptr<Motor> MotorManager::getMotor(uint32_t motor_id) const {
        std::shared_lock lock(motors_mutex_);
        auto it = motors_.find(motor_id);
        return it != motors_.end() ? it->second : nullptr;
    }

    std::vector<std::shared_ptr<Motor> > MotorManager::getAllMotors() const {
        std::shared_lock lock(motors_mutex_);
        std::vector<std::shared_ptr<Motor> > result;
        result.reserve(motors_.size());
        for (const auto &[id, motor]: motors_) {
            result.push_back(motor);
        }
        return result;
    }

    bool MotorManager::hasMotor(uint32_t motor_id) const {
        std::shared_lock lock(motors_mutex_);
        return motors_.count(motor_id) > 0;
    }

    bool MotorManager::enableAll() {
        std::shared_lock lock(motors_mutex_);
        bool all_ok = true;
        for (auto &[id, motor]: motors_) {
            if (!motor->enable()) all_ok = false;
        }
        return all_ok;
    }

    bool MotorManager::disableAll() {
        std::shared_lock lock(motors_mutex_);
        bool all_ok = true;
        for (auto &[id, motor]: motors_) {
            if (!motor->disable()) all_ok = false;
        }
        return all_ok;
    }

    bool MotorManager::emergencyStopAll() {
        std::shared_lock lock(motors_mutex_);
        bool all_ok = true;
        for (auto &[id, motor]: motors_) {
            if (!motor->emergencyStop()) all_ok = false;
        }
        return all_ok;
    }

    bool MotorManager::clearAllFaults() {
        std::shared_lock lock(motors_mutex_);
        bool all_ok = true;
        for (auto &[id, motor]: motors_) {
            if (!motor->clearFault()) all_ok = false;
        }
        return all_ok;
    }

    std::vector<MotorInfo> MotorManager::getMotorInfos() const {
        std::shared_lock lock(motors_mutex_);
        std::vector<MotorInfo> infos;
        infos.reserve(motors_.size());

        for (const auto &[id, motor]: motors_) {
            MotorInfo info;
            info.id = id;
            info.name = motor->name();
            info.driver_type = motor->type();
            info.capabilities = motor->capabilities();
            info.enabled = motor->getState().enabled;
            info.last_update = motor->getState().timestamp;
            infos.push_back(info);
        }
        return infos;
    }

    void MotorManager::setGlobalStateCallback(const std::function<void(uint32_t, const MotorState &)> &cb) {
        global_state_cb_ = cb;
    }

    bool MotorManager::start() {
        if (running_) return true;
        if (!options_.enable_background_thread) return true;
        try {
            running_ = true;
            poll_thread_ = std::thread(&MotorManager::pollLoop, this);
            return true;
        } catch (...) {
            running_ = false;
            return false;
        }
    }

    void MotorManager::stop() {
        running_ = false;
        if (poll_thread_.joinable()) {
            poll_thread_.join();
        }
    }

    bool MotorManager::isRunning() const {
        return running_;
    }

    void MotorManager::pollOnce() {
        std::shared_lock lock(motors_mutex_);

        for (auto &[id, motor]: motors_) {
            // 只处理需要主动查询的电机（如意优）
            if (motor->supports(MotorCapability::PollStatus)) {
                auto frames = motor->onStatusPoll();
                for (const auto &frame: frames) {
                    if (socket_) {
                        auto result = socket_->writeFrame(frame);
                        if (result.isError()) {
                            // 可以记录错误
                        }
                    }
                }
            }
        }
    }

    void MotorManager::pollLoop() {
        while (running_) {
            auto start = std::chrono::steady_clock::now();
            pollOnce();
            std::this_thread::sleep_until(start + std::chrono::milliseconds(options_.status_poll_interval_ms));
        }
    }
}
