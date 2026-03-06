/**
 * @Author: ZZSZSZSZZ
 * @CreateTime: 2026-02-23
 * @Description: 
 * @Version: 1.0
 */

#include "motor/motor_manager.hpp"
#include "can/can_coding.hpp"
#include <mutex>

namespace robot::motor {
    // MotorCANDevice
    MotorCANDevice::MotorCANDevice(const std::shared_ptr<Motor> &motor) : motor_(motor) {
    }

    void MotorCANDevice::onFrameReceived(const can::CANFrame &frame) {
        // 使用can_coding验证帧格式
        auto req = getFrameRequirement();
        if (!can::CANFrameDecoder::validate(frame, req)) {
            // 帧格式不符合要求，记录日志并忽略
            return;
        }

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
        can::CANDeviceFrameRequirement req{};
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
        state_only_polling_.store(options.skip_poll_if_disabled, std::memory_order_relaxed);
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
        // 收集所有电机ID
        std::vector<uint32_t> all_ids;
        {
            std::shared_lock lock(motors_mutex_);
            all_ids.reserve(motors_.size());
            for (const auto &[id, _]: motors_) {
                all_ids.push_back(id);
            }
        }
        return enableMotors(all_ids);
    }

    bool MotorManager::disableAll() {
        // 收集所有电机ID
        std::vector<uint32_t> all_ids;
        {
            std::shared_lock lock(motors_mutex_);
            all_ids.reserve(motors_.size());
            for (const auto &[id, _]: motors_) {
                all_ids.push_back(id);
            }
        }
        return disableMotors(all_ids);
    }

    bool MotorManager::enableMotors(const std::vector<uint32_t>& motor_ids, uint32_t timeout_ms) {
        // 获取电机对象并发送使能命令
        std::vector<std::shared_ptr<Motor>> motors;
        {
            std::shared_lock lock(motors_mutex_);
            for (uint32_t id: motor_ids) {
                auto it = motors_.find(id);
                if (it != motors_.end()) {
                    motors.push_back(it->second);
                }
            }
        }

        bool all_ok = true;
        for (auto& motor: motors) {
            if (!motor->enable()) all_ok = false;
        }

        // 如果启用了跳过未使能电机查询，等待所有电机使能完成
        if (options_.skip_poll_if_disabled) {
            state_only_polling_.store(true, std::memory_order_relaxed);

            constexpr uint32_t check_interval_ms = 10;
            uint32_t waited_ms = 0;
            bool all_enabled = false;
            while (waited_ms < timeout_ms) {
                {
                    std::shared_lock lock(motors_mutex_);
                    all_enabled = true;
                    for (uint32_t id: motor_ids) {
                        auto it = motors_.find(id);
                        if (it == motors_.end() || it->second->getState().state_machine != MotorStateMachine::Enabled) {
                            all_enabled = false;
                            break;
                        }
                    }
                    if (all_enabled) break;
                }
                std::this_thread::sleep_for(std::chrono::milliseconds(check_interval_ms));
                waited_ms += check_interval_ms;
            }
            if (!all_enabled) {
                return false;
            }

            state_only_polling_.store(false, std::memory_order_relaxed);
        }

        return all_ok;
    }

    bool MotorManager::disableMotors(const std::vector<uint32_t>& motor_ids, uint32_t timeout_ms) {
        // 获取电机对象并发送失能命令
        std::vector<std::shared_ptr<Motor>> motors;
        {
            std::shared_lock lock(motors_mutex_);
            for (uint32_t id: motor_ids) {
                auto it = motors_.find(id);
                if (it != motors_.end()) {
                    motors.push_back(it->second);
                }
            }
        }

        bool all_ok = true;
        for (auto& motor: motors) {
            if (!motor->disable()) all_ok = false;
        }

        // 如果启用了跳过未使能电机查询，等待所有电机失能完成
        if (options_.skip_poll_if_disabled) {
            state_only_polling_.store(true, std::memory_order_relaxed);

            constexpr uint32_t check_interval_ms = 10;
            uint32_t waited_ms = 0;
            bool all_disabled = false;
            while (waited_ms < timeout_ms) {
                {
                    std::shared_lock lock(motors_mutex_);
                    all_disabled = true;
                    for (uint32_t id: motor_ids) {
                        auto it = motors_.find(id);
                        if (it == motors_.end()) continue;
                        const auto state = it->second->getState().state_machine;
                        if (state != MotorStateMachine::Disabled && state != MotorStateMachine::Fault) {
                            all_disabled = false;
                            break;
                        }
                    }
                    if (all_disabled) break;
                }
                std::this_thread::sleep_for(std::chrono::milliseconds(check_interval_ms));
                waited_ms += check_interval_ms;
            }
            if (!all_disabled) {
                return false;
            }

            state_only_polling_.store(false, std::memory_order_relaxed);
        }

        return all_ok;
    }

    bool MotorManager::emergencyStopAll() {
        std::shared_lock lock(motors_mutex_);
        bool all_ok = true;
        for (auto &[id, motor]: motors_) {
            if (!motor->emergencyStop()) all_ok = false;
        }

        if (socket_) {
            pollOnce();
        }

        return all_ok;
    }

    bool MotorManager::clearAllFaults() {
        std::shared_lock lock(motors_mutex_);
        bool all_ok = true;
        for (auto &[id, motor]: motors_) {
            if (!motor->clearFault()) all_ok = false;
        }

        if (socket_) {
            pollOnce();
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
            // info.enabled = motor->getState().enabled;
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

        std::vector<can::CANFrame> all_frames;
        
        // 判断是否只查询状态
        const bool state_only = state_only_polling_.load(std::memory_order_relaxed);
        
        for (auto &[id, motor]: motors_) {
            const auto state = motor->getState().state_machine;
            
            // 如果启用了跳过未使能电机查询，跳过 Disabled 和 Fault 状态的电机
            if (options_.skip_poll_if_disabled) {
                if (state == MotorStateMachine::Disabled || state == MotorStateMachine::Fault) {
                    continue;
                }
            }
            
            // 然后根据模式添加状态查询帧
            if (state_only) {
                // 只查询状态（使能/故障）
                auto frames = motor->onStateOnlyPoll();
                all_frames.insert(all_frames.end(), frames.begin(), frames.end());
            } else {
                // 正常查询所有数据
                auto frames = motor->onStatusPoll();
                all_frames.insert(all_frames.end(), frames.begin(), frames.end());
            }
        }

        for (const auto &frame: all_frames) {
            if (!socket_) break;

            // 重试机制处理 EAGAIN
            int retries = 3;
            while (retries-- > 0) {
                auto result = socket_->writeFrame(frame);
                if (result.isSuccess()) {
                    break; // 发送成功
                }

                auto ec = result.error();
                if (ec.category == common::ErrorCategory::Transport &&
                    ec.sub_code == EAGAIN) {
                    // 缓冲区满，短暂等待后重试
                    std::this_thread::sleep_for(std::chrono::microseconds(100));
                    continue;
                }

                // 其他错误，记录并放弃
                Logger::warn("CAN write failed: " + ec.toString());
                break;
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
