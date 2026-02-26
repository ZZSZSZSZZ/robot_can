/**
 * @Author: ZZSZSZSZZ
 * @CreateTime: 2026-02-22
 * @Description: 
 * @Version: 1.0
 */

#pragma once

#include <atomic>
#include <shared_mutex>
#include <thread>

#include "can/can_device.hpp"
#include "can/can_frame_router.hpp"
#include "can/can_socket.hpp"
#include "motor_factory.hpp"
#include "protocol_interface.hpp"

namespace robot::motor {
    // 电机-CAN桥接设备
    class MotorCANDevice : public can::CANDevice {
    public:
        explicit MotorCANDevice(const std::shared_ptr<Motor> &motor);

        // CANDevice 实现
        void onFrameReceived(const CANFrame &frame) override;

        uint32_t getReceiveId() const override;

        bool isExtendedId() const override;

        can::CANDeviceFrameRequirement getFrameRequirement() const override;

        std::string getName() const override { return motor_->name(); }

        std::shared_ptr<Motor> getMotor() const { return motor_; }

    private:
        std::shared_ptr<Motor> motor_;
    };

    // 电机管理器
    class MotorManager {
    public:
        struct Options {
            uint32_t status_poll_interval_ms;
            bool enable_background_thread;

            Options()
                : status_poll_interval_ms(10)
                  , enable_background_thread(true) {
            }
        };

        explicit MotorManager(const std::shared_ptr<can::CANSocket> &socket,
                              const std::shared_ptr<can::CANFrameRouter> &router, const Options &options = Options());

        ~MotorManager();

        MotorManager(const MotorManager &) = delete;

        MotorManager &operator=(const MotorManager &) = delete;

        // ==================== 电机管理 ====================

        bool addMotor(std::shared_ptr<Motor> motor);

        void removeMotor(uint32_t motor_id);

        std::shared_ptr<Motor> getMotor(uint32_t motor_id) const;

        std::vector<std::shared_ptr<Motor> > getAllMotors() const;

        bool hasMotor(uint32_t motor_id) const;

        // ==================== 批量操作 ====================

        bool enableAll();

        bool disableAll();

        bool emergencyStopAll();

        bool clearAllFaults();

        // ==================== 状态监控 ====================

        std::vector<MotorInfo> getMotorInfos() const;

        void setGlobalStateCallback(const std::function<void(uint32_t, const MotorState &)> &cb);

        // ==================== 生命周期 ====================

        bool start(); // 启动轮询线程（receiver 由外部启动）
        void stop(); // 停止轮询线程
        bool isRunning() const;

        // 手动轮询一次（用于非线程模式或调试）
        void pollOnce();

    private:
        std::shared_ptr<can::CANSocket> socket_;
        std::shared_ptr<can::CANFrameRouter> router_;
        Options options_;

        std::unordered_map<uint32_t, std::shared_ptr<Motor> > motors_;
        mutable std::shared_mutex motors_mutex_;

        std::atomic<bool> running_{false};
        std::thread poll_thread_;

        std::function<void(uint32_t, const MotorState &)> global_state_cb_;

        void pollLoop();
    };
}
