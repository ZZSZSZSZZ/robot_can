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
#include "drivers/eyou/eyou_command.hpp"

namespace robot::motor {
    // 电机-CAN桥接设备
    class MotorCANDevice : public can::CANDevice {
    public:
        explicit MotorCANDevice(const std::shared_ptr<Motor> &motor);

        // CANDevice 实现
        void onFrameReceived(const eyou::CANFrame &frame) override;

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
            bool skip_poll_if_disabled; // 未使能时跳过查询

            Options()
                : status_poll_interval_ms(10),
                  enable_background_thread(true),
                  skip_poll_if_disabled(true) {
            }
        };

        /// 构造函数（多接口模式）
        /// @param router CAN帧路由器
        /// @param options 选项
        explicit MotorManager(const std::shared_ptr<can::CANFrameRouter> &router, const Options &options = Options());

        /// 构造函数（单接口模式，向后兼容）
        /// @param socket CAN套接字
        /// @param router CAN帧路由器
        /// @param options 选项
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

        // ==================== 指定电机操作（带状态确认） ====================

        /// 使能指定电机并等待状态确认
        /// @param motor_ids 电机ID列表
        /// @param timeout_ms 超时时间（毫秒，默认3000ms）
        /// @return 是否成功使能
        bool enableMotors(const std::vector<uint32_t> &motor_ids, uint32_t timeout_ms = 3000);

        /// 失能指定电机并等待状态确认
        /// @param motor_ids 电机ID列表
        /// @param timeout_ms 超时时间（毫秒，默认3000ms）
        /// @return 是否成功失能
        bool disableMotors(const std::vector<uint32_t> &motor_ids, uint32_t timeout_ms = 3000);

        // ==================== 状态监控 ====================

        std::vector<MotorInfo> getMotorInfos() const;

        void setGlobalStateCallback(const std::function<void(uint32_t, const MotorState &)> &cb);

        // ==================== 生命周期 ====================

        bool start(); // 启动轮询线程（receiver 由外部启动）
        void stop(); // 停止轮询线程
        bool isRunning() const;

        // 手动轮询一次（用于非线程模式或调试）
        // 当 state_only_polling_ 为 true 时，只查询状态
        void pollOnce();

        // ==================== 多CAN接口管理 ====================

        /// 添加CAN接口
        /// @param name 接口名称（如 "can0", "can1"）
        /// @param socket CAN套接字
        /// @return 是否成功
        bool addInterface(const std::string &name, const std::shared_ptr<can::CANSocket> &socket);

        /// 移除CAN接口
        /// @param name 接口名称
        void removeInterface(const std::string &name);

        /// 获取指定接口的socket
        /// @param name 接口名称
        /// @return CAN套接字，未找到返回nullptr
        std::shared_ptr<can::CANSocket> getInterface(const std::string &name) const;

        /// 获取所有接口名称
        /// @return 接口名称列表
        std::vector<std::string> getInterfaceNames() const;

        /// 获取接口数量
        /// @return 接口数量
        size_t getInterfaceCount() const;

    private:
        std::shared_ptr<can::CANSocket> socket_; // 单接口模式使用（向后兼容）
        std::shared_ptr<can::CANFrameRouter> router_;
        Options options_;

        // 多接口管理
        mutable std::shared_mutex sockets_mutex_;
        std::unordered_map<std::string, std::shared_ptr<can::CANSocket> > sockets_;

        std::unordered_map<uint32_t, std::shared_ptr<Motor> > motors_;
        mutable std::shared_mutex motors_mutex_;

        std::atomic<bool> running_{false};
        std::thread poll_thread_;

        std::function<void(uint32_t, const MotorState &)> global_state_cb_;

        // 标志：当前只查询状态（不查询位置/速度等数据）
        std::atomic<bool> state_only_polling_{};

        void pollLoop();
    };
}
