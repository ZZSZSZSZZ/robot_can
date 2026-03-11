/**
 * @Author: ZZSZSZSZZ
 * @CreateTime: 2026-03-09
 * @Description: Robot 主类 - 管理机器人的所有组件和通信
 * @Version: 1.0
 */

#pragma once

#include <atomic>
#include <functional>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "common/common.hpp"
#include "can/can_socket.hpp"
#include "can/can_frame_router.hpp"
#include "can/can_receiver.hpp"
#include "motor/motor_manager.hpp"
#include "motor/motor_type.hpp"
#include "robot/arm_component.hpp"

namespace robot {
    /**
     * @brief Robot 主类
     *
     * 参考 openarm::can::socket::OpenArm 设计
     * 统一管理 CAN 通信、电机管理、手臂组件等
     */
    class Robot {
    public:
        /**
         * @brief Robot 配置选项
         */
        struct Options {
            // CAN 配置
            bool enable_can_fd; // 是否启用 CAN FD
            int receive_thread_priority; // 接收线程实时优先级

            // 电机管理器配置
            uint32_t status_poll_interval_ms; // 状态轮询间隔
            bool enable_background_thread; // 启用后台轮询线程
            bool skip_poll_if_disabled; // 未使能时跳过查询

            Options()
                : enable_can_fd(false),
                  receive_thread_priority(50),
                  status_poll_interval_ms(10),
                  enable_background_thread(true),
                  skip_poll_if_disabled(true) {
            }
        };

        /**
         * @brief 构造函数（单 CAN 接口模式）
         * @param can_interface CAN 接口名称（如 "can0"）
         * @param options 配置选项
         */
        Robot(const std::string &can_interface, const Options &options = Options());

        /**
         * @brief 构造函数（多 CAN 接口模式）
         * @param can_interfaces CAN 接口名称列表
         * @param options 配置选项
         */
        Robot(const std::vector<std::string> &can_interfaces, const Options &options = Options());

        ~Robot();

        // 禁止拷贝和赋值
        Robot(const Robot &) = delete;

        Robot &operator=(const Robot &) = delete;

        // ==================== 初始化 ====================

        /**
         * @brief 初始化 Robot
         * @return Result<void> 初始化结果
         */
        Result<void> initialize();

        /**
         * @brief 关闭 Robot，释放资源
         */
        void shutdown();

        /**
         * @brief 检查是否已初始化
         */
        bool isInitialized() const { return initialized_; }

        // ==================== 手臂组件管理 ====================

        /**
         * @brief 创建并初始化手臂组件
         * @param name 组件名称
         * @param motor_configs 电机配置列表
         * @return 创建的手臂组件
         */
        std::shared_ptr<ArmComponent> createArmComponent(
            const std::string &name,
            const std::vector<motor::MotorConfig> &motor_configs);

        /**
         * @brief 获取手臂组件
         * @param name 组件名称
         * @return 手臂组件，未找到返回 nullptr
         */
        std::shared_ptr<ArmComponent> getArmComponent(const std::string &name) const;

        /**
         * @brief 移除手臂组件
         * @param name 组件名称
         */
        void removeArmComponent(const std::string &name);

        /**
         * @brief 获取所有手臂组件名称
         */
        std::vector<std::string> getArmComponentNames() const;

        // ==================== 电机管理（便捷接口） ====================

        /**
         * @brief 使能所有电机的电源
         * @return 是否全部成功
         */
        bool enableAllMotors();

        /**
         * @brief 失能所有电机的电源
         * @return 是否全部成功
         */
        bool disableAllMotors();

        /**
         * @brief 紧急停止所有电机
         * @return 是否成功
         */
        bool emergencyStopAll();

        /**
         * @brief 清除所有电机故障
         * @return 是否成功
         */
        bool clearAllFaults();


        // ==================== 状态监控 ====================

        /**
         * @brief 设置全局状态回调
         * @param callback 回调函数
         */
        void setGlobalStateCallback(std::function<void(uint32_t motor_id, const motor::MotorState &state)> callback);

        /**
         * @brief 获取所有电机信息
         */
        std::vector<motor::MotorInfo> getAllMotorInfos() const;

        /**
         * @brief 获取指定电机状态
         * @param motor_id 电机 ID
         * @return 电机状态
         */
        motor::MotorState getMotorState(uint32_t motor_id) const;

        /**
         * @brief 获取所有电机状态
         * @return 电机ID到电机状态的映射表
         */
        std::unordered_map<uint32_t, motor::MotorState> getAllMotorState() const;

        // ==================== 底层访问（高级用法） ====================

        /**
         * @brief 获取电机管理器
         */
        std::shared_ptr<motor::MotorManager> getMotorManager() const { return motor_manager_; }

        /**
         * @brief 获取 CAN 帧路由器
         */
        std::shared_ptr<can::CANFrameRouter> getRouter() const { return router_; }

        /**
         * @brief 获取 CAN 接收器
         */
        std::shared_ptr<can::CANReceiver> getReceiver() const { return receiver_; }

        /**
         * @brief 获取 CAN Socket
         * @param interface_name 接口名称（单接口模式可留空）
         */
        std::shared_ptr<can::CANSocket> getCANSocket(const std::string &interface_name = "") const;

    private:
        // 配置
        std::vector<std::string> can_interfaces_;
        Options options_;
        bool multi_interface_mode_ = false;

        // 核心组件
        std::shared_ptr<can::CANFrameRouter> router_;
        std::shared_ptr<can::CANReceiver> receiver_;
        std::shared_ptr<motor::MotorManager> motor_manager_;

        // CAN Sockets（多接口模式）
        std::unordered_map<std::string, std::shared_ptr<can::CANSocket> > sockets_;

        // 单接口模式下的默认 socket
        std::shared_ptr<can::CANSocket> default_socket_;

        // 手臂组件
        std::unordered_map<std::string, std::shared_ptr<ArmComponent> > arm_components_;

        // 状态
        std::atomic<bool> initialized_{false};

        // 初始化步骤
        Result<void> initializeCANSockets();

        Result<void> initializeRouter();

        Result<void> initializeReceiver();

        Result<void> initializeMotorManager();
    };
} // namespace robot
