/**
 * @Author: ZZSZSZSZZ
 * @CreateTime: 2026-03-06
 * @Description: 电机接口定义
 * @Version: 2.0
 */

#pragma once

#include <functional>
#include <string>
#include <vector>

#include "motor_type.hpp"
#include "can/can_frame.hpp"

namespace robot::motor {
    // 前向声明
    struct MotorState;
    struct MotorConfig;

    /**
     * @brief 电机命令基类
     */
    class MotorCommand {
    public:
        virtual ~MotorCommand() = default;
    };

    /**
     * @brief 电机接口 - 定义所有电机必须实现的核心操作
     */
    class Motor {
    public:
        virtual ~Motor() = default;

        // ========== 基本信息 ==========

        /// 电机 ID
        /// @return 电机 ID
        virtual uint32_t id() const = 0;

        /// 电机名称
        /// @return 电机名称字符串
        virtual const std::string &name() const = 0;

        /// 获取电机型号
        /// @return 电机型号字符串
        virtual const std::string &type() const = 0;

        // ========== 状态管理 ==========

        /// 获取电机状态
        /// @return 电机状态
        virtual MotorState getState() const = 0;

        /// 等待状态更新
        /// @param state 输出参数，接收更新后的状态
        /// @param timeout_ms 超时时间(毫秒)
        /// @return 是否在超时前收到更新
        virtual bool waitForStateUpdate(MotorState &state, uint32_t timeout_ms) = 0;

        /// 获取故障码
        /// @return 故障码
        virtual uint32_t getFaultCode() const { return 0; }

        /// 获取故障描述
        /// @return 故障描述列表
        virtual std::vector<std::string> getFaultDescriptions() const { return {}; }

        // ========== 基础控制 ==========

        /// 使能电机
        /// @return 是否成功
        virtual bool enable() = 0;

        /// 失能电机
        /// @return 是否成功
        virtual bool disable() = 0;

        /// 紧急停止
        /// @return 是否成功
        virtual bool emergencyStop() = 0;

        /// 清除故障
        /// @return 是否成功
        virtual bool clearFault() = 0;

        /// 设置零位
        /// @return 是否成功
        virtual bool setZeroPosition() = 0;

        // ========== 运动控制 ==========

        /// 执行自定义命令
        /// @param cmd 控制命令
        /// @return 是否成功
        virtual bool command(const MotorCommand &cmd) = 0;

        /// 位置控制
        /// @param position_rad 目标位置(弧度)
        /// @param max_vel 最大速度限制
        /// @param max_torque 最大扭矩限制
        /// @return 是否成功
        virtual bool setPosition(double position_rad, double max_vel, double max_torque) = 0;

        /// 速度控制
        /// @param velocity_rad_s 目标速度(弧度/秒)
        /// @param max_current 最大电流限制
        /// @return 是否成功
        virtual bool setVelocity(double velocity_rad_s, double max_current) = 0;

        /// 力矩控制
        /// @param torque_nm 目标力矩(牛米)
        /// @return 是否成功
        virtual bool setTorque(double torque_nm) = 0;

        /// 电流控制
        /// @param current_a 目标电流(安培)
        /// @return 是否成功
        virtual bool setCurrent(double current_a) = 0;

        // ========== 配置管理 ==========

        /// 配置电机参数
        /// @param config 电机配置
        /// @return 是否成功
        virtual bool configure(const MotorConfig &config) = 0;

        /// 获取电机配置
        /// @return 当前配置
        virtual MotorConfig getConfig() const = 0;

        // ========== 回调设置 ==========

        /// 设置状态更新回调
        /// @param cb 回调函数
        virtual void setStateCallback(std::function<void(const MotorState &)> cb) = 0;

        /// 状态轮询回调（用于主动查询型电机）
        /// @return 轮询帧列表（非CAN电机返回空）
        virtual std::vector<can::CANFrame> onStatusPoll() { return {}; }

        /// 只查询状态（使能状态、故障状态）
        /// @return 状态查询帧列表（非CAN电机返回空）
        virtual std::vector<can::CANFrame> onStateOnlyPoll() { return {}; }

        /// 接收CAN帧回调（由MotorManager调用，仅CAN电机需要实现）
        /// @param frame 接收到的CAN帧
        virtual void onCANFrameReceived(const can::CANFrame &frame) {}
    };
}
