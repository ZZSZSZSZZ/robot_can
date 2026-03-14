/**
 * @Author: ZZSZSZSZZ
 * @CreateTime: 2026-03-12
 * @Description: 灵足电机工厂类
 * @Version: 1.0
 */

#pragma once

#include "lingzu_motor.hpp"

namespace robot::motor::lingzu {
    class LingzuFactory {
    public:
        /// 注册灵足电机类型到工厂
        static void registerMotorType();

        /// 通过配置创建电机
        static std::shared_ptr<LingzuMotor> createMotor(const MotorConfig& config);

        /// 快速创建电机
        /// @param id 电机CAN ID
        /// @param type 电机型号 (默认 LINGZU_MF8010)
        static std::shared_ptr<LingzuMotor> createMotor(uint32_t id, const std::string& type = "LINGZU_MF8010");
    };
} // namespace robot::motor::lingzu
