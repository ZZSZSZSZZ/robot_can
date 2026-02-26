/**
 * @Author: ZZSZSZSZZ
 * @CreateTime: 2026-02-23
 * @Description: 
 * @Version: 1.0
 */

#pragma once

#include "eyou_motor.hpp"

namespace robot::motor::eyou {
    class EYOUFactory {
    public:
        static void registerMotorType();

        // 通过配置创建（type 决定型号）
        static std::shared_ptr<EYOUMotor> createMotor(const MotorConfig& config);

        // 快速创建
        static std::shared_ptr<EYOUMotor> createMotor(uint32_t id, const std::string& type = "EYOU_PP08");
    };
}
