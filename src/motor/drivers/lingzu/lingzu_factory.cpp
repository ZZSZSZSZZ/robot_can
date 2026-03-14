/**
 * @Author: ZZSZSZSZZ
 * @CreateTime: 2026-03-12
 * @Description: 灵足电机工厂实现
 * @Version: 1.0
 */

#include "motor/drivers/lingzu/lingzu_factory.hpp"
#include "motor/motor_factory.hpp"

namespace robot::motor::lingzu {

    void LingzuFactory::registerMotorType() {
        auto registerSpec = [](const LingzuMotorSpec* spec) {
            MotorFactory::registerType(spec->type, [spec](const MotorConfig& cfg) {
                MotorConfig new_cfg = cfg;
                new_cfg.type = spec->type;  // 确保一致
                // 灵足电机使用扩展帧
                new_cfg.useExtendedFrame(8);
                return std::static_pointer_cast<Motor>(std::make_shared<LingzuMotor>(new_cfg, *spec));
            });
        };

        registerSpec(&Specs::LINGZU_RS05);
    }

    std::shared_ptr<LingzuMotor> LingzuFactory::createMotor(const MotorConfig& config) {
        return std::make_shared<LingzuMotor>(config);
    }

    std::shared_ptr<LingzuMotor> LingzuFactory::createMotor(uint32_t id, const std::string& type) {
        MotorConfig cfg;
        cfg.id = id;
        cfg.name = "LINGZU_" + std::to_string(id);
        cfg.type = type;
        cfg.tx_can_id = id;
        cfg.rx_can_id = id;
        // 灵足电机使用扩展帧
        cfg.useExtendedFrame(8);
        return createMotor(cfg);
    }

} // namespace robot::motor::lingzu
