/**
 * @Author: ZZSZSZSZZ
 * @CreateTime: 2026-02-24
 * @Description: 
 * @Version: 1.0
 */

#include "motor/drivers/eyou/eyou_factory.hpp"
#include "motor/motor_factory.hpp"

namespace robot::motor::eyou {
    void EYOUFactory::registerMotorType() {
        auto registerSpec = [](const EYOUMotorSpec *spec) {
            MotorFactory::registerType(spec->type, [spec](const MotorConfig &cfg) {
                MotorConfig new_cfg = cfg;
                new_cfg.type = spec->type; // 确保一致
                return std::static_pointer_cast<Motor>(std::make_shared<EYOUMotor>(new_cfg, *spec));
            });
        };

        registerSpec(&Specs::EYOU_PP08);
        registerSpec(&Specs::EYOU_PP11);
        registerSpec(&Specs::EYOU_PP11L);
    }

    std::shared_ptr<EYOUMotor> EYOUFactory::createMotor(const MotorConfig &config) {
        return std::make_shared<EYOUMotor>(config);
    }

    std::shared_ptr<EYOUMotor> EYOUFactory::createMotor(uint32_t id, const std::string &type) {
        MotorConfig cfg;
        cfg.id = id;
        cfg.name = "EYOU_" + std::to_string(id);
        cfg.type = type;
        cfg.tx_can_id = id;
        cfg.rx_can_id = id;
        cfg.useStandardFrame(8);
        return createMotor(cfg);
    }
}
