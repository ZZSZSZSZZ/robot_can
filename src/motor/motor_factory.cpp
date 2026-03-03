/**
 * @Author: ZZSZSZSZZ
 * @CreateTime: 2026-02-26
 * @Description: 
 * @Version: 1.0
 */

#include <utility>

#include "motor/motor_factory.hpp"

namespace robot::motor {
    std::unordered_map<std::string, MotorFactory::Creator> &MotorFactory::registry() {
        static std::unordered_map<std::string, Creator> reg;
        return reg;
    }

    void MotorFactory::registerType(const std::string &type, Creator creator) {
        registry()[type] = std::move(creator);
    }

    std::shared_ptr<Motor> MotorFactory::create(const MotorConfig &cfg) {
        const auto it = registry().find(cfg.type);
        return it != registry().end() ? it->second(cfg) : nullptr;
    }

    bool MotorFactory::isRegistered(const std::string &type) {
        return registry().count(type) > 0;
    }

    std::vector<std::string> MotorFactory::getRegisteredTypes() {
        std::vector<std::string> types;
        for (const auto &[name, _]: registry()) types.push_back(name);
        return types;
    }
}
