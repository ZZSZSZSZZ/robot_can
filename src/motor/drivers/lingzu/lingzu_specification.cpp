/**
 * @Author: ZZSZSZSZZ
 * @CreateTime: 2026-03-12
 * @Description: 灵足电机规格注册表实现
 * @Version: 1.0
 */

#include "motor/drivers/lingzu/lingzu_specification.hpp"

namespace robot::motor::lingzu {
    const std::unordered_map<std::string, const LingzuMotorSpec*>& LingzuSpecRegistry::getMap() {
        static const std::unordered_map<std::string, const LingzuMotorSpec*> map = {
            {Specs::LINGZU_RS05.type, &Specs::LINGZU_RS05},
        };
        return map;
    }

    const LingzuMotorSpec* LingzuSpecRegistry::find(const std::string& type) {
        const auto& map = getMap();
        auto it = map.find(type);
        return it != map.end() ? it->second : nullptr;
    }

    std::vector<std::string> LingzuSpecRegistry::getAllDriverTypes() {
        std::vector<std::string> types;
        for (const auto& [type, _] : getMap()) {
            types.push_back(type);
        }
        return types;
    }
} // namespace robot::motor::lingzu
