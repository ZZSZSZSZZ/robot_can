/**
 * @Author: ZZSZSZSZZ
 * @CreateTime: 2026-02-26
 * @Description: 
 * @Version: 1.0
 */

#include "motor/drivers/eyou/eyou_specification.hpp"

namespace robot::motor::eyou {
    // 规格注册表实现
    const std::unordered_map<std::string, const EYOUMotorSpec *> &EYOUSpecRegistry::getMap() {
        static const std::unordered_map<std::string, const EYOUMotorSpec *> map = {
            {Specs::EYOU_PP08.type, &Specs::EYOU_PP08},
            {Specs::EYOU_PP11.type, &Specs::EYOU_PP11},
            {Specs::EYOU_PP11L.type, &Specs::EYOU_PP11L}
        };
        return map;
    }

    const EYOUMotorSpec *EYOUSpecRegistry::find(const std::string &driver_type) {
        const auto &map = getMap();
        auto it = map.find(driver_type);
        return it != map.end() ? it->second : nullptr;
    }

    std::vector<std::string> EYOUSpecRegistry::getAllDriverTypes() {
        std::vector<std::string> types;
        for (const auto &[type, _]: getMap()) {
            types.push_back(type);
        }
        return types;
    }
}
