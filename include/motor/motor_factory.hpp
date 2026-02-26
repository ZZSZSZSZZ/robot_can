/**
 * @Author: ZZSZSZSZZ
 * @CreateTime: 2026-02-26
 * @Description: 
 * @Version: 1.0
 */

#pragma once

#include "motor_interface.hpp"

namespace robot::motor {
    class MotorFactory {
    public:
        using Creator = std::function<std::shared_ptr<Motor>(const MotorConfig &)>;

        static void registerType(const std::string &type, Creator creator);

        static std::shared_ptr<Motor> create(const MotorConfig &cfg);

        static bool isRegistered(const std::string &type);

        static std::vector<std::string> getRegisteredTypes();

    private:
        static std::unordered_map<std::string, Creator> &registry();
    };
}
