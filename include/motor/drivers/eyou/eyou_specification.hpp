/**
 * @Author: ZZSZSZSZZ
 * @CreateTime: 2026-02-26
 * @Description: 
 * @Version: 1.0
 */

#pragma once

#include <string>
#include <unordered_map>
#include <vector>

namespace robot::motor::eyou {
    // 电机型号与额定参数配置
    struct EYOUMotorSpec {
        std::string type;
        double rated_torque_nm; // 额定扭矩 (Nm)
        double rated_current_ma = 1200.0; // 额定电流 (mA)

        std::string getBrand() const {
            std::string dt(type);
            size_t pos = dt.find('_');
            return pos != std::string::npos ? dt.substr(0, pos) : dt;
        }

        std::string getModel() const {
            std::string dt(type);
            size_t pos = dt.find('_');
            return pos != std::string::npos ? dt.substr(pos + 1) : "";
        }
    };

    namespace Specs {
        inline const EYOUMotorSpec EYOU_PP08 = {"EYOU_PP08", 1.0};
        inline const EYOUMotorSpec EYOU_PP11 = {"EYOU_PP11", 6.6};
        inline const EYOUMotorSpec EYOU_PP11L = {"EYOU_PP11L", 12.0};
        // 添加更多型号...
    }

    // 规格查找表
    class EYOUSpecRegistry {
    public:
        static const EYOUMotorSpec *find(const std::string &driver_type);

        static std::vector<std::string> getAllDriverTypes();

    private:
        static const std::unordered_map<std::string, const EYOUMotorSpec *> &getMap();
    };
}
