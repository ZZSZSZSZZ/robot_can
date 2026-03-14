/**
 * @Author: ZZSZSZSZZ
 * @CreateTime: 2026-03-12
 * @Description: 灵足电机型号与额定参数配置
 * @Version: 1.0
 */

#pragma once

#include <string>
#include <unordered_map>
#include <vector>

namespace robot::motor::lingzu {
    // 灵足电机规格定义
    struct LingzuMotorSpec {
        std::string type;           // 电机型号
        double rated_torque_nm;     // 额定扭矩 (Nm)
        double rated_current_ma;    // 额定电流 (mA)
        double max_velocity_rad_s;  // 最大角速度 (rad/s)
        double kp_max;              // 最大位置增益
        double kd_max;              // 最大速度增益

        /// 获取品牌
        std::string getBrand() const {
            std::string dt(type);
            size_t pos = dt.find('_');
            return pos != std::string::npos ? dt.substr(0, pos) : dt;
        }

        /// 获取型号
        std::string getModel() const {
            std::string dt(type);
            size_t pos = dt.find('_');
            return pos != std::string::npos ? dt.substr(pos + 1) : "";
        }
    };

    // 预定义的灵足电机规格
    namespace Specs {
        // 标准型号
        inline const LingzuMotorSpec LINGZU_RS05 = {
            "LINGZU_RS05",    // 型号
            1.5,                // 额定扭矩 1.5Nm
            2000.0,             // 额定电流 2000mA
            50.0,               // 最大角速度 50rad/s
            500.0,              // 最大位置增益
            5.0                 // 最大速度增益
        };
    }

    // 规格注册表
    class LingzuSpecRegistry {
    public:
        /// 根据电机型号查找规格
        static const LingzuMotorSpec* find(const std::string& type);

        /// 获取所有支持的电机类型
        static std::vector<std::string> getAllDriverTypes();

    private:
        static const std::unordered_map<std::string, const LingzuMotorSpec*>& getMap();
    };
} // namespace robot::motor::lingzu
