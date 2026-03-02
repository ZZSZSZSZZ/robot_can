/**
 * @Author: ZZSZSZSZZ
 * @CreateTime: 2026-02-26
 * @Description: 电机型号与额定参数配置绑定
 * @Version: 1.0
 */

#pragma once

#include <string>
#include <unordered_map>
#include <vector>

namespace robot::motor::eyou {
    // 电机型号与额定参数配置绑定
    struct EYOUMotorSpec {
        std::string type; // 电机型号
        double rated_torque_nm; // 额定扭矩 (Nm)
        double rated_current_ma = 1200.0; // 额定电流 (mA)

        /// 获取品牌
        /// @return 品牌名
        std::string getBrand() const {
            std::string dt(type);
            size_t pos = dt.find('_');
            return pos != std::string::npos ? dt.substr(0, pos) : dt;
        }

        /// 获取型号
        /// @return 型号
        std::string getModel() const {
            std::string dt(type);
            size_t pos = dt.find('_');
            return pos != std::string::npos ? dt.substr(pos + 1) : "";
        }
    };

    // 定义电机型号与额定参数配置
    namespace Specs {
        inline const EYOUMotorSpec EYOU_PP08 = {"EYOU_PP08", 1.0};
        inline const EYOUMotorSpec EYOU_PP11 = {"EYOU_PP11", 6.6};
        inline const EYOUMotorSpec EYOU_PP11L = {"EYOU_PP11L", 12.0};
    }

    // 规格查找表
    class EYOUSpecRegistry {
    public:
        /// 根据电机型号查找额定参数绑定表
        /// @param type 电机型号
        /// @return 额定参数绑定表
        static const EYOUMotorSpec *find(const std::string &type);

        static std::vector<std::string> getAllDriverTypes();

    private:
        static const std::unordered_map<std::string, const EYOUMotorSpec *> &getMap();
    };
}
