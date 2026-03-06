/**
 * @Author: ZZSZSZSZZ
 * @CreateTime: 2026-02-26
 * @Description: 实现 EYOU 电机驱动相关的单位转换函数
 * @Version: 1.0
 */

#include "motor/drivers/eyou/eyou_units.hpp"

namespace robot::motor::eyou {
    /// 角度转换为脉冲数
    /// @param degrees 角度
    /// @return 脉冲数
    /// @note 转换公式: 脉冲数 = 角度 * 65536 / 360.0
    int32_t EYOUUnits::degreesToPulses(double degrees) {
        return static_cast<int32_t>(degrees * PULSES_PER_REV / DEGREES_PER_REV);
    }

    /// 脉冲数转换为角度
    /// @param pulses 脉冲数
    /// @return 角度
    /// @note 转换公式: 角度 = 脉冲数 * 360.0 / 65536
    double EYOUUnits::pulsesToDegrees(int32_t pulses) {
        return pulses * DEGREES_PER_REV / PULSES_PER_REV;
    }

    // int32_t EYOUUnits::radiansToPulses(double radians) {
    //     return static_cast<int32_t>(radians * PULSES_PER_REV / (2.0 * M_PI));
    // }

    /// 弧度转换为脉冲数
    /// @param radians 弧度
    /// @return 脉冲数
    /// @note 转换公式: 脉冲数 = 弧度 * 65536 / 2π
    int32_t EYOUUnits::radiansToPulses(double radians) {
        bool is_negative = radians < 0;

        if (is_negative) {
            radians += 2.0 * M_PI; // 负弧度映射到 ( 0, 2π )
        }

        double v = radians * PULSES_PER_REV / (2.0 * M_PI);
        auto rounded = static_cast<int32_t>(std::llround(v));

        auto low = static_cast<uint16_t>(rounded); // 低16位脉冲值

        if (is_negative) {
            // 负角度：高16位全1，低16位为映射后的脉冲值
            return (0xFFFF) << 16 | low;
        }
        // 正角度：高16位为0，低16位为直接计算的脉冲值
        return low;
    }

    double EYOUUnits::pulsesToRadians(const int32_t pulses) {
        return pulses * 2.0 * M_PI / PULSES_PER_REV;
    }

    /// 转速每分转换为脉冲数
    /// @param rpm 转速每分
    /// @return 脉冲数
    /// @note 转换公式: 脉冲数 = 转速每分 * 65536 / 60
    int32_t EYOUUnits::rpmToPulsesPerSec(const double rpm) {
        return static_cast<int32_t>(rpm * PULSES_PER_REV / 60.0);
    }

    /// 脉冲数转换为转速每分
    /// @param pulsesPerSec 脉冲数
    /// @return 转速每分
    /// @note 转换公式: 转速每分 = 脉冲数 * 60 / 65536
    double EYOUUnits::pulsesPerSecToRpm(const int32_t pulsesPerSec) {
        return pulsesPerSec * 60.0 / PULSES_PER_REV;
    }

    /// 转速每秒转换为脉冲数
    /// @param rps 转速每秒
    /// @return 脉冲数
    /// @note 转换公式: 脉冲数 = 转速每秒 * 65536 / 60
    int32_t EYOUUnits::rpsToPulsesPerSec(const double rps) {
        return static_cast<int32_t>(rps * PULSES_PER_REV);
    }

    /// 脉冲数转换为转速每秒
    /// @param pulsesPerSec 脉冲数
    /// @return 转速每秒
    /// @note 转换公式: 转速每秒 = 脉冲数 * 60 / 65536
    double EYOUUnits::pulsesPerSecToRps(int32_t pulsesPerSec) {
        return pulsesPerSec / PULSES_PER_REV;
    }

    /// 角速度转换为脉冲每秒
    /// @param rad_s 角速度
    /// @return
    /// @note 转换公式: 脉冲每秒 = 角速度 * 60 * 65536 / 2π
    int32_t EYOUUnits::radPerSecToPulses(double rad_s) {
        return static_cast<int32_t>(rad_s * 60.0 * PULSES_PER_REV / (2.0 * M_PI));
    }

    /// 脉冲每秒转换为角速度
    /// @param pulses 脉冲每秒
    /// @return 角速度
    /// @note 转换公式：角速度 = 脉冲每秒 * 2π / (60 * 65536)
    double EYOUUnits::pulsesToRadPerSec(int32_t pulses) {
        return pulses * 2.0 * M_PI / (60.0 * PULSES_PER_REV);
    }

    int32_t EYOUUnits::ampsToMilliamps(double amps) {
        return static_cast<int32_t>(amps * 1000.0);
    }

    double EYOUUnits::milliampsToAmps(int32_t milliamps) {
        return milliamps / 1000.0;
    }

    double EYOUUnits::currentToTorque(double current_ma, double rated_torque_nm, double rated_current_ma) {
        if (rated_current_ma <= 0.0) {
            return 0.0; // 防止除零
        }
        return current_ma / rated_current_ma * rated_torque_nm;
    }

    double EYOUUnits::torqueToCurrent(double torque_nm, double rated_torque_nm, double rated_current_ma) {
        if (rated_torque_nm <= 0.0) {
            return 0.0; // 防止除零
        }
        return torque_nm / rated_torque_nm * rated_current_ma;
    }

    double EYOUUnits::currentToTorque(double current_ma, const EYOUMotorSpec &spec) {
        return currentToTorque(current_ma, spec.rated_torque_nm, spec.rated_current_ma);
    }

    double EYOUUnits::torqueToCurrent(double torque_nm, const EYOUMotorSpec &spec) {
        return torqueToCurrent(torque_nm, spec.rated_torque_nm, spec.rated_current_ma);
    }
}
