/**
 * @Author: ZZSZSZSZZ
 * @CreateTime: 2026-02-26
 * @Description: 意优电机单位转换类
 * @Version: 1.0
 */

#pragma once

#include "motor/drivers/eyou/eyou_types.hpp"
#include <cmath>

namespace robot::motor::eyou {
    class EYOUUnits {
    public:
        static constexpr double PULSES_PER_REV = 65536.0;
        static constexpr double DEGREES_PER_REV = 360.0;
        static constexpr double DEFAULT_RATED_CURRENT_MA = 1200.0;

        static int32_t degreesToPulses(double degrees);

        static double pulsesToDegrees(int32_t pulses);

        static int32_t radiansToPulses(double radians);

        static double pulsesToRadians(int32_t pulses);

        static int32_t rpmToPulsesPerSec(double rpm);

        static double pulsesPerSecToRpm(int32_t pulsesPerSec);

        static int32_t rpsToPulsesPerSec(double rps);

        static double pulsesPerSecToRps(int32_t pulsesPerSec);

        static int32_t radPerSecToPulses(double rad_s);

        static double pulsesToRadPerSec(int32_t pulses);

        static int32_t ampsToMilliamps(double amps);

        static double milliampsToAmps(int32_t milliamps);

        static double currentToTorque(double current_ma, double rated_torque_nm,
                                      double rated_current_ma = DEFAULT_RATED_CURRENT_MA);

        static double torqueToCurrent(double torque_nm, double rated_torque_nm,
                                      double rated_current_ma = DEFAULT_RATED_CURRENT_MA);

        static double currentToTorque(double current_ma, const EYOUMotorSpec &spec);

        static double torqueToCurrent(double torque_nm, const EYOUMotorSpec &spec);
    };
}
