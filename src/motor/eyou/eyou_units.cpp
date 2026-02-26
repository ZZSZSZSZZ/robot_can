/**
 * @Author: ZZSZSZSZZ
 * @CreateTime: 2026-02-26
 * @Description: 
 * @Version: 1.0
 */

#include "motor/drivers/eyou/eyou_units.hpp"

namespace robot::motor::eyou {
    int32_t EYOUUnits::degreesToPulses(double degrees) {
        return static_cast<int32_t>(degrees * PULSES_PER_REV / DEGREES_PER_REV);
    }

    double EYOUUnits::pulsesToDegrees(int32_t pulses) {
        return pulses * DEGREES_PER_REV / PULSES_PER_REV;
    }

    int32_t EYOUUnits::radiansToPulses(double radians) {
        return static_cast<int32_t>(radians * PULSES_PER_REV / (2.0 * M_PI));
    }

    double EYOUUnits::pulsesToRadians(int32_t pulses) {
        return pulses * 2.0 * M_PI / PULSES_PER_REV;
    }

    int32_t EYOUUnits::rpmToPulsesPerSec(double rpm) {
        return static_cast<int32_t>(rpm * PULSES_PER_REV / 60.0);
    }

    double EYOUUnits::pulsesPerSecToRpm(int32_t pulsesPerSec) {
        return pulsesPerSec * 60.0 / PULSES_PER_REV;
    }

    int32_t EYOUUnits::radPerSecToPulses(double rad_s) {
        return static_cast<int32_t>(rad_s * 60.0 * PULSES_PER_REV / (2.0 * M_PI));
    }

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
        return (current_ma / rated_current_ma) * rated_torque_nm;
    }

    double EYOUUnits::torqueToCurrent(double torque_nm, double rated_torque_nm, double rated_current_ma) {
        if (rated_torque_nm <= 0.0) {
            return 0.0; // 防止除零
        }
        return (torque_nm / rated_torque_nm) * rated_current_ma;
    }

    double EYOUUnits::currentToTorque(double current_ma, const EYOUMotorSpec &spec) {
        return currentToTorque(current_ma, spec.rated_torque_nm, spec.rated_current_ma);
    }

    double EYOUUnits::torqueToCurrent(double torque_nm, const EYOUMotorSpec &spec) {
        return torqueToCurrent(torque_nm, spec.rated_torque_nm, spec.rated_current_ma);
    }
}
