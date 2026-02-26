/**
 * @Author: ZZSZSZSZZ
 * @CreateTime: 2026-02-26
 * @Description: 意优电机单位转换
 * @Version: 1.0
 */

#pragma once

#include <cstdint>

namespace robot::motor::eyou {
    namespace Cmd {
        constexpr uint8_t NETWORK_MGMT = 0x00;
        constexpr uint8_t WRITE = 0x01;
        constexpr uint8_t WRITE_REPLY = 0x02;
        constexpr uint8_t READ = 0x03;
        constexpr uint8_t READ_REPLY = 0x04;
        constexpr uint8_t FAST_WRITE = 0x05;
    }

    namespace Addr {
        constexpr uint8_t SERIAL_NUMBER = 0x02;
        constexpr uint8_t HARDWARE_VERSION = 0x03;
        constexpr uint8_t FIRMWARE_VERSION = 0x04;
        constexpr uint8_t CURRENT_VALUE = 0x05;
        constexpr uint8_t VELOCITY_VALUE = 0x06;
        constexpr uint8_t POSITION_VALUE = 0x07;
        constexpr uint8_t TARGET_CURRENT = 0x08;
        constexpr uint8_t TARGET_VELOCITY = 0x09;
        constexpr uint8_t TARGET_POSITION = 0x0A;
        constexpr uint8_t TARGET_ACCEL = 0x0B;
        constexpr uint8_t TARGET_DECEL = 0x0C;
        constexpr uint8_t WORK_MODE = 0x0F;
        constexpr uint8_t ENABLE_STATE = 0x10;
        constexpr uint8_t STOP_STATE = 0x11;
        constexpr uint8_t ALARM_STATUS = 0x15;
        constexpr uint8_t GEAR_RATIO = 0x1A;
        constexpr uint8_t BUS_VOLTAGE = 0x1B;
        constexpr uint8_t TEMPERATURE = 0x1D;
        constexpr uint8_t MAX_CURRENT = 0x30;
        constexpr uint8_t MAX_VELOCITY = 0x31;
        constexpr uint8_t POS_MAX_VEL = 0x35;
        constexpr uint8_t POS_MAX_ACCEL = 0x36;
        constexpr uint8_t POS_MAX_DECEL = 0x37;
        constexpr uint8_t POS_LIMIT_ENABLE = 0x38;
        constexpr uint8_t POS_LIMIT_MAX = 0x39;
        constexpr uint8_t POS_LIMIT_MIN = 0x3A;
        constexpr uint8_t POS_OFFSET = 0x3B;
        constexpr uint8_t CAN_BAUDRATE = 0x4B;
        constexpr uint8_t CAN_ID_SET = 0x4C;
        constexpr uint8_t SAVE_DATA = 0x4D;
    }

    namespace Alarm {
        constexpr uint32_t OVER_VOLTAGE = 0x0001;
        constexpr uint32_t UNDER_VOLTAGE = 0x0002;
        constexpr uint32_t OVER_TEMP = 0x0004;
        constexpr uint32_t OVER_CURRENT = 0x0010;
        constexpr uint32_t OVERLOAD = 0x0020;
        constexpr uint32_t MOTOR_LOCK = 0x0040;
        constexpr uint32_t PHASE_LOSS = 0x0080;
        constexpr uint32_t PARAM_ERROR = 0x0100;
        constexpr uint32_t ENCODER_MAG_ERROR = 0x0200;
        constexpr uint32_t ENCODER_UVLO = 0x0400;
        constexpr uint32_t ENCODER_ANGLE_ERROR = 0x0800;
    }

    enum class WorkMode : uint32_t {
        ProfilePositionVelocity = 1,
        ProfilePositionTime = 2,
        ProfileVelocity = 3,
        Current = 4,
        CyclicSyncPosition = 5,
    };
}
