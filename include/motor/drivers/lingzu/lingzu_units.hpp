/**
 * @Author: ZZSZSZSZZ
 * @CreateTime: 2026-03-12
 * @Description: 灵足电机单位转换类
 * @Version: 1.0
 */

#pragma once

#include <cmath>
#include "lingzu_specification.hpp"
#include "lingzu_protocol_constants.hpp"

namespace robot::motor::lingzu {
    class LingzuUnits {
    public:
        // 角度转换: [0~65535] <=> [-12.57f~12.57f]
        static uint16_t radiansToPosition(double radians);
        static double positionToRadians(uint16_t position);

        // 角速度转换: [0~65535] <=> [-50rad/s~50rad/s]
        static uint16_t radPerSecToVelocity(double rad_s);
        static double velocityToRadPerSec(uint16_t velocity);

        // Kp转换: [0~65535] <=> [0.0~500.0]
        static uint16_t kpToRaw(double kp);
        static double rawToKp(uint16_t raw);

        // Kd转换: [0~65535] <=> [0.0~5.0]
        static uint16_t kdToRaw(double kd);
        static double rawToKd(uint16_t raw);

        // 力矩转换: [0~65535] <=> [-5.5Nm~5.5Nm]
        static uint16_t torqueToRaw(double torque_nm);
        static double rawToTorque(uint16_t raw);

        // 温度转换: 原始值 = Temp(摄氏度) * 10
        static uint16_t celsiusToRaw(double celsius);
        static double rawToCelsius(uint16_t raw);

        // CAN ID 构建辅助函数
        // 标准发送帧: bit28-24 命令类型, bit23-16=0, bit15-8=0xFD, bit7-0=canid
        static uint32_t buildTransmitCanId(uint8_t cmd_type, uint8_t can_id);

        // 运控模式发送帧(类型1): bit28-24=0x01, bit23-8=力矩, bit7-0=canid
        static uint32_t buildMotionControlCanId(uint16_t torque_raw, uint8_t can_id);

        // 设置CAN ID帧(类型7): bit28-24=0x07, bit23-16=0, bit15-8=0xFD, bit7-0=原canid
        // 新CAN ID在数据区Byte0
        static uint32_t buildSetCanId(uint8_t original_can_id);

        // 应答帧解析: bit28-24 命令类型, bit22-23 模式状态, bit21-16 故障代码, bit15-8=当前电机CAN_ID, bit7-0=主机CAN_ID
        static uint8_t parseResponseCmdType(uint32_t can_id);
        static uint8_t parseResponseModeState(uint32_t can_id);
        static uint8_t parseResponseFaultCode(uint32_t can_id);
        // 解析电机CAN_ID (bit8-15)
        static uint8_t parseResponseCanId(uint32_t can_id);
    };
} // namespace robot::motor::lingzu
