/**
 * @Author: ZZSZSZSZZ
 * @CreateTime: 2026-03-12
 * @Description: 灵足电机单位转换实现
 * @Version: 1.0
 */

#include "motor/drivers/lingzu/lingzu_units.hpp"

namespace robot::motor::lingzu {

    // 角度转换: [0~65535] <=> [-12.57f~12.57f]
    uint16_t LingzuUnits::radiansToPosition(double radians) {
        // 限制在有效范围内
        if (radians < Limits::POS_RAD_MIN) radians = Limits::POS_RAD_MIN;
        if (radians > Limits::POS_RAD_MAX) radians = Limits::POS_RAD_MAX;

        // 线性映射: -12.57 -> 0, 12.57 -> 65535
        double normalized = (radians - Limits::POS_RAD_MIN) / (Limits::POS_RAD_MAX - Limits::POS_RAD_MIN);
        return static_cast<uint16_t>(normalized * Limits::POS_MAX);
    }

    double LingzuUnits::positionToRadians(uint16_t position) {
        // 线性映射: 0 -> -12.57, 65535 -> 12.57
        double normalized = static_cast<double>(position) / Limits::POS_MAX;
        return Limits::POS_RAD_MIN + normalized * (Limits::POS_RAD_MAX - Limits::POS_RAD_MIN);
    }

    // 角速度转换: [0~65535] <=> [-50rad/s~50rad/s]
    uint16_t LingzuUnits::radPerSecToVelocity(double rad_s) {
        // 限制在有效范围内
        if (rad_s < Limits::VEL_RAD_S_MIN) rad_s = Limits::VEL_RAD_S_MIN;
        if (rad_s > Limits::VEL_RAD_S_MAX) rad_s = Limits::VEL_RAD_S_MAX;

        // 线性映射: -50 -> 0, 50 -> 65535
        double normalized = (rad_s - Limits::VEL_RAD_S_MIN) / (Limits::VEL_RAD_S_MAX - Limits::VEL_RAD_S_MIN);
        return static_cast<uint16_t>(normalized * Limits::VEL_MAX);
    }

    double LingzuUnits::velocityToRadPerSec(uint16_t velocity) {
        // 线性映射: 0 -> -50, 65535 -> 50
        double normalized = static_cast<double>(velocity) / Limits::VEL_MAX;
        return Limits::VEL_RAD_S_MIN + normalized * (Limits::VEL_RAD_S_MAX - Limits::VEL_RAD_S_MIN);
    }

    // Kp转换: [0~65535] <=> [0.0~500.0]
    uint16_t LingzuUnits::kpToRaw(double kp) {
        // 限制在有效范围内
        if (kp < Limits::KP_REAL_MIN) kp = Limits::KP_REAL_MIN;
        if (kp > Limits::KP_REAL_MAX) kp = Limits::KP_REAL_MAX;

        // 线性映射
        double normalized = (kp - Limits::KP_REAL_MIN) / (Limits::KP_REAL_MAX - Limits::KP_REAL_MIN);
        return static_cast<uint16_t>(normalized * Limits::KP_MAX);
    }

    double LingzuUnits::rawToKp(uint16_t raw) {
        // 线性映射
        double normalized = static_cast<double>(raw) / Limits::KP_MAX;
        return Limits::KP_REAL_MIN + normalized * (Limits::KP_REAL_MAX - Limits::KP_REAL_MIN);
    }

    // Kd转换: [0~65535] <=> [0.0~5.0]
    uint16_t LingzuUnits::kdToRaw(double kd) {
        // 限制在有效范围内
        if (kd < Limits::KD_REAL_MIN) kd = Limits::KD_REAL_MIN;
        if (kd > Limits::KD_REAL_MAX) kd = Limits::KD_REAL_MAX;

        // 线性映射
        double normalized = (kd - Limits::KD_REAL_MIN) / (Limits::KD_REAL_MAX - Limits::KD_REAL_MIN);
        return static_cast<uint16_t>(normalized * Limits::KD_MAX);
    }

    double LingzuUnits::rawToKd(uint16_t raw) {
        // 线性映射
        double normalized = static_cast<double>(raw) / Limits::KD_MAX;
        return Limits::KD_REAL_MIN + normalized * (Limits::KD_REAL_MAX - Limits::KD_REAL_MIN);
    }

    // 力矩转换: [0~65535] <=> [-5.5Nm~5.5Nm]
    uint16_t LingzuUnits::torqueToRaw(double torque_nm) {
        // 限制在有效范围内
        if (torque_nm < Limits::TORQUE_MIN) torque_nm = Limits::TORQUE_MIN;
        if (torque_nm > Limits::TORQUE_MAX) torque_nm = Limits::TORQUE_MAX;

        // 线性映射: -5.5 -> 0, 5.5 -> 65535
        double normalized = (torque_nm - Limits::TORQUE_MIN) / (Limits::TORQUE_MAX - Limits::TORQUE_MIN);
        return static_cast<uint16_t>(normalized * 65535);
    }

    double LingzuUnits::rawToTorque(uint16_t raw) {
        // 线性映射: 0 -> -5.5, 65535 -> 5.5
        double normalized = static_cast<double>(raw) / 65535.0;
        return Limits::TORQUE_MIN + normalized * (Limits::TORQUE_MAX - Limits::TORQUE_MIN);
    }

    // 温度转换: 原始值 = Temp(摄氏度) * 10
    uint16_t LingzuUnits::celsiusToRaw(double celsius) {
        return static_cast<uint16_t>(celsius * Limits::TEMP_SCALE);
    }

    double LingzuUnits::rawToCelsius(uint16_t raw) {
        return static_cast<double>(raw) / Limits::TEMP_SCALE;
    }

    // CAN ID 构建辅助函数
    // 标准发送帧: bit28-24 命令类型, bit23-16=0, bit15-8=0xFD, bit7-0=canid
    uint32_t LingzuUnits::buildTransmitCanId(uint8_t cmd_type, uint8_t can_id) {
        uint32_t can_id_ext = 0;
        can_id_ext |= (static_cast<uint32_t>(cmd_type) << CanId::CMD_SHIFT);
        can_id_ext |= CanId::BASE_MASK;  // bit15-8 = 0xFD
        can_id_ext |= static_cast<uint32_t>(can_id);
        return can_id_ext;
    }

    // 运控模式发送帧(类型1): bit28-24=0x01, bit23-8=力矩, bit7-0=canid
    uint32_t LingzuUnits::buildMotionControlCanId(uint16_t torque_raw, uint8_t can_id) {
        uint32_t can_id_ext = 0;
        can_id_ext |= (static_cast<uint32_t>(CmdType::MOTION_CONTROL) << CanId::CMD_SHIFT);
        can_id_ext |= (static_cast<uint32_t>(torque_raw) << CanId::TORQUE_SHIFT);  // bit23-8 = 力矩
        can_id_ext |= static_cast<uint32_t>(can_id);
        return can_id_ext;
    }

    // 设置CAN ID帧(类型7): bit28-24=0x07, bit23-16=0, bit15-8=0xFD, bit7-0=原canid
    uint32_t LingzuUnits::buildSetCanId(uint8_t original_can_id) {
        uint32_t can_id_ext = 0;
        can_id_ext |= (static_cast<uint32_t>(CmdType::SET_CAN_ID) << CanId::CMD_SHIFT);
        can_id_ext |= CanId::BASE_MASK;  // bit15-8 = 0xFD
        can_id_ext |= static_cast<uint32_t>(original_can_id);
        return can_id_ext;
    }

    // 应答帧解析: bit28-24 命令类型, bit22-23 模式状态, bit21-16 故障代码, bit15-8=当前电机CAN_ID, bit7-0=主机CAN_ID
    uint8_t LingzuUnits::parseResponseCmdType(uint32_t can_id) {
        return static_cast<uint8_t>((can_id >> CanId::CMD_SHIFT) & 0x1F);
    }

    uint8_t LingzuUnits::parseResponseModeState(uint32_t can_id) {
        return static_cast<uint8_t>((can_id >> CanId::MODE_STATE_SHIFT) & 0x03);
    }

    uint8_t LingzuUnits::parseResponseFaultCode(uint32_t can_id) {
        return static_cast<uint8_t>((can_id >> CanId::FAULT_CODE_SHIFT) & 0x3F);
    }

    // 解析电机CAN_ID (bit8-15)
    uint8_t LingzuUnits::parseResponseCanId(uint32_t can_id) {
        return static_cast<uint8_t>((can_id >> 8) & 0xFF);
    }

} // namespace robot::motor::lingzu
