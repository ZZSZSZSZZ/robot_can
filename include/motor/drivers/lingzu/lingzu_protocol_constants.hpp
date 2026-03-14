/**
 * @Author: ZZSZSZSZZ
 * @CreateTime: 2026-03-12
 * @Description: 灵足电机 CAN 通信协议常量
 * @Version: 1.0
 */

#pragma once

#include <cstdint>

namespace robot::motor::lingzu {
    // 灵足电机 CAN ID 格式定义
    // 通信类型1 (运控模式):
    //   发送帧: bit28-24=0x01, bit23-8=力矩(0~65535), bit7-0=目标电机CAN_ID
    // 应答帧: bit28-24=命令类型, bit22-23=模式状态, bit21-16=故障代码, bit15-8=当前电机CAN_ID, bit7-0=主机CAN_ID
    // 通信类型7 (设置CAN_ID):
    //   发送帧: bit28-24=0x07, bit23-16=0, bit15-8=0xFD, bit7-0=目标电机CAN_ID
    //   其中 bit16-23 = 预设置CAN_ID (在数据区中)
    // 其他类型:
    //   发送帧: bit28-24=命令类型, bit23-16=0, bit15-8=0xFD, bit7-0=目标电机CAN_ID
    // 应答帧:
    //   bit28-24=命令类型, bit22-23=模式状态, bit21-16=故障代码, bit15-8=0xFD, bit7-0=canid

    // 命令类型 (bit28-24)
    namespace CmdType {
        constexpr uint8_t MOTION_CONTROL = 0x01;      // 运控模式电机控制指令
        constexpr uint8_t FEEDBACK = 0x02;            // 电机反馈数据
        constexpr uint8_t ENABLE = 0x03;              // 电机使能运行
        constexpr uint8_t STOP = 0x04;                // 电机停止运行
        constexpr uint8_t SET_ZERO = 0x06;            // 设置电机机械零位
        constexpr uint8_t SET_CAN_ID = 0x07;          // 设置电机CAN_ID
        constexpr uint8_t WRITE_PARAM = 0x12;         // 单个参数写入 (0x12 = 18)
        constexpr uint8_t SAVE_DATA = 0x16;           // 电机数据保存帧 (0x16 = 22)
        constexpr uint8_t AUTO_REPORT = 0x18;         // 电机主动上报帧 (0x18 = 24)
    }

    // CAN ID 构造辅助常量
    namespace CanId {
        constexpr uint32_t BASE_MASK = 0x0000FD00;    // bit15-8 = 0xFD
        constexpr uint32_t CMD_SHIFT = 24;            // 命令类型左移位数
        constexpr uint32_t MODE_STATE_SHIFT = 22;     // 模式状态左移位数
        constexpr uint32_t FAULT_CODE_SHIFT = 16;     // 故障代码左移位数
        constexpr uint32_t NEW_CAN_ID_SHIFT = 16;     // 新CAN ID左移位数 (用于类型7)
        constexpr uint32_t TORQUE_SHIFT = 8;          // 力矩左移位数 (用于类型1)
    }

    // 模式状态 (bit22-23, 仅在应答帧中有效)
    namespace ModeState {
        constexpr uint8_t RESET = 0;                  // Reset模式[复位]
        constexpr uint8_t CALI = 1;                   // Cali模式[标定]
        constexpr uint8_t MOTOR = 2;                  // Motor模式[运行]
    }

    // 故障代码位定义 (bit21-16, 仅在应答帧中有效)
    namespace Fault {
        constexpr uint8_t NONE = 0;                   // 无故障
        constexpr uint8_t UNDEFINED = 0x01;           // bit21: 未标定
        constexpr uint8_t STALL_OVERLOAD = 0x02;      // bit20: 堵转过载故障
        constexpr uint8_t ENCODER_FAULT = 0x04;       // bit19: 磁编码器故障
        constexpr uint8_t OVER_TEMP = 0x08;           // bit18: 过温
        constexpr uint8_t DRIVER_FAULT = 0x10;        // bit17: 驱动故障
        constexpr uint8_t UNDER_VOLTAGE = 0x20;       // bit16: 欠压故障
    }

    // 数据区索引定义
    namespace DataIndex {
        constexpr uint8_t POS_HIGH = 0;               // 目标角度高字节
        constexpr uint8_t POS_LOW = 1;                // 目标角度低字节
        constexpr uint8_t VEL_HIGH = 2;               // 目标角速度高字节
        constexpr uint8_t VEL_LOW = 3;                // 目标角速度低字节
        constexpr uint8_t KP_HIGH = 4;                // Kp高字节
        constexpr uint8_t KP_LOW = 5;                 // Kp低字节
        constexpr uint8_t KD_HIGH = 6;                // Kd高字节
        constexpr uint8_t KD_LOW = 7;                 // Kd低字节
    }

    // 参数范围定义
    namespace Limits {
        constexpr uint16_t POS_MIN = 0;               // 角度最小值
        constexpr uint16_t POS_MAX = 65535;           // 角度最大值
        constexpr double POS_RAD_MIN = -12.57;        // 角度最小值(弧度)
        constexpr double POS_RAD_MAX = 12.57;         // 角度最大值(弧度)

        constexpr uint16_t VEL_MIN = 0;               // 角速度最小值
        constexpr uint16_t VEL_MAX = 65535;           // 角速度最大值
        constexpr double VEL_RAD_S_MIN = -50.0;       // 角速度最小值(rad/s)
        constexpr double VEL_RAD_S_MAX = 50.0;        // 角速度最大值(rad/s)

        constexpr uint16_t KP_MIN = 0;                // Kp最小值
        constexpr uint16_t KP_MAX = 65535;            // Kp最大值
        constexpr double KP_REAL_MIN = 0.0;           // Kp实际最小值
        constexpr double KP_REAL_MAX = 500.0;         // Kp实际最大值

        constexpr uint16_t KD_MIN = 0;                // Kd最小值
        constexpr uint16_t KD_MAX = 65535;            // Kd最大值
        constexpr double KD_REAL_MIN = 0.0;           // Kd实际最小值
        constexpr double KD_REAL_MAX = 5.0;           // Kd实际最大值

        constexpr double TORQUE_MIN = -5.5;           // 力矩最小值(Nm)
        constexpr double TORQUE_MAX = 5.5;            // 力矩最大值(Nm)

        constexpr double TEMP_SCALE = 10.0;           // 温度缩放因子 (实际温度 = 原始值 / 10)
    }

    // 主动上报开关
    namespace AutoReport {
        constexpr uint8_t DISABLE = 0x00;             // 关闭主动上报(默认)
        constexpr uint8_t ENABLE = 0x01;              // 开启主动上报(默认间隔10ms)
    }
} // namespace robot::motor::lingzu
