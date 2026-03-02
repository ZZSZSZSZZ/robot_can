/**
 * @Author: ZZSZSZSZZ
 * @CreateTime: 2026-02-26
 * @Description: 意优电机 CAN 通信协议常量
 * @Version: 1.0
 */

#pragma once

#include <cstdint>

namespace robot::motor::eyou {
    // 命令类型
    namespace Cmd {
        constexpr uint8_t NETWORK_MGMT = 0x00; // 网络管理 (用于心跳、网络、同步)
        constexpr uint8_t WRITE = 0x01; // 写入命令
        constexpr uint8_t WRITE_REPLY = 0x02; // 写入命令返回
        constexpr uint8_t READ = 0x03; // 读取命令
        constexpr uint8_t READ_REPLY = 0x04; // 读取命令返回
        constexpr uint8_t FAST_WRITE = 0x05; // 快写命令
    }

    // 指令地址
    namespace Addr {
        constexpr uint8_t SERIAL_NUMBER = 0x02; // 设备序列号 ( SN 序列号，使用 data1 - data3 的 3 字节)
        constexpr uint8_t HARDWARE_VERSION = 0x03; // 设备硬件版本号 ( HW 硬件版本)
        constexpr uint8_t FIRMWARE_VERSION = 0x04; // 设备固件版本号 ( FW 固件版本)

        constexpr uint8_t CURRENT_VALUE = 0x05; // 当前扭矩值 (额定扭矩千分比)
        constexpr uint8_t VELOCITY_VALUE = 0x06; // 当前速度值 (每秒变化的脉冲数)
        constexpr uint8_t POSITION_VALUE = 0x07; // 当前位置值 ( 65536 脉冲对应 1 圈)
        constexpr uint8_t TARGET_CURRENT = 0x08; // 设置目标扭矩值 (额定扭矩千分比)
        constexpr uint8_t TARGET_VELOCITY = 0x09; // 设置目标速度值
        constexpr uint8_t TARGET_POSITION = 0x0A; // 设置目标位置值
        constexpr uint8_t TARGET_ACCEL = 0x0B; // 设置目标加速度值
        constexpr uint8_t TARGET_DECEL = 0x0C; // 设置目标加速度值

        constexpr uint8_t WORK_MODE = 0x0F; // 当前工作模式
        constexpr uint8_t ENABLE_STATE = 0x10; // 使能/失能状态 ( 01 = 使能, 00 = 失能)
        constexpr uint8_t STOP_STATE = 0x11; // 结束当前运行状态 ( 01 结束当前运行，电机不失能)
        constexpr uint8_t ALARM_STATUS = 0x15; // 告警指示 (重新使能清空告警)
        constexpr uint8_t GEAR_RATIO = 0x1A; // 电子齿轮比 (减速箱减速比)
        constexpr uint8_t BUS_VOLTAGE = 0x1B; // 当前母线电压值
        constexpr uint8_t TEMPERATURE = 0x1D; // 母线保护工作电压

        constexpr uint8_t MAX_CURRENT = 0x30; // 最大工作扭矩限制
        constexpr uint8_t MAX_VELOCITY = 0x31; // 最大工作速度限制

        constexpr uint8_t POS_MAX_VEL = 0x35; // 位置梯形曲线的最大速度值
        constexpr uint8_t POS_MAX_ACCEL = 0x36; // 位置梯形曲线的加速度最大值
        constexpr uint8_t POS_MAX_DECEL = 0x37; // 位置梯形曲线的减速度最大值
        constexpr uint8_t POS_LIMIT_ENABLE = 0x38; // 位置限位状态
        constexpr uint8_t POS_LIMIT_MAX = 0x39; // 位置的上限值
        constexpr uint8_t POS_LIMIT_MIN = 0x3A; // 位置的下限值
        constexpr uint8_t POS_OFFSET = 0x3B; // 位置的偏置参数值

        constexpr uint8_t CAN_BAUDRATE = 0x4B; // 修改 CAN 波特率 ( data3 = 0x0A (波特率 1M )，默认 05 ( 500K ))
        constexpr uint8_t CAN_ID_SET = 0x4C; // 修改设备的 CANID ( data0 - data2 = 00 后三字节, data3 = 修改 ID )
        constexpr uint8_t SAVE_DATA = 0x4D; // 数据保存 ( 01 存储数据到 eeprom 中)
    }

    // 告警指示
    namespace Alarm {
        constexpr uint32_t OVER_VOLTAGE = 0x0001; // 过压保护 (大于电压门限，小于恢复电压后系统启动)
        constexpr uint32_t UNDER_VOLTAGE = 0x0002; // 欠压保护 (母线电压功率不足)
        constexpr uint32_t OVER_TEMP = 0x0004; // 过温保护 (温度大于设定保护温度)
        constexpr uint32_t BLOCKING_TURNS = 0x0008; // 堵转保护 (运动状态下堵转)
        constexpr uint32_t OVER_CURRENT = 0x0010; // 过流保护
        constexpr uint32_t OVERLOAD = 0x0020; // 过载保护
        constexpr uint32_t MOTOR_LOCK = 0x0040; // 电机锁保护
        constexpr uint32_t PHASE_LOSS = 0x0080; // 缺相保护
        constexpr uint32_t PARAM_ERROR = 0x0100; // 参数读写异常
        constexpr uint32_t ENCODER_MAG_ERROR = 0x0200; // 磁编码器磁场出错
        constexpr uint32_t ENCODER_UVLO = 0x0400; // 磁编码器欠压
        constexpr uint32_t ENCODER_ANGLE_ERROR = 0x0800; // 磁编角度出错
    }

    // 工作模式
    enum class WorkMode : uint32_t {
        ProfilePosition = 1, // 轮廓位置模式
        Velocity = 3, // 速度模式
        Current = 4, // 电流模式
        CyclicSyncPosition = 5, // 周期同步位置模式
    };
}
