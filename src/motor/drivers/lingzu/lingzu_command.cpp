/**
 * @Author: ZZSZSZSZZ
 * @CreateTime: 2026-03-12
 * @Description: 灵足电机命令实现
 * @Version: 1.0
 */

#include "motor/drivers/lingzu/lingzu_command.hpp"
#include "motor/drivers/lingzu/lingzu_protocol_constants.hpp"
#include "motor/drivers/lingzu/lingzu_units.hpp"
#include "can/can_coding.hpp"

namespace robot::motor::lingzu {

    // 辅助函数 - 构建灵足扩展帧
    static CANFrame makeExtendedFrame(uint32_t id, const std::vector<uint8_t>& data) {
        std::vector<uint8_t> payload = data;
        payload.resize(8, 0x00);  // 确保8字节

        can::CANDeviceFrameRequirement req{};
        req.preferredType = can::CANFrameType::Extended;  // 灵足使用扩展帧
        req.requireExtendedId = true;
        req.maxDataLength = 8;
        req.requireCanFd = false;

        return can::CANFrameEncoder::encode(payload, id, req);
    }

    // 辅助函数 - 编码16位数据到字节数组 (高字节在前)
    static void encodeUint16BE(uint16_t value, std::vector<uint8_t>& data, size_t offset) {
        if (data.size() < offset + 2) return;
        data[offset] = static_cast<uint8_t>((value >> 8) & 0xFF);     // 高字节
        data[offset + 1] = static_cast<uint8_t>(value & 0xFF);        // 低字节
    }

    // 运控模式命令实现
    std::vector<CANFrame> LingzuMITCmd::encode(uint32_t motor_id) const {
        // 转换物理值到协议值
        uint16_t pos_raw = LingzuUnits::radiansToPosition(position_);
        uint16_t vel_raw = LingzuUnits::radPerSecToVelocity(velocity_);
        uint16_t torque_raw = LingzuUnits::torqueToRaw(torque_);
        uint16_t kp_raw = LingzuUnits::kpToRaw(kp_);
        uint16_t kd_raw = LingzuUnits::kdToRaw(kd_);

        // 构建发送帧CAN ID: bit28-24=0x01, bit23-8=力矩, bit7-0=canid
        uint32_t tx_can_id = LingzuUnits::buildMotionControlCanId(torque_raw, static_cast<uint8_t>(motor_id));

        // 构建8字节数据区
        std::vector<uint8_t> data(8, 0x00);

        // 填充数据区 (高字节在前)
        encodeUint16BE(pos_raw, data, DataIndex::POS_HIGH);
        encodeUint16BE(vel_raw, data, DataIndex::VEL_HIGH);
        encodeUint16BE(kp_raw, data, DataIndex::KP_HIGH);
        encodeUint16BE(kd_raw, data, DataIndex::KD_HIGH);

        return {makeExtendedFrame(tx_can_id, data)};
    }

    // 使能/失能命令实现
    std::vector<CANFrame> LingzuEnableCmd::encode(uint32_t motor_id) const {
        // 构建发送帧CAN ID
        uint32_t tx_can_id = LingzuUnits::buildTransmitCanId(CmdType::ENABLE, static_cast<uint8_t>(motor_id));

        // 使能命令数据区全0即可
        std::vector<uint8_t> data(8, 0x00);

        return {makeExtendedFrame(tx_can_id, data)};
    }

    // 停止命令实现
    std::vector<CANFrame> LingzuStopCmd::encode(uint32_t motor_id) const {
        // 构建发送帧CAN ID
        uint32_t tx_can_id = LingzuUnits::buildTransmitCanId(CmdType::STOP, static_cast<uint8_t>(motor_id));

        std::vector<uint8_t> data(8, 0x00);
        // Byte[0]=1时表示清故障
        if (clear_fault_) {
            data[0] = 0x01;
        }

        return {makeExtendedFrame(tx_can_id, data)};
    }

    // 设置机械零位命令实现
    std::vector<CANFrame> LingzuSetZeroCmd::encode(uint32_t motor_id) const {
        // 构建发送帧CAN ID
        uint32_t tx_can_id = LingzuUnits::buildTransmitCanId(CmdType::SET_ZERO, static_cast<uint8_t>(motor_id));

        std::vector<uint8_t> data(8, 0x00);
        // Byte[0]=1
        data[0] = 0x01;

        return {makeExtendedFrame(tx_can_id, data)};
    }

    // 设置CAN ID命令实现
    // 通信类型7: bit28-24=0x07, bit23-16=0, bit15-8=0xFD, bit7-0=目标电机CAN_ID
    // 新CAN ID在数据区Byte0
    std::vector<CANFrame> LingzuSetCanIdCmd::encode(uint32_t motor_id) const {
        // 构建发送帧CAN ID (使用原CAN ID作为目标)
        uint32_t tx_can_id = LingzuUnits::buildSetCanId(static_cast<uint8_t>(motor_id));

        // 数据区Byte0 = 新的CAN ID，其余为0
        std::vector<uint8_t> data(8, 0x00);
        data[0] = new_can_id_;

        return {makeExtendedFrame(tx_can_id, data)};
    }

    // 保存数据命令实现
    std::vector<CANFrame> LingzuSaveDataCmd::encode(uint32_t motor_id) const {
        // 构建发送帧CAN ID
        uint32_t tx_can_id = LingzuUnits::buildTransmitCanId(CmdType::SAVE_DATA, static_cast<uint8_t>(motor_id));

        // 固定数据: 01 02 03 04 05 06 07 08
        std::vector<uint8_t> data = {0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08};

        return {makeExtendedFrame(tx_can_id, data)};
    }

    // 主动上报控制命令实现
    std::vector<CANFrame> LingzuAutoReportCmd::encode(uint32_t motor_id) const {
        // 构建发送帧CAN ID
        uint32_t tx_can_id = LingzuUnits::buildTransmitCanId(CmdType::AUTO_REPORT, static_cast<uint8_t>(motor_id));

//        std::vector<uint8_t> data(8, 0x00);
        std::vector<uint8_t> data = {0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x00, 0x00};
        // Byte[5] = F_CMD (上报开关)
        data[6] = enable_ ? AutoReport::ENABLE : AutoReport::DISABLE;

        return {makeExtendedFrame(tx_can_id, data)};
    }

    // 单个参数写入命令实现
    std::vector<CANFrame> LingzuWriteParamCmd::encode(uint32_t motor_id) const {
        // 构建发送帧CAN ID
        uint32_t tx_can_id = LingzuUnits::buildTransmitCanId(CmdType::WRITE_PARAM, static_cast<uint8_t>(motor_id));

        std::vector<uint8_t> data(8, 0x00);

        // Byte0~1: index (参数索引)
        encodeUint16BE(index_, data, 0);

        // Byte2~3: 保留, 填0

        // Byte4~7: 参数数据 (低字节在前)
        data[4] = static_cast<uint8_t>(value_ & 0xFF);
        data[5] = static_cast<uint8_t>((value_ >> 8) & 0xFF);
        data[6] = static_cast<uint8_t>((value_ >> 16) & 0xFF);
        data[7] = static_cast<uint8_t>((value_ >> 24) & 0xFF);

        return {makeExtendedFrame(tx_can_id, data)};
    }

} // namespace robot::motor::lingzu
