/**
 * @Author: ZZSZSZSZZ
 * @CreateTime: 2026-02-21
 * @Description: 
 * @Version: 1.0
 */

#pragma once

#include "can_device.hpp"
#include "can_frame.hpp"

namespace robot::can {
    class CANFrameEncoder {
    public:
        // 将协议数据包编码为CAN帧，自动选择格式
        static CANFrame encode(const std::vector<uint8_t> &data, uint32_t canId, const CANDeviceFrameRequirement &req);

        // 计算最佳DLC (CAN FD)
        static uint8_t calcDlc(size_t dataLen);
    };

    // 帧解码器
    class CANFrameDecoder {
    public:
        // 验证帧是否满足设备需求
        static bool validate(const CANFrame &frame, const CANDeviceFrameRequirement &req);

        // 提取数据 payload
        static std::vector<uint8_t> extractPayload(const CANFrame &frame);
    };
}
