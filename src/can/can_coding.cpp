/**
 * @Author: ZZSZSZSZZ
 * @CreateTime: 2026-02-21
 * @Description: 
 * @Version: 1.0
 */

#include "can/can_coding.hpp"

namespace robot::can {
    CANFrame CANFrameEncoder::encode(const std::vector<uint8_t> &data, uint32_t canId,
                                  const CANDeviceFrameRequirement &req) {
        // 选择最佳格式
        auto fmt = req.getOptimalFormat();

        // 如果数据太长，必须使用CAN FD
        if (data.size() > 8) {
            fmt = CANFrameFormat::canFd(req.requireExtendedId, data.size(), true);
        }

        CANFrame frame;
        frame.format = fmt;
        frame.id = canId;
        frame.data = data;
        frame.timestamp = 0;

        // 截断或填充数据
        size_t maxLen = fmt.type == CANFrameType::CanFd ? 64 : 8;
        if (frame.data.size() > maxLen) frame.data.resize(maxLen);

        return frame;
    }

    uint8_t CANFrameEncoder::calcDlc(size_t dataLen) {
        // CAN FD DLC编码: 0-8 = 实际长度, 9=12, 10=16, 11=20, 12=24, 13=32, 14=48, 15=64
        static const uint8_t dlcTable[] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 12, 16, 20, 24, 32, 48, 64};
        for (int i = 0; i < 16; i++) {
            if (dlcTable[i] >= dataLen) return i;
        }
        return 15; // 64字节
    }

    // FrameDecoder实现
    bool CANFrameDecoder::validate(const CANFrame &frame, const CANDeviceFrameRequirement &req) {
        return req.isCompatible(frame.format);
    }

    std::vector<uint8_t> CANFrameDecoder::extractPayload(const CANFrame &frame) {
        return frame.data;
    }
}
