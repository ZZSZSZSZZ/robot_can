/**
 * @Author: ZZSZSZSZZ
 * @CreateTime: 2026-02-17
 * @Description: CAN 帧结构体 支持标准/扩展, 以及 CAN FD
 * @Version: 1.0
 */

#pragma once

#include <cstdint>
#include <vector>
#include <cstring>
#include <algorithm>
#include <linux/can.h>
#include <linux/can/raw.h>

namespace robot::can {
    // CAN 帧类型
    enum class CANFrameType : uint8_t {
        Standard, // 标准 CAN 帧 (11 位 ID, 最多 8 字节数据)
        Extended, // 扩展 CAN 帧 (29 位 ID, 最多 8 字节数据)
        CanFd // CAN FD 帧 (11/29 位 ID, 最多 64 字节数据, 支持波特率切换 BRS 和错误状态指示 ESI)
    };

    // CAN 帧格式
    struct CANFrameFormat {
        CANFrameType type; // 帧类型
        bool isExtendedId; // 是否使用扩展ID
        uint8_t dlc; // 数据长度代码
        bool bitRateSwitch; // CAN FD: 是否使用BRS (波特率切换)
        bool errorStateInd; // CAN FD: 错误状态指示

        // 创建标准 CAN 帧格式
        static CANFrameFormat standard(const uint8_t dataLen = 8) {
            return {CANFrameType::Standard, false, std::min(dataLen, static_cast<uint8_t>(8)), false, false};
        }

        // 创建扩展 CAN 帧格式
        static CANFrameFormat extended(const uint8_t dataLen = 8) {
            return {CANFrameType::Extended, true, std::min(dataLen, static_cast<uint8_t>(8)), false, false};
        }

        // 创建 CAN FD 帧格式
        static CANFrameFormat canFd(const bool extId = false, const uint8_t dataLen = 64, const bool brs = true) {
            return {CANFrameType::CanFd, extId, std::min(dataLen, static_cast<uint8_t>(64)), brs, false};
        }
    };

    // CAN 帧
    struct CANFrame {
        CANFrameFormat format; // CAN 帧格式
        uint32_t id; // CAN ID
        std::vector<uint8_t> data; // CAN 帧数据
        uint64_t timestamp; // 接收/发送时间戳 (微秒)

        CANFrame() = default;

        /// 从 Linux CAN 帧结构体构造 (标准/扩展)
        /// @param frame CAN 帧
        /// @param type 标准/扩展
        explicit CANFrame(const can_frame &frame, const CANFrameType type = CANFrameType::Standard) {
            format.type = type;
            // 判读是否为扩展 CAN ID
            format.isExtendedId = (frame.can_id & CAN_EFF_FLAG) != 0;
            format.dlc = frame.can_dlc;
            // 只在 CAN FD 中使用
            format.bitRateSwitch = false;
            format.errorStateInd = false;
            // 提取 CAN ID
            id = frame.can_id & (format.isExtendedId ? CAN_EFF_MASK : CAN_SFF_MASK);
            // 填充数据
            data.assign(frame.data, frame.data + frame.can_dlc);
        }

        /// 从 Linux CAN FD 帧结构体构造
        /// @param frame CAN FD 帧
        explicit CANFrame(const canfd_frame &frame) {
            format.type = CANFrameType::CanFd;
            // 判读是否为扩展 CAN ID
            format.isExtendedId = (frame.can_id & CAN_EFF_FLAG) != 0;
            format.dlc = frame.len;
            // 提取标志位
            format.bitRateSwitch = (frame.flags & CANFD_BRS) != 0;
            format.errorStateInd = (frame.flags & CANFD_ESI) != 0;
            // 提取 CAN ID
            id = frame.can_id & (format.isExtendedId ? CAN_EFF_MASK : CAN_SFF_MASK);
            // 填充数据
            data.assign(frame.data, frame.data + frame.len);
        }

        /// 转换为 Linux CAN 帧 (标准/扩展)
        can_frame toCanFrame() const {
            // 声明并初始化 Linux CAN 帧
            can_frame frame = {};

            frame.can_id = id;
            // 设置扩展帧标志
            if (format.isExtendedId) frame.can_id |= CAN_EFF_FLAG;
            frame.can_dlc = std::min(static_cast<uint8_t>(8), format.dlc);
            memcpy(frame.data, data.data(), frame.can_dlc);

            return frame;
        }

        /// 转换为 Linux CAN FD 帧
        canfd_frame toCanFdFrame() const {
            canfd_frame frame = {};

            frame.can_id = id;
            // 设置扩展帧标志
            if (format.isExtendedId) frame.can_id |= CAN_EFF_FLAG;
            frame.len = format.dlc;
            frame.flags = 0;
            // 设置扩展帧标志
            if (format.bitRateSwitch) frame.flags |= CANFD_BRS;
            if (format.errorStateInd) frame.flags |= CANFD_ESI;
            memcpy(frame.data, data.data(), format.dlc);

            return frame;
        }

        // 快速创建标准 CAN 帧
        static CANFrame makeStandard(const uint32_t id, const std::vector<uint8_t> &payload) {
            CANFrame f;
            f.format = CANFrameFormat::standard(payload.size());
            f.id = id & CAN_SFF_MASK;
            f.data = payload;
            if (f.data.size() > 8) f.data.resize(8);
            return f;
        }

        // 快速创建扩展 CAN 帧
        static CANFrame makeExtended(const uint32_t id, const std::vector<uint8_t> &payload) {
            CANFrame f;
            f.format = CANFrameFormat::extended(payload.size());
            f.id = id & CAN_EFF_MASK;
            f.data = payload;
            if (f.data.size() > 8) f.data.resize(8);
            return f;
        }

        // 快速创建 CAN FD 帧
        static CANFrame makeCanFd(const uint32_t id, const std::vector<uint8_t> &payload, const bool extId = false,
                                  const bool brs = true) {
            CANFrame f;
            f.format = CANFrameFormat::canFd(extId, payload.size(), brs);
            f.id = id & (extId ? CAN_EFF_MASK : CAN_SFF_MASK);
            f.data = payload;
            if (f.data.size() > 64) f.data.resize(64);
            return f;
        }
    };
}
