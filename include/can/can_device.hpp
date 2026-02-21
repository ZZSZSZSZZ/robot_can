/**
 * @Author: ZZSZSZSZZ
 * @CreateTime: 2026-02-17
 * @Description: CAN 设备抽象基类
 * @Version: 1.0
 */

#pragma once

#include "can_frame.hpp"
#include "common/common.hpp"

namespace robot::can {
    // 设备 CAN 帧格式需求
    // 描述一个 CAN 设备对帧格式的需求
    struct CANDeviceFrameRequirement {
        CANFrameType preferredType; // 首选帧类型
        bool requireExtendedId; // 是否需要扩展 ID
        uint8_t maxDataLength; // 最大数据长度需求
        bool requireCanFd; // 是否必须使用CAN FD

        // 检查是否兼容给定格式
        bool isCompatible(const FrameFormat& fmt) const {
            if (requireCanFd && fmt.type != CANFrameType::CanFd) return false;
            if (requireExtendedId && !fmt.isExtendedId) return false;
            if (fmt.dlc > maxDataLength) return false;
            return true;
        }

        // 获取最佳格式
        FrameFormat getOptimalFormat() const {
            if (requireCanFd) {
                return FrameFormat::canFd(requireExtendedId, maxDataLength);
            }
            if (requireExtendedId) {
                return FrameFormat::extended(maxDataLength);
            }
            return FrameFormat::standard(maxDataLength);
        }
    };

    // CAN 设备抽象类
    // 声明 CAN 设备的回调接口及标识方法
    class CANDevice {
    public:
        virtual ~CANDevice() = default;

        /// 当设备对应的接收 ID 的帧到达时，此函数被调用
        /// @param frame CAN 帧
        virtual void onFrameReceived(const CANFrame &frame) = 0;

        /// 返回是否为拓展帧格式
        virtual bool isExtendedId() const = 0;

        /// 返回该设备的帧格式需求
        virtual CANDeviceFrameRequirement getFrameRequirement() const = 0;

        /// 返回设备 CAN ID
        virtual uint32_t getReceiveId() const = 0;

        /// 返回设备的名称
        virtual std::string getName() const = 0;
    };
}
