/**
 * @Author: ZZSZSZSZZ
 * @CreateTime: 2026-02-18
 * @Description: CAN 帧路由器
 * @Version: 1.0
 */

#pragma once

#include <memory>
#include <mutex>

#include "can/can_device.hpp"

namespace robot::can {
    // CAN 帧路由器
    // 根据 CAN ID 将接收到的帧分发给已注册的设备
    class CANFrameRouter {
    public:
        /// 注册设备
        /// @param device CANDevice 对象
        void registerDevice(const std::shared_ptr<CANDevice> &device);

        /// 移除设备
        /// @param device CANDevice 对象
        void unregisterDevice(const std::shared_ptr<CANDevice> &device);

        /// 移除设备
        /// @param canId CAN ID
        /// @param isExtended 是否为拓展帧
        void unregisterDevice(uint32_t canId, bool isExtended);

        /// 将接收到的 CAN 帧路由到对应的设备
        /// @param frame CAN 帧
        /// @return 若找到设备，调用其 onFrameReceived，并返回成功
        Result<void> route(const CANFrame &frame);

        /// 获取设备的帧格式需求
        CANDeviceFrameRequirement getRequirement(uint32_t canId, bool isExtended) const;

        /// 返回当前注册设备数量
        size_t deviceCount() const;

    private:
        // 封装设备标识
        struct DeviceKey {
            uint32_t can_id; // CAN ID
            bool extended; // 是否为扩展帧
            bool operator==(const DeviceKey &o) const { return can_id == o.can_id && extended == o.extended; }
        };

        // 为 DeviceKey 计算哈希值
        struct DeviceKeyHash {
            size_t operator()(const DeviceKey &k) const {
                return std::hash<uint32_t>()(k.can_id) ^ std::hash<bool>()(k.extended) << 31;
            }
        };

        mutable std::mutex mutex_; // 线程锁
        std::unordered_map<DeviceKey, std::shared_ptr<CANDevice>, DeviceKeyHash> devices_; // 设备查找表
    };
}
