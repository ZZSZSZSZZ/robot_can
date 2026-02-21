/**
 * @Author: ZZSZSZSZZ
 * @CreateTime: 2026-02-18
 * @Description: CAN 设备路由器
 * @Version: 1.0
 */

#include "can/can_frame_router.hpp"
#include "can/can_coding.hpp"

namespace robot::can {
    void CANFrameRouter::registerDevice(const std::shared_ptr<CANDevice> &device) {
        std::lock_guard lock(mutex_);
        // 创建设备标识
        const DeviceKey key{device->getReceiveId(), device->isExtendedId()};
        // 将设备注册到设备容器
        devices_[key] = device;

        std::stringstream ss;
        ss << "0x" << std::hex << key.can_id;
        Logger::info("Registered CAN device: " + device->getName() +
                     " (ID: " + ss.str() + (key.extended ? " ext" : " std") + ")");
    }

    void CANFrameRouter::unregisterDevice(const std::shared_ptr<CANDevice> &device) {
        std::lock_guard lock(mutex_);
        // 移除相应设备
        devices_.erase(DeviceKey{device->getReceiveId(), device->isExtendedId()});
    }

    void CANFrameRouter::unregisterDevice(const uint32_t canId, const bool isExtended) {
        std::lock_guard lock(mutex_);
        // 根据 CAN ID 和 拓展帧标识移除相应设备
        devices_.erase(DeviceKey{canId, isExtended});
    }

    Result<void> CANFrameRouter::route(const CANFrame &frame) {
        // 用于在锁外持有设备指针
        std::shared_ptr<CANDevice> device;
        {
            std::lock_guard lock(mutex_);
            const DeviceKey key{frame.id, frame.format.isExtendedId};
            const auto it = devices_.find(key);
            if (it == devices_.end()) {
                Logger::warn("No device registered for ID=0x" + std::to_string(frame.id) +
                             (frame.format.isExtendedId ? " ext" : " std"));
                return ErrorCode::device(common::DeviceError::NotFound, frame.id);
            }
            // 复制 shared_ptr, 增加引用计数, 保证设备在锁外有效
            device = it->second;
        }

        // 验证帧格式
        if (const auto req = device->getFrameRequirement(); !CANFrameDecoder::validate(frame, req)) {
            Logger::warn("Frame validation failed for device " + device->getName() +
                         " (ID=0x" + std::to_string(frame.id) + ")");
            return ErrorCode::protocol(common::ProtocolError::InvalidFrameFormat, 0);
        }

        try {
            device->onFrameReceived(frame);
            return ErrorCode::success();
        } catch (const std::exception &e) {
            Logger::error("Exception in device callback: " + std::string(e.what()));
            return ErrorCode::protocol(common::ProtocolError::DecodingFailed, 0);
        }
    }

    CANDeviceFrameRequirement CANFrameRouter::getRequirement(uint32_t canId, bool isExtended) const {
        std::lock_guard lock(mutex_);
        auto it = devices_.find(DeviceKey{canId, isExtended});
        if (it != devices_.end()) {
            return it->second->getFrameRequirement();
        }
        // 若设备未注册，返回一个默认需求
        return {CANFrameType::Standard, false, 8, false}; // 默认
    }

    size_t CANFrameRouter::deviceCount() const {
        std::lock_guard lock(mutex_);
        return devices_.size();
    }
}
