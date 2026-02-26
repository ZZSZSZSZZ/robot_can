/**
* @Author: ZZSZSZSZZ
 * @CreateTime: 2026-02-20
 * @Description:
 * @Version: 1.0
 */

#include <iostream>
#include <chrono>
#include <thread>
#include <vector>
#include <condition_variable>
#include <map>
#include <pthread.h>          // 用于线程优先级
#include <sys/socket.h>        // 用于套接字选项

#include "can/can_socket_impl.hpp"
#include "can/can_frame_router.hpp"
#include "can/can_receiver.hpp"
#include "can/can_coding.hpp"
#include "common/logger.hpp"

using namespace robot::can;
// using namespace robot::motor;

// 日志回调（简单打印到 stdout）
void logCallback(const std::string& level, const std::string& msg) {
    static std::mutex cout_mutex;
    std::lock_guard lock(cout_mutex);
    std::cout << "[" << level << "] " << msg << std::endl;
}

// ==================== 回显设备 ====================
class EchoDevice : public CANDevice {
public:
    EchoDevice(uint32_t recv_id, uint32_t send_id, bool ext, std::shared_ptr<CANSocket> socket)
        : recv_id_(recv_id), send_id_(send_id), ext_(ext), socket_(socket) {}

    void onFrameReceived(const CANFrame& frame) override {
        // 立即将收到的数据原样发回（使用响应 ID）
        CANFrame response = ext_ ? CANFrame::makeExtended(send_id_, frame.data)
                                  : CANFrame::makeStandard(send_id_, frame.data);
        auto res = socket_->writeFrame(response);
        if (res.isError()) {
            Logger::error("EchoDevice write failed: " + res.error().toString());
        } else {
            // Logger::debug("EchoDevice sent response for seq " + std::to_string(frame.data[0]));
        }
    }

    bool isExtendedId() const override { return ext_; }
    uint32_t getReceiveId() const override { return recv_id_; }
    std::string getName() const override { return "EchoDevice"; }

    CANDeviceFrameRequirement getFrameRequirement() const override {
        return {ext_ ? CANFrameType::Extended : CANFrameType::Standard,
                ext_, 8, false};
    }

private:
    uint32_t recv_id_, send_id_;
    bool ext_;
    std::shared_ptr<CANSocket> socket_;
};

// ==================== 测试设备（支持序列号匹配） ====================
class LatencyTestDevice : public CANDevice {
public:
    LatencyTestDevice(uint32_t expect_recv_id, bool ext)
        : expect_recv_id_(expect_recv_id), ext_(ext) {}

    // 注册一个待确认的帧（由主线程调用）
    void expectResponse(uint8_t seq, std::chrono::steady_clock::time_point send_time) {
        std::lock_guard lock(mutex_);
        PendingInfo info;
        info.send_time = send_time;
        info.received = false;
        pending_[seq] = info;
    }

    // 等待特定序列号的响应，返回 RTT（微秒），超时返回 -1
    int64_t waitForResponse(uint8_t seq, int timeout_ms) {
        auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(timeout_ms);
        std::unique_lock lock(mutex_);
        while (true) {
            auto it = pending_.find(seq);
            if (it != pending_.end() && it->second.received) {
                // 已收到响应，计算 RTT
                auto recv_time = it->second.recv_time;
                auto send_time = it->second.send_time;
                pending_.erase(it);
                return std::chrono::duration_cast<std::chrono::microseconds>(recv_time - send_time).count();
            }
            if (std::chrono::steady_clock::now() >= deadline) {
                // 超时，删除待确认项并返回 -1
                pending_.erase(seq);
                return -1;
            }
            cv_.wait_until(lock, deadline);
        }
    }

    void onFrameReceived(const CANFrame& frame) override {
        if (frame.id == expect_recv_id_ && frame.format.isExtendedId == ext_) {
            if (frame.data.empty()) return;
            uint8_t seq = frame.data[0];  // 序列号存储在第一个字节
            std::lock_guard lock(mutex_);
            auto it = pending_.find(seq);
            if (it != pending_.end()) {
                it->second.received = true;
                it->second.recv_time = std::chrono::steady_clock::now();
                cv_.notify_all();  // 唤醒所有等待（实际只有对应 seq 的等待者会检查）
                // Logger::debug("LatencyTestDevice received response for seq " + std::to_string(seq));
            } else {
                Logger::debug("LatencyTestDevice received unexpected seq " + std::to_string(seq));
            }
        }
    }

    bool isExtendedId() const override { return ext_; }
    uint32_t getReceiveId() const override { return expect_recv_id_; }
    std::string getName() const override { return "LatencyTestDevice"; }

    CANDeviceFrameRequirement getFrameRequirement() const override {
        return {ext_ ? CANFrameType::Extended : CANFrameType::Standard,
                ext_, 8, false};
    }

private:
    struct PendingInfo {
        std::chrono::steady_clock::time_point send_time;
        std::chrono::steady_clock::time_point recv_time;
        bool received;
    };

    uint32_t expect_recv_id_;
    bool ext_;
    std::mutex mutex_;
    std::condition_variable cv_;
    std::map<uint8_t, PendingInfo> pending_;
};

// ==================== 主程序 ====================
int main() {
    Logger::setCallback(logCallback);

    // --- 配置参数 ---
    const std::string CAN_INTERFACE = "can0";
    const bool ENABLE_CAN_FD = false;          // 是否启用 CAN FD
    const uint32_t REQ_ID = 0x100;              // 请求帧 ID
    const uint32_t RESP_ID = 0x200;             // 响应帧 ID
    const bool USE_EXT_ID = false;               // 是否使用扩展 ID
    const int TEST_COUNT = 10000;                 // 测试次数
    const int PAYLOAD_SIZE = 4;                   // 数据载荷字节数（至少1字节用于序列号）
    const int SEND_INTERVAL_MS = 0;               // 发送间隔（毫秒）
    const int RESPONSE_TIMEOUT_MS = 100;          // 等待响应超时（毫秒）
    const bool SET_REALTIME_PRIORITY = true;      // 是否为接收线程设置实时优先级
    const int RX_BUFFER_SIZE = 65536;              // 接收缓冲区大小（字节）
    const int TX_BUFFER_SIZE = 65536;              // 发送缓冲区大小（字节）

    // --- 打开套接字 ---
    auto socket = std::make_shared<LinuxCANSocket>();
    Result<void> openRes = socket->open(CAN_INTERFACE, ENABLE_CAN_FD);
    if (openRes.isError()) {
        Logger::error("Failed to open CAN socket: " + openRes.error().toString());
        return -1;
    }
    Logger::info("CAN socket opened on " + socket->getInterfaceName());

    // --- 设置套接字缓冲区大小（可选）---
    if (RX_BUFFER_SIZE > 0) {
        int rcvbuf = RX_BUFFER_SIZE;
        if (setsockopt(socket->getSocket(), SOL_SOCKET, SO_RCVBUF, &rcvbuf, sizeof(rcvbuf)) == 0) {
            Logger::info("RX buffer set to " + std::to_string(RX_BUFFER_SIZE) + " bytes");
        } else {
            Logger::warn("Failed to set RX buffer size");
        }
    }
    if (TX_BUFFER_SIZE > 0) {
        int sndbuf = TX_BUFFER_SIZE;
        if (setsockopt(socket->getSocket(), SOL_SOCKET, SO_SNDBUF, &sndbuf, sizeof(sndbuf)) == 0) {
            Logger::info("TX buffer set to " + std::to_string(TX_BUFFER_SIZE) + " bytes");
        } else {
            Logger::warn("Failed to set TX buffer size");
        }
    }

    // --- 创建路由器 ---
    auto router = std::make_shared<CANFrameRouter>();

    // --- 创建设备 ---
    auto echoDev = std::make_shared<EchoDevice>(REQ_ID, RESP_ID, USE_EXT_ID, socket);
    auto testDev = std::make_shared<LatencyTestDevice>(RESP_ID, USE_EXT_ID);

    router->registerDevice(echoDev);
    router->registerDevice(testDev);
    Logger::info("Devices registered.");

    // --- 启动接收线程 ---
    auto receiver = std::make_shared<CANReceiver>(socket, router);
    if (receiver->start().isError()) {
        Logger::error("Failed to start receiver");
        return -1;
    }

    // --- 设置接收线程实时优先级 ---
    if (SET_REALTIME_PRIORITY) {
        pthread_t native = receiver->getThread().native_handle();
        sched_param param;
        param.sched_priority = 50;  // 优先级范围通常 1-99（需 root 权限）
        if (pthread_setschedparam(native, SCHED_FIFO, &param) == 0) {
            Logger::info("Receiver thread set to real-time priority (FIFO, prio=50)");
        } else {
            Logger::warn("Failed to set real-time priority (may need root)");
        }
    }

    // --- 延迟测试 ---
    std::vector<uint64_t> rtts;
    rtts.reserve(TEST_COUNT);

    Logger::info("Starting latency test, sending " + std::to_string(TEST_COUNT) + " frames...");

    for (int i = 0; i < TEST_COUNT; ++i) {
        uint8_t seq = static_cast<uint8_t>(i & 0xFF);  // 序列号（循环使用 0-255）

        // 构造数据：第一个字节为序列号，其余为可选的测试数据
        std::vector<uint8_t> tx_data(PAYLOAD_SIZE, 0);
        tx_data[0] = seq;
        for (size_t j = 1; j < tx_data.size(); ++j) {
            tx_data[j] = i >> (8 * j) & 0xFF;  // 填充一些可变数据
        }

        // 记录发送时间并通知测试设备
        auto send_time = std::chrono::steady_clock::now();
        testDev->expectResponse(seq, send_time);

        // 发送请求帧
        CANFrame request = USE_EXT_ID ? CANFrame::makeExtended(REQ_ID, tx_data)
                                       : CANFrame::makeStandard(REQ_ID, tx_data);
        Result<void> writeRes = socket->writeFrame(request);
        if (writeRes.isError()) {
            Logger::error("Write failed: " + writeRes.error().toString());
            continue;
        }

        // 等待响应
        int64_t rtt_us = testDev->waitForResponse(seq, RESPONSE_TIMEOUT_MS);
        if (rtt_us >= 0) {
            rtts.push_back(rtt_us);
            // Logger::debug("Frame " + std::to_string(i) + " (seq " + std::to_string(seq) +
            //               ") RTT = " + std::to_string(rtt_us) + " us");
        } else {
            Logger::warn("Timeout for frame " + std::to_string(i) + " (seq " + std::to_string(seq) + ")");
        }

        // 间隔
        std::this_thread::sleep_for(std::chrono::milliseconds(SEND_INTERVAL_MS));
    }

    // --- 统计结果 ---
    if (rtts.empty()) {
        Logger::error("No successful responses received.");
    } else {
        uint64_t sum = 0, min = rtts[0], max = rtts[0];
        for (auto us : rtts) {
            sum += us;
            if (us < min) min = us;
            if (us > max) max = us;
        }
        double avg = static_cast<double>(sum) / rtts.size();

        Logger::info("========== Latency Test Results ==========");
        Logger::info("Successful frames: " + std::to_string(rtts.size()) + "/" + std::to_string(TEST_COUNT));
        Logger::info("Min RTT: " + std::to_string(min) + " us");
        Logger::info("Max RTT: " + std::to_string(max) + " us");
        Logger::info("Avg RTT: " + std::to_string(avg) + " us");
        Logger::info("==========================================");
    }

    // --- 清理 ---
    receiver->stop();
    socket->close();
    Logger::info("Test finished.");
    return 0;
}