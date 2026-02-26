/**
 * @Author: ZZSZSZSZZ
 * @CreateTime: 2026-02-17
 * @Description: Linux CAN 套接字实现
 * @Version: 1.0
 */

#include <linux/can.h>
#include <net/if.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <poll.h>

#include "can/can_socket_impl.hpp"

namespace robot::can {
    Result<void> LinuxCANSocket::open(const std::string &interface, bool enable_can_fd) {
        // 创建 socket
        socket_fd_ = ::socket(PF_CAN, SOCK_RAW, CAN_RAW);
        if (socket_fd_ < 0) {
            return ErrorCode::transport(common::TransportError::SocketCreateFailed, errno);
        }

        // CAN FD 模式设置
        if (enable_can_fd) {
            int enable = 1;
            if (::setsockopt(socket_fd_, SOL_CAN_RAW, CAN_RAW_FD_FRAMES, &enable, sizeof(enable)) < 0) {
                ::close(socket_fd_);
                socket_fd_ = -1;
                return ErrorCode::transport(common::TransportError::SocketCreateFailed, errno);
            }
        }

        // 绑定接口
        ifreq ifr{};
        strncpy(ifr.ifr_name, interface.c_str(), IFNAMSIZ - 1);
        if (::ioctl(socket_fd_, SIOCGIFINDEX, &ifr) < 0) {
            int err = errno;
            ::close(socket_fd_);
            return ErrorCode::transport(common::TransportError::SocketBindFailed, err);
        }

        sockaddr_can addr{};
        addr.can_family = AF_CAN;
        addr.can_ifindex = ifr.ifr_ifindex;

        if (::bind(socket_fd_, reinterpret_cast<sockaddr *>(&addr), sizeof(addr)) < 0) {
            int err = errno;
            ::close(socket_fd_);
            return ErrorCode::transport(common::TransportError::SocketBindFailed, err);
        }

        int loopback = 1; // 1 = enabled (default), 0 = disabled
        setsockopt(socket_fd_, SOL_CAN_RAW, CAN_RAW_LOOPBACK, &loopback, sizeof(loopback));

        // 设置接收自身发送的消息标志
        int recv_own = 1;
        if (::setsockopt(socket_fd_, SOL_CAN_RAW, CAN_RAW_RECV_OWN_MSGS, &recv_own, sizeof(recv_own)) < 0) {
            Logger::error("Failed to set RECV_OWN_MSGS: " + std::string(strerror(errno)));
        }

        interface_ = interface;
        can_fd_enabled_ = enable_can_fd;
        is_open_ = true;
        return ErrorCode::success();
    }

    void LinuxCANSocket::close() {
        if (socket_fd_ >= 0) {
            ::close(socket_fd_);
            socket_fd_ = -1;
            is_open_ = false;
        }
    }

    Result<void> LinuxCANSocket::writeFrame(const CANFrame &frame) {
        if (!is_open_) return ErrorCode::transport(common::TransportError::SocketClosed, 0);

        ssize_t write = 0;
        if (frame.format.type == CANFrameType::CanFd && can_fd_enabled_) {
            auto fdFrame = frame.toCanFdFrame();
            write = ::write(socket_fd_, &fdFrame, sizeof(fdFrame));
            // Logger::debug("write(fd) returned " + std::to_string(write) + ", errno=" + std::to_string(errno));
        } else {
            auto stdFrame = frame.toCanFrame();
            write = ::write(socket_fd_, &stdFrame, sizeof(stdFrame));
            // Logger::debug("write(std) returned " + std::to_string(write) + ", errno=" + std::to_string(errno));
        }

        if (write < 0) {
            return ErrorCode::transport(common::TransportError::WriteFailed, errno);
        }
        return ErrorCode::success();
    }

    Result<CANFrame> LinuxCANSocket::receiveFrame(int timeout_ms) {
        if (!is_open_) return ErrorCode::transport(common::TransportError::SocketClosed, 0);

        // 使用poll等待
        if (timeout_ms >= 0) {
            pollfd pfd = {socket_fd_, POLLIN, 0};
            int ret = ::poll(&pfd, 1, timeout_ms);
            // Logger::debug("poll() returned " + std::to_string(ret) + ", errno=" + std::to_string(errno));
            if (ret < 0) return ErrorCode::transport(common::TransportError::ReadFailed, errno);
            if (ret == 0) return ErrorCode::transport(common::TransportError::Timeout, 0);
        }

        // 尝试读取CAN FD大小
        if (can_fd_enabled_) {
            canfd_frame fdFrame{};
            ssize_t n = ::read(socket_fd_, &fdFrame, sizeof(fdFrame));
            // Logger::debug("read(fd) returned " + std::to_string(n) + ", errno=" + std::to_string(errno));
            if (n == sizeof(fdFrame)) {
                return CANFrame(fdFrame);
            } else if (n == sizeof(can_frame)) {
                // 回退到标准帧
                return CANFrame(*reinterpret_cast<can_frame *>(&fdFrame), CANFrameType::Standard);
            }
        } else {
            can_frame frame{};
            ssize_t n = ::read(socket_fd_, &frame, sizeof(frame));
            // Logger::debug("read(can) returned " + std::to_string(n) + ", errno=" + std::to_string(errno));
            if (n == sizeof(frame)) {
                return CANFrame(frame, CANFrameType::Standard);
            }
        }

        return ErrorCode::transport(common::TransportError::ReadFailed, errno);
    }

    Result<bool> LinuxCANSocket::isDataAvailable(int timeout_us) {
        if (!is_open_) return common::ErrorCode::transport(common::TransportError::SocketClosed, 0);

        pollfd pfd = {socket_fd_, POLLIN, 0};
        int ret = ::poll(&pfd, 1, timeout_us / 1000);
        if (ret < 0) return ErrorCode::transport(common::TransportError::ReadFailed, errno);
        return ret > 0;
    }
}
