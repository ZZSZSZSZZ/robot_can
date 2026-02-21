/**
 * @Author: ZZSZSZSZZ
 * @CreateTime: 2026-02-17
 * @Description: 异常类
 * @Version: 1.0
 */

#pragma once

#include <chrono>
#include <sstream>
#include <stdexcept>
#include <vector>

#include "error_code.hpp"

namespace robot::common {
    // 异常基类
    class ExceptionBase : public std::runtime_error {
    public:
        /// 异常基类构造
        /// @param ec 错误码
        /// @param msg 异常消息
        /// @param source 异常来源
        ExceptionBase(const ErrorCode &ec, const std::string &msg, const std::string &source = "")
            : std::runtime_error(msg),
              error_code_(ec),
              source_(source),
              timestamp_(std::chrono::steady_clock::now()) {
        }

        const ErrorCode &errorCode() const { return error_code_; }
        const std::string &source() const { return source_; }
        auto timestamp() const { return timestamp_; }

        // 获取完整错误信息
        // 例如：[System:5:2] Failed to open can socket (at 来源)
        virtual std::string detailedMessage() const {
            std::stringstream ss;
            ss << "[" << error_code_.categoryName()
                    << ":" << error_code_.code
                    << (error_code_.sub_code ? ":" + std::to_string(error_code_.sub_code) : "")
                    << "] " << what();
            if (!source_.empty()) {
                ss << " (at " << source_ << ")";
            }
            return ss.str();
        }

    protected:
        ErrorCode error_code_;
        std::string source_;
        std::chrono::steady_clock::time_point timestamp_;
    };

    // 内存、线程、权限等系统异常
    class SystemException : public ExceptionBase {
    public:
        SystemException(SystemError err, const std::string &msg, const std::string &source = "")
            : ExceptionBase(ErrorCode::system(err), msg, source) {
        }
    };

    // CAN Socket通信异常
    class TransportException : public ExceptionBase {
    public:
        TransportException(TransportError err, const std::string &msg,
                           const std::string &source = "", int errno_val = 0)
            : ExceptionBase(ErrorCode::transport(err, errno_val), msg, source), errno_(errno_val) {
        }

        int getErrno() const { return errno_; }

    private:
        int errno_;
    };

    // 帧格式、编解码异常
    class ProtocolException : public ExceptionBase {
    public:
        ProtocolException(ProtocolError err, const std::string &msg,
                          const std::string &source = "",
                          const std::vector<uint8_t> &raw_data = {})
            : ExceptionBase(ErrorCode::protocol(err), msg, source),
              raw_data_(raw_data) {
        }

        const std::vector<uint8_t> &getRawData() const { return raw_data_; }

    private:
        std::vector<uint8_t> raw_data_;
    };

    // 电机状态、能力、硬件异常 (最重要，包含设备标识)
    class DeviceException : public ExceptionBase {
    public:
        DeviceException(DeviceError err, const std::string &msg,
                        const std::string &device_name = "",
                        const std::string &device_type = "",
                        uint32_t device_id = 0)
            : ExceptionBase(ErrorCode::device(err), msg, device_name),
              device_name_(device_name),
              device_type_(device_type),
              device_id_(device_id) {
        }

        const std::string &deviceName() const { return device_name_; }
        const std::string &deviceType() const { return device_type_; }
        uint32_t deviceId() const { return device_id_; }

        std::string detailedMessage() const override {
            std::stringstream ss;
            ss << ExceptionBase::detailedMessage();
            if (!device_type_.empty()) {
                ss << " [Type: " << device_type_;
                if (device_id_ != 0) ss << ", ID: 0x" << std::hex << device_id_;
                ss << "]";
            }
            return ss.str();
        }

    private:
        std::string device_name_;
        std::string device_type_;
        uint32_t device_id_;
    };

    // 业务逻辑、组件异常
    class ApplicationException : public ExceptionBase {
    public:
        ApplicationException(ApplicationError err, const std::string &msg,
                             const std::string &component = "")
            : ExceptionBase(ErrorCode::application(err), msg, component),
              component_(component) {
        }

        const std::string &component() const { return component_; }

    private:
        std::string component_;
    };

    // 配置异常
    class ConfigException : public ExceptionBase {
    public:
        ConfigException(ConfigError err, const std::string &msg,
                        const std::string &file = "", int line = 0)
            : ExceptionBase(ErrorCode::config(err), msg, file),
              file_(file), line_(line) {
        }

        const std::string &file() const { return file_; }
        int lineNumber() const { return line_; }

    private:
        std::string file_;
        int line_;
    };
}
