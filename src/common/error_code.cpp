/**
 * @Author: ZZSZSZSZZ
 * @CreateTime: 2026-02-17
 * @Description: 错误码
 * @Version: 1.0
 */

#include <sstream>

#include "common/error_code.hpp"

namespace robot::common {
    std::string ErrorCode::categoryName() const {
        switch (category) {
            case ErrorCategory::Success: return "Success";
            case ErrorCategory::System: return "System";
            case ErrorCategory::Transport: return "Transport";
            case ErrorCategory::Protocol: return "Protocol";
            case ErrorCategory::Device: return "Device";
            case ErrorCategory::Application: return "Application";
            case ErrorCategory::Configuration: return "Configuration";
            default: return "Unknown";
        }
    }

    std::string ErrorCode::toString() const {
        std::stringstream ss;
        ss << categoryName() << "::";

        switch (category) {
            case ErrorCategory::System:
                switch (static_cast<SystemError>(code)) {
                    case SystemError::ResourceAllocationFailed: ss << "ResourceAllocationFailed";
                        break;
                    case SystemError::ThreadCreationFailed: ss << "ThreadCreationFailed";
                        break;
                    case SystemError::PermissionDenied: ss << "PermissionDenied";
                        break;
                    case SystemError::InvalidArgument: ss << "InvalidArgument";
                        break;
                    case SystemError::NotImplemented: ss << "NotImplemented";
                        break;
                    default: ss << "Unknown(" << code << ")";
                        break;
                }
                break;
            case ErrorCategory::Transport:
                switch (static_cast<TransportError>(code)) {
                    case TransportError::SocketCreateFailed: ss << "SocketCreateFailed";
                        break;
                    case TransportError::SocketBindFailed: ss << "SocketBindFailed";
                        break;
                    case TransportError::SocketClosed: ss << "SocketClosed";
                        break;
                    case TransportError::WriteFailed: ss << "WriteFailed";
                        break;
                    case TransportError::ReadFailed: ss << "ReadFailed";
                        break;
                    case TransportError::Timeout: ss << "Timeout";
                        break;
                    case TransportError::BusOff: ss << "BusOff";
                        break;
                    default: ss << "Unknown(" << code << ")";
                        break;
                }
                break;
            case ErrorCategory::Protocol:
                switch (static_cast<ProtocolError>(code)) {
                    case ProtocolError::InvalidFrameFormat: ss << "InvalidFrameFormat";
                        break;
                    case ProtocolError::ChecksumMismatch: ss << "ChecksumMismatch";
                        break;
                    case ProtocolError::InvalidMessageId: ss << "InvalidMessageId";
                        break;
                    case ProtocolError::DecodingFailed: ss << "DecodingFailed";
                        break;
                    // 添加其他遗漏的枚举
                    case ProtocolError::SequenceError: ss << "SequenceError";
                        break;
                    case ProtocolError::BufferUnderrun: ss << "BufferUnderrun";
                        break;
                    case ProtocolError::EncodingFailed: ss << "EncodingFailed";
                        break;
                    case ProtocolError::VersionMismatch: ss << "VersionMismatch";
                        break;
                    case ProtocolError::UnsupportedCommand: ss << "UnsupportedCommand";
                        break;
                    default: ss << "Unknown(" << code << ")";
                        break;
                }
                break;
            case ErrorCategory::Device:
                switch (static_cast<DeviceError>(code)) {
                    case DeviceError::NotFound: ss << "NotFound";
                        break;
                    case DeviceError::NotInitialized: ss << "NotInitialized";
                        break;
                    case DeviceError::NotEnabled: ss << "NotEnabled";
                        break;
                    case DeviceError::AlreadyEnabled: ss << "AlreadyEnabled";
                        break;
                    case DeviceError::ModeNotSupported: ss << "ModeNotSupported";
                        break;
                    case DeviceError::CapabilityNotSupported: ss << "CapabilityNotSupported";
                        break;
                    case DeviceError::ParameterOutOfRange: ss << "ParameterOutOfRange";
                        break;
                    case DeviceError::HardwareFault: ss << "HardwareFault";
                        break;
                    case DeviceError::OverTemperature: ss << "OverTemperature";
                        break;
                    case DeviceError::OverCurrent: ss << "OverCurrent";
                        break;
                    case DeviceError::PositionLimitExceeded: ss << "PositionLimitExceeded";
                        break;
                    case DeviceError::VelocityLimitExceeded: ss << "VelocityLimitExceeded";
                        break;
                    case DeviceError::TorqueLimitExceeded: ss << "TorqueLimitExceeded";
                        break;
                    case DeviceError::CommunicationLost: ss << "CommunicationLost";
                        break;
                    case DeviceError::CalibrationError: ss << "CalibrationError";
                        break;
                    case DeviceError::EncoderError: ss << "EncoderError";
                        break;
                    default: ss << "Unknown(" << code << ")";
                        break;
                }
                break;
            case ErrorCategory::Application:
                switch (static_cast<ApplicationError>(code)) {
                    case ApplicationError::InvalidConfiguration: ss << "InvalidConfiguration";
                        break;
                    case ApplicationError::ComponentNotFound: ss << "ComponentNotFound";
                        break;
                    case ApplicationError::StateMismatch: ss << "StateMismatch";
                        break;
                    case ApplicationError::OperationNotAllowed: ss << "OperationNotAllowed";
                        break;
                    case ApplicationError::DependencyMissing: ss << "DependencyMissing";
                        break;
                    case ApplicationError::InitializationFailed: ss << "InitializationFailed";
                        break;
                    case ApplicationError::ShutdownFailed: ss << "ShutdownFailed";
                        break;
                    default: ss << "Unknown(" << code << ")";
                        break;
                }
                break;
            case ErrorCategory::Configuration:
                switch (static_cast<ConfigError>(code)) {
                    case ConfigError::FileNotFound: ss << "FileNotFound";
                        break;
                    case ConfigError::ParseError: ss << "ParseError";
                        break;
                    case ConfigError::MissingRequiredField: ss << "MissingRequiredField";
                        break;
                    case ConfigError::InvalidValue: ss << "InvalidValue";
                        break;
                    case ConfigError::TypeMismatch: ss << "TypeMismatch";
                        break;
                    case ConfigError::DuplicateId: ss << "DuplicateId";
                        break;
                    default: ss << "Unknown(" << code << ")";
                        break;
                }
                break;
            case ErrorCategory::Success:
                ss << "Success";
                break;
            default:
                ss << "Unknown(" << static_cast<int>(category) << "," << code << ")";
                break;
        }

        if (sub_code != 0) {
            ss << " (sub_code=" << sub_code << ")";
        }
        return ss.str();
    }
}