/**
 * @Author: ZZSZSZSZZ
 * @CreateTime: 2026-02-17
 * @Description: 错误码
 * @Version: 1.0
 */

#pragma once

#include <cstdint>
#include <string>

namespace robot::common {
    // 主错误
    enum class ErrorCategory : uint8_t {
        Success = 0, // 成功
        System, // 内存、线程、权限等系统错误
        Transport, // CAN Socket通信错误
        Protocol, // 帧格式、编解码错误
        Device, // 电机状态、能力、硬件错误
        Application, // 业务逻辑、组件错误
        Configuration // 配置错误
    };

    // 内存、线程、权限等系统错误
    enum class SystemError : uint16_t {
        None = 0,
        ResourceAllocationFailed, // 内存/资源分配失败
        ThreadCreationFailed, // 线程创建失败
        PermissionDenied, // 权限不足
        InvalidArgument, // 无效参数
        NotImplemented, // 功能未实现
        Unknown
    };

    // CAN Socket通信错误
    enum class TransportError : uint16_t {
        None = 0,
        SocketCreateFailed, // Socket创建失败
        SocketBindFailed, // 绑定失败
        SocketClosed, // 连接已关闭
        WriteFailed, // 写入失败
        ReadFailed, // 读取失败
        Timeout, // 超时
        BufferOverflow, // 缓冲区溢出
        BusOff, // CAN总线关闭
        FrameTooLong, // 帧过长
        InvalidInterface // 无效接口
    };

    // 帧格式、编解码错误
    enum class ProtocolError : uint16_t {
        None = 0,
        InvalidFrameFormat, // 无效帧格式
        ChecksumMismatch, // 校验失败
        InvalidMessageId, // 无效消息ID
        SequenceError, // 序列错误
        BufferUnderrun, // 数据不足
        EncodingFailed, // 编码失败
        DecodingFailed, // 解码失败
        VersionMismatch, // 协议版本不匹配
        UnsupportedCommand // 不支持的命令
    };

    // 电机状态、能力、硬件错误
    enum class DeviceError : uint16_t {
        None = 0,
        NotFound, // 设备未找到
        NotInitialized, // 未初始化
        NotEnabled, // 未使能
        AlreadyEnabled, // 已使能
        ModeNotSupported, // 模式不支持
        CapabilityNotSupported, // 能力不支持
        ParameterOutOfRange, // 参数越界
        HardwareFault, // 硬件故障
        OverTemperature, // 过温
        OverCurrent, // 过流
        PositionLimitExceeded, // 位置超限
        VelocityLimitExceeded, // 速度超限
        TorqueLimitExceeded, // 力矩超限
        CommunicationLost, // 通信丢失
        CalibrationError, // 校准错误
        EncoderError // 编码器错误
    };

    // 业务逻辑、组件错误
    enum class ApplicationError : uint16_t {
        None = 0,
        InvalidConfiguration, // 无效配置
        ComponentNotFound, // 组件未找到
        StateMismatch, // 状态不匹配
        OperationNotAllowed, // 操作不允许
        DependencyMissing, // 依赖缺失
        InitializationFailed, // 初始化失败
        ShutdownFailed // 关闭失败
    };

    // 配置错误
    enum class ConfigError : uint16_t {
        None = 0,
        FileNotFound, // 配置文件不存在
        ParseError, // 解析错误
        MissingRequiredField, // 缺少必填项
        InvalidValue, // 无效值
        TypeMismatch, // 类型不匹配
        DuplicateId // ID重复
    };

    // 错误码
    struct ErrorCode {
        ErrorCategory category; // 主错误码
        uint16_t code; // 具体错误值
        uint16_t sub_code; // 子错误/扩展码

        // 默认构造
        ErrorCode() : category(ErrorCategory::Success), code(0), sub_code(0) {
        }

        /// 错误码
        /// @param cat 主错误码
        /// @param c 具体错误值
        /// @param sub 子错误/扩展码 (默认为0)
        ErrorCode(ErrorCategory cat, uint16_t c, uint16_t sub = 0)
            : category(cat), code(c), sub_code(sub) {
        }

        bool isSuccess() const { return category == ErrorCategory::Success; }
        bool isError() const { return !isSuccess(); }

        // 快速创建辅助函数
        static ErrorCode success() {
            return {};
        }

        static ErrorCode system(SystemError e, uint16_t sub = 0) {
            return {ErrorCategory::System, static_cast<uint16_t>(e), sub};
        }

        static ErrorCode transport(TransportError e, uint16_t sub = 0) {
            return {ErrorCategory::Transport, static_cast<uint16_t>(e), sub};
        }

        static ErrorCode protocol(ProtocolError e, uint16_t sub = 0) {
            return {ErrorCategory::Protocol, static_cast<uint16_t>(e), sub};
        }

        static ErrorCode device(DeviceError e, uint16_t sub = 0) {
            return {ErrorCategory::Device, static_cast<uint16_t>(e), sub};
        }

        static ErrorCode application(ApplicationError e, uint16_t sub = 0) {
            return {ErrorCategory::Application, static_cast<uint16_t>(e), sub};
        }

        static ErrorCode config(ConfigError e, uint16_t sub = 0) {
            return {ErrorCategory::Configuration, static_cast<uint16_t>(e), sub};
        }

        // 转换为可读字符串
        std::string toString() const;

        // 获取错误类别名
        std::string categoryName() const;
    };

    // 错误码比较
    inline bool operator==(const ErrorCode &lhs, const ErrorCode &rhs) {
        return lhs.category == rhs.category && lhs.code == rhs.code;
    }

    inline bool operator!=(const ErrorCode &lhs, const ErrorCode &rhs) {
        return !(lhs == rhs);
    }
}
