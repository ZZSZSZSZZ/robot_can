/**
 * @Author: ZZSZSZSZZ
 * @CreateTime: 2026-02-17
 * @Description: 日志类
 * @Version: 1.0
 */

#pragma once

#include <functional>
#include <iostream>
#include <mutex>

namespace robot::common {
    // 日志回调
    using LogCallback = std::function<void(const std::string &level, const std::string &msg)>;

    // 日志类
    class Logger {
    public:
#if LOG_ENABLED
        /// 设置回调函数
        /// @param cb 回调函数
        static void setCallback(LogCallback cb) { callback_ = cb; }
#else
        static void setCallback(LogCallback cb) {
        }
#endif

#if LOG_ENABLED
        /// Debug 日志
        /// @param msg 消息
        static void debug(const std::string &msg) { log("DEBUG", msg); }

        /// Info 日志
        /// @param msg 消息
        static void info(const std::string &msg) { log("INFO", msg); }

        /// Warn 日志
        /// @param msg 消息
        static void warn(const std::string &msg) { log("WARN", msg); }

        /// Error 日志
        /// @param msg 消息
        static void error(const std::string &msg) { log("ERROR", msg); }
#else
        static void debug(const std::string &msg) {
        }

        static void info(const std::string &msg) {
        }

        static void warn(const std::string &msg) {
        }

        static void error(const std::string &msg) {
        }
#endif

    private:
#if LOG_ENABLED
        /// 日志
        /// 如果已设置回调, 则将级别和消息转发给回调函数; 否则将消息输出到标准错误流 std::cerr（带方括号级别前缀）
        /// @param level 日志等级
        /// @param msg 消息
        static void log(const std::string &level, const std::string &msg) {
            std::lock_guard lock(mutex_);
            if (callback_) {
                callback_(level, msg);
            } else {
                std::cerr << "[ " << level << " ] " << msg << std::endl;
            }
        }

        static inline LogCallback callback_;
        static inline std::mutex mutex_;
#else
#endif
    };
}
