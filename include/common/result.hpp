/**
 * @Author: ZZSZSZSZZ
 * @CreateTime: 2026-02-17
 * @Description: 结果类
 * @Version: 1.0
 */

#pragma once

#include "exception.hpp"

namespace robot::common {
    // 结果类 - 有返回值
    template<typename T>
    class Result {
    public:
        // 创建一个成功状态对象
        Result() : is_success_(true), error_(ErrorCode::success()) {
        }

        // 创建一个带返回值的成功状态对象
        Result(const T &value) : is_success_(true), value_(value), error_(ErrorCode::success()) {
        }

        // 创建一个带返回值的成功状态对象
        Result(T &&value) : is_success_(true), value_(std::move(value)), error_(ErrorCode::success()) {
        }

        // 创建一个带错误码的失败状态对象
        Result(const ErrorCode &ec) : is_success_(false), error_(ec) {
        }

        bool isSuccess() const { return is_success_; }
        bool isError() const { return !is_success_; }

        const T &value() const {
            if (!is_success_) throw ExceptionBase(error_, "Accessing error result value");
            return value_;
        }

        T &value() {
            if (!is_success_) throw ExceptionBase(error_, "Accessing error result value");
            return value_;
        }

        const ErrorCode &error() const { return error_; }

        /// 默认值回退
        /// @param default_val 默认值
        /// @return 如果成功，返回成功值；否则返回提供的默认值
        T valueOr(const T &default_val) const {
            return is_success_ ? value_ : default_val;
        }

        // 链式操作
        template<typename Func>
        auto map(Func &&f) -> Result<decltype(f(std::declval<T>()))> {
            using ReturnType = decltype(f(std::declval<T>()));
            if (is_success_) {
                return Result<ReturnType>(f(value_));
            } else {
                return Result<ReturnType>(error_);
            }
        }

        template<typename Func>
        Result<T> mapError(Func &&f) {
            if (!is_success_) {
                f(error_);
            }
            return *this;
        }

    private:
        // 标记是否成功
        bool is_success_;
        // T类型返回值
        T value_;
        // 错误码
        ErrorCode error_;
    };

    // 结果类 - 无返回值
    template<>
    class Result<void> {
    public:
        // 创建一个成功状态对象
        Result() : success_(true), error_(ErrorCode::success()) {
        }

        // 创建一个带错误码的失败状态对象
        Result(const ErrorCode &ec) : success_(ec.isSuccess()), error_(ec) {
        }

        bool isSuccess() const { return success_; }
        bool isError() const { return !success_; }
        const ErrorCode &error() const { return error_; }

    private:
        // 标记是否成功
        bool success_;
        // 错误码
        ErrorCode error_;
    };
}
