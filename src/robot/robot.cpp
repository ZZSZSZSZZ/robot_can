/**
 * @Author: ZZSZSZSZZ
 * @CreateTime: 2026-03-09
 * @Description: Robot 类实现
 * @Version: 1.0
 */

#include "robot/robot.hpp"
#include "can/can_socket_impl.hpp"
#include "motor/drivers/eyou/eyou_factory.hpp"
#include "common/logger.hpp"

namespace robot {
    using common::ErrorCode;
    using common::ApplicationError;
    using common::SystemError;
    using common::TransportError;

    // ==================== 构造函数 ====================

    Robot::Robot(const std::string &can_interface, const Options &options) : options_(options) {
        can_interfaces_.push_back(can_interface);
    }

    Robot::Robot(const std::vector<std::string> &can_interfaces, const Options &options)
        : can_interfaces_(can_interfaces), options_(options), multi_interface_mode_(true) {
        if (can_interfaces_.empty()) {
            throw std::invalid_argument("At least one CAN interface must be provided");
        }
    }

    Robot::~Robot() {
        if (initialized_) {
            shutdown();
        }
    }

    // ==================== 初始化 ====================

    Result<void> Robot::initialize() {
        if (initialized_) {
            return ErrorCode::application(ApplicationError::OperationNotAllowed);
        }

        Logger::info("Initializing Robot...");

        // 注册电机类型
        motor::eyou::EYOUFactory::registerMotorType();

        // 初始化 CAN Sockets
        auto result = initializeCANSockets();
        if (result.isError()) {
            Logger::error("Failed to initialize CAN sockets");
            return result;
        }

        // 初始化 Router
        result = initializeRouter();
        if (result.isError()) {
            Logger::error("Failed to initialize router");
            return result;
        }

        // 初始化 Receiver
        result = initializeReceiver();
        if (result.isError()) {
            Logger::error("Failed to initialize receiver");
            return result;
        }

        // 初始化 Motor Manager
        result = initializeMotorManager();
        if (result.isError()) {
            Logger::error("Failed to initialize motor");
            return result;
        }

        initialized_ = true;
        Logger::info("Robot initialized successfully");
        return ErrorCode();
    }

    void Robot::shutdown() {
        if (!initialized_) {
            return;
        }

        Logger::info("Shutting down Robot...");

        // 停止所有组件
        for (auto &[name, component]: arm_components_) {
            component->shutdown();
        }
        arm_components_.clear();

        // 停止电机管理器
        if (motor_manager_) {
            motor_manager_->stop();
        }

        // 停止接收器
        if (receiver_) {
            receiver_->stop();
        }

        // 清理 sockets
        for (auto &[name, socket]: sockets_) {
            if (socket) {
                socket->close();
            }
        }
        sockets_.clear();
        default_socket_.reset();

        // 清理其他组件
        motor_manager_.reset();
        receiver_.reset();
        router_.reset();

        initialized_ = false;
        Logger::info("Robot shutdown complete");
    }

    Result<void> Robot::initializeCANSockets() {
        for (const auto &interface: can_interfaces_) {
            auto socket = std::make_shared<can::LinuxCANSocket>();
            auto result = socket->open(interface, options_.enable_can_fd);

            if (result.isError()) {
                return ErrorCode::transport(TransportError::InvalidInterface);
            }

            sockets_[interface] = socket;
            Logger::info("CAN interface '" + interface + "' opened successfully");
        }

        // 设置默认 socket（单接口模式或第一个接口）
        default_socket_ = sockets_.begin()->second;

        return ErrorCode();
    }

    Result<void> Robot::initializeRouter() {
        router_ = std::make_shared<can::CANFrameRouter>();
        return ErrorCode();
    }

    Result<void> Robot::initializeReceiver() {
        receiver_ = std::make_shared<can::CANReceiver>(default_socket_, router_);

        auto result = receiver_->start();
        if (result.isError()) {
            return ErrorCode::system(SystemError::ThreadCreationFailed);
        }

        // 设置接收线程优先级
        if (options_.receive_thread_priority > 0) {
            pthread_t native = receiver_->getThread().native_handle();
            sched_param param;
            param.sched_priority = options_.receive_thread_priority;

            if (pthread_setschedparam(native, SCHED_FIFO, &param) == 0) {
                Logger::info("Receiver thread set to real-time priority (FIFO, prio=" +
                             std::to_string(options_.receive_thread_priority) + ")");
            } else {
                Logger::warn("Failed to set real-time priority (may need root)");
            }
        }

        return ErrorCode();
    }

    Result<void> Robot::initializeMotorManager() {
        motor::MotorManager::Options motor_opts;
        motor_opts.status_poll_interval_ms = options_.status_poll_interval_ms;
        motor_opts.enable_background_thread = options_.enable_background_thread;
        motor_opts.skip_poll_if_disabled = options_.skip_poll_if_disabled;

        if (multi_interface_mode_) {
            // 多接口模式：使用 router 构造
            motor_manager_ = std::make_shared<motor::MotorManager>(router_, motor_opts);

            // 添加所有接口到 motor manager
            for (const auto &[name, socket]: sockets_) {
                motor_manager_->addInterface(name, socket);
            }
        } else {
            // 单接口模式：使用 socket + router 构造
            motor_manager_ = std::make_shared<motor::MotorManager>(default_socket_, router_, motor_opts);
            
            // 同时用实际接口名称注册，以便电机配置中的 interface_name 能正确匹配
            if (!can_interfaces_.empty()) {
                motor_manager_->addInterface(can_interfaces_[0], default_socket_);
            }
        }

        if (!motor_manager_->start()) {
            return ErrorCode::system(SystemError::ThreadCreationFailed);
        }

        return ErrorCode();
    }

    // ==================== 手臂组件管理 ====================

    std::shared_ptr<ArmComponent> Robot::createArmComponent(
        const std::string &name,
        const std::vector<motor::MotorConfig> &motor_configs) {
        if (!initialized_) {
            Logger::error("Cannot create arm component: Robot not initialized");
            return nullptr;
        }

        if (arm_components_.find(name) != arm_components_.end()) {
            Logger::warn("Arm component '" + name + "' already exists, returning existing");
            return arm_components_[name];
        }

        auto arm = std::make_shared<ArmComponent>(motor_manager_, name);
        arm->initialize(motor_configs);

        arm_components_[name] = arm;
        Logger::info("Created arm component: " + name + " with " +
                     std::to_string(motor_configs.size()) + " motors");

        return arm;
    }

    std::shared_ptr<ArmComponent> Robot::getArmComponent(const std::string &name) const {
        auto it = arm_components_.find(name);
        if (it != arm_components_.end()) {
            return it->second;
        }
        return nullptr;
    }

    void Robot::removeArmComponent(const std::string &name) {
        auto it = arm_components_.find(name);
        if (it != arm_components_.end()) {
            it->second->shutdown();
            arm_components_.erase(it);
            Logger::info("Removed arm component: " + name);
        }
    }

    std::vector<std::string> Robot::getArmComponentNames() const {
        std::vector<std::string> names;
        names.reserve(arm_components_.size());
        for (const auto &[name, _]: arm_components_) {
            names.push_back(name);
        }
        return names;
    }

    // ==================== 电机管理 ====================

    bool Robot::enableAllMotors() {
        if (!initialized_) {
            Logger::error("Cannot enable motors: Robot not initialized");
            return false;
        }
        return motor_manager_->enableAll();
    }

    bool Robot::disableAllMotors() {
        if (!initialized_) {
            Logger::error("Cannot disable motors: Robot not initialized");
            return false;
        }
        return motor_manager_->disableAll();
    }

    bool Robot::emergencyStopAll() {
        if (!initialized_) {
            Logger::error("Cannot emergency stop: Robot not initialized");
            return false;
        }
        return motor_manager_->emergencyStopAll();
    }

    bool Robot::clearAllFaults() {
        if (!initialized_) {
            Logger::error("Cannot clear faults: Robot not initialized");
            return false;
        }
        return motor_manager_->clearAllFaults();
    }

    // ==================== 状态监控 ====================

    void Robot::setGlobalStateCallback(
        std::function<void(uint32_t motor_id, const motor::MotorState &state)> callback) {
        if (motor_manager_) {
            motor_manager_->setGlobalStateCallback(callback);
        }
    }

    std::vector<motor::MotorInfo> Robot::getAllMotorInfos() const {
        if (!initialized_ || !motor_manager_) {
            return {};
        }
        return motor_manager_->getMotorInfos();
    }

    motor::MotorState Robot::getMotorState(uint32_t motor_id) const {
        if (!initialized_ || !motor_manager_) {
            return motor::MotorState{};
        }

        auto motor = motor_manager_->getMotor(motor_id);
        if (motor) {
            return motor->getState();
        }

        return motor::MotorState{};
    }

    std::unordered_map<uint32_t, motor::MotorState> Robot::getAllMotorState() const {
        std::unordered_map<uint32_t, motor::MotorState> states;

        if (!initialized_ || !motor_manager_) {
            return states;
        }

        auto motors = motor_manager_->getAllMotors();
        for (const auto &motor: motors) {
            if (motor) {
                states[motor->id()] = motor->getState();
            }
        }

        return states;
    }

    // ==================== 底层访问 ====================

    std::shared_ptr<can::CANSocket> Robot::getCANSocket(const std::string &interface_name) const {
        if (!interface_name.empty()) {
            auto it = sockets_.find(interface_name);
            if (it != sockets_.end()) {
                return it->second;
            }
            return nullptr;
        }
        return default_socket_;
    }
} // namespace robot
