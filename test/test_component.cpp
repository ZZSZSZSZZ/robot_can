/**
 * @Author: ZZSZSZSZZ
 * @CreateTime: 2026-02-26
 * @Description: 
 * @Version: 1.0
 */

#include "can/can_socket_impl.hpp"
#include "can/can_frame_router.hpp"
#include "can/can_receiver.hpp"
#include "motor/motor_manager.hpp"
#include "motor/drivers/eyou/eyou_factory.hpp"
#include "motor/drivers/eyou/eyou_motor.hpp"
#include "robot/arm_component.hpp"

int main() {
    using namespace robot;

    // 1. 创建CAN组件
    auto socket = std::make_shared<can::LinuxCANSocket>();
    auto openRes = socket->open("vcan0", false);
    if (openRes.isError()) {
        std::cerr << "Failed to open CAN\n";
        return -1;
    }

    auto router = std::make_shared<can::CANFrameRouter>();

    // 4. 启动接收和轮询
    auto receiver = std::make_shared<can::CANReceiver>(socket, router);
    receiver->start();

    pthread_t native = receiver->getThread().native_handle();
    sched_param param;
    param.sched_priority = 50; // 优先级范围通常 1-99（需 root 权限）
    if (pthread_setschedparam(native, SCHED_FIFO, &param) == 0) {
        Logger::info("Receiver thread set to real-time priority (FIFO, prio=50)");
    } else {
        Logger::warn("Failed to set real-time priority (may need root)");
    }


    // 2. 创建电机管理器
    motor::MotorManager::Options opts;
    opts.status_poll_interval_ms = 10;

    auto manager = std::make_shared<motor::MotorManager>(socket, router, opts);

    // 3. 添加意优电机
    motor::eyou::EYOUFactory::registerMotorType();

    auto arm = std::make_shared<ArmComponent>(manager);

    std::vector<motor::MotorConfig> cfgs;

    for (int i = 1; i < 2; ++i) {
        motor::MotorConfig cfg;
        cfg.id = i;
        cfg.name = "EYOU_" + std::to_string(i);
        cfg.type = "EYOU_PP11";
        if (i > 4) cfg.type = "EYOU_PP11L";
        cfg.tx_can_id = i;
        cfg.rx_can_id = i;
        cfg.useStandardFrame(8);
        cfgs.push_back(cfg);
    }

    arm->initialize(cfgs);

    // 5. 设置状态回调
    manager->setGlobalStateCallback([](uint32_t id, const motor::MotorState &s) {
        std::cout << "Motor " << id << ": "
                << "pos=" << s.position << " rad, "
                << "torque=" << s.torque << " Nm\n";
    });

    if (!arm->enableAll()) {
        Logger::warn("Failed to enable all arm");
        return 0;
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    // 6. 控制示例

    std::vector<PositionParam> params;
    std::vector<double> position = {0.6, 0.6, 0.6, -0.6, 0.6, 0.6, 0.6};
    std::vector<double> velocity = {25, 25, 25, 10, 10, 5, 5};
    std::vector<double> torque = {1, 1, 2, 8, 12, 12, 12};
    // std::vector<double> kp = {10, 10, 10, 10, 10, 10, 10};
    // std::vector<double> kd = {1, 1, 1, 1, 1, 1, 1};

    params.reserve(arm->getMotorCount());
    for (size_t i = 0; i < arm->getMotorCount(); ++i) {
        params[i] = {position[i], velocity[i], torque[i], 10};
    }
    arm->setPositions(params);

    std::this_thread::sleep_for(std::chrono::seconds(5));

    for (int i = 1; i < 157; ++i) {
        double x = i * 0.01;
        position = {0 + x, 0 + x, 0 + x, 0 - x, 0 - x, 0 + x, 0 + x};

        for (size_t j = 0; j < arm->getMotorCount(); ++j) {
            params[i] = {position[j], velocity[j], torque[j], 10};
        }
        arm->setPositions(params);
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    std::this_thread::sleep_for(std::chrono::seconds(5));
    params.clear();

    // mit 控制 千万不要用
    // for (int i = 1; i < 157; ++i) {
    //     double x = i * 0.01;
    //     positions = {0 + x, 0 + x, 0 + x, 0 - x, 0 - x, 0 + x, 0 + x};
    //     arm->setMITPositions(positions, velocity, torques, kp, kd);
    //     std::this_thread::sleep_for(std::chrono::milliseconds(10));
    // }
    // std::this_thread::sleep_for(std::chrono::seconds(5));

    for (size_t i = 0; i < arm->getMotorCount(); ++i) {
        params[i] = {0, velocity[i], torque[i], 10};
    }
    arm->setPositions(params);

    std::this_thread::sleep_for(std::chrono::seconds(5));

    if (!arm->disableAll()) {
        Logger::warn("Failed to disable all arm");
        return 0;
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    // 停止
    params.clear();
    arm->shutdown();
    manager->stop();
    receiver->stop();

    return 0;
}
