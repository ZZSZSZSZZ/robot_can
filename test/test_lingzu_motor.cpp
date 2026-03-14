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

#include "motor/drivers/lingzu/lingzu_factory.hpp"
#include "motor/drivers/lingzu/lingzu_motor.hpp"
#include "motor/drivers/lingzu/lingzu_units.hpp"

int main() {
    using namespace robot;

    // 1. 创建CAN组件
    auto socket = std::make_shared<can::LinuxCANSocket>();
    auto openRes = socket->open("can0", false);
    if (openRes.isError()) {
        std::cerr << "Failed to open CAN\n";
        return -1;
    }

    auto router = std::make_shared<can::CANFrameRouter>();

    // 2. 创建电机管理器
    motor::MotorManager::Options opts;
    opts.status_poll_interval_ms = 10;
    opts.skip_poll_if_disabled = true;

    auto manager = std::make_shared<motor::MotorManager>(socket, router, opts);

    // 3. 添加电机
    motor::lingzu::LingzuFactory::registerMotorType();

    std::cout << "Registered driver types:\n";
    for (const auto &t: motor::MotorFactory::getRegisteredTypes()) {
        std::cout << "  " << t << "\n";
    }

    motor::MotorConfig cfg;
    cfg.id = 4;
    cfg.name = "LINGZU_1";
    cfg.type = "LINGZU_RS05";  // 使用已定义的电机类型
    cfg.tx_can_id = cfg.id;
    cfg.rx_can_id = cfg.id;
    cfg.useExtendedFrame(8);  // 灵足电机使用扩展帧

    auto motor1 = motor::MotorFactory::create(cfg);

    manager->addMotor(motor1);

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

    manager->start();

    // 5. 设置状态回调
    manager->setGlobalStateCallback([](uint32_t id, const motor::MotorState &s) {
        std::cout << "Motor " << id << ": "
                  << "pos=" << s.position << " rad, "
                  << "torque=" << s.torque << " Nm\n";
    });

//    if (!manager->enableAll()) return 0;
    manager->enableAll();

    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    std::this_thread::sleep_for(std::chrono::seconds(10));

//    motor1->setMIT(3.14, 0, 0, 5, 1);

    std::this_thread::sleep_for(std::chrono::seconds(3));

//    motor1->setMIT(0, 0, 0, 5, 1);

    std::this_thread::sleep_for(std::chrono::seconds(3));


    manager->disableAll();

    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    // 停止
    manager->stop();
    receiver->stop();

    return 0;
}
