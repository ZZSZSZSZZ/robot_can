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
#include "motor/drivers/eyou/eyou_units.hpp"  // 可以直接使用 EYOUUnits

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

    // 2. 创建电机管理器
    motor::MotorManager::Options opts;
    opts.status_poll_interval_ms = 10;
    opts.skip_poll_if_disabled = true;

    auto manager = std::make_shared<motor::MotorManager>(socket, router, opts);

    // 3. 添加意优电机
    motor::eyou::EYOUFactory::registerMotorType();

    std::cout << "Registered driver types:\n";
    for (const auto &t: motor::MotorFactory::getRegisteredTypes()) {
        std::cout << "  " << t << "\n";
    }

    motor::MotorConfig cfg;
    cfg.id = 1;
    cfg.name = "EYOU_1";
    cfg.type = "EYOU_PP11";
    cfg.tx_can_id = cfg.id;
    cfg.rx_can_id = cfg.id;
    cfg.useStandardFrame(8);

    auto motor1 = motor::MotorFactory::create(cfg);

    cfg.id = 2;
    cfg.name = "EYOU_2";
    cfg.type = "EYOU_PP11";
    cfg.tx_can_id = cfg.id;
    cfg.rx_can_id = cfg.id;
    cfg.useStandardFrame(8);

    auto motor2 = motor::MotorFactory::create(cfg);

    cfg.id = 3;
    cfg.name = "EYOU_3";
    cfg.type = "EYOU_PP11";
    cfg.tx_can_id = cfg.id;
    cfg.rx_can_id = cfg.id;
    cfg.useStandardFrame(8);

    auto motor3 = motor::MotorFactory::create(cfg);

    cfg.id = 4;
    cfg.name = "EYOU_4";
    cfg.type = "EYOU_PP11L";
    cfg.tx_can_id = cfg.id;
    cfg.rx_can_id = cfg.id;
    cfg.useStandardFrame(8);

    auto motor4 = motor::MotorFactory::create(cfg);

    cfg.id = 5;
    cfg.name = "EYOU_5";
    cfg.type = "EYOU_PP11L";
    cfg.tx_can_id = cfg.id;
    cfg.rx_can_id = cfg.id;
    cfg.useStandardFrame(8);

    auto motor5 = motor::MotorFactory::create(cfg);

    cfg.id = 6;
    cfg.name = "EYOU_6";
    cfg.type = "EYOU_PP11L";
    cfg.tx_can_id = cfg.id;
    cfg.rx_can_id = cfg.id;
    cfg.useStandardFrame(8);

    auto motor6 = motor::MotorFactory::create(cfg);

    cfg.id = 7;
    cfg.name = "EYOU_7";
    cfg.type = "EYOU_PP11L";
    cfg.tx_can_id = cfg.id;
    cfg.rx_can_id = cfg.id;
    cfg.useStandardFrame(8);

    auto motor7 = motor::MotorFactory::create(cfg);

    // cfg.id = 8;
    // cfg.name = "EYOU_8";
    // cfg.type = "EYOU_PP11L";
    // cfg.useStandardFrame(8);
    //
    // auto motor8 = motor::MotorFactory::create(cfg);
    //
    // cfg.id = 9;
    // cfg.name = "EYOU_9";
    // cfg.type = "EYOU_PP11L";
    // cfg.useStandardFrame(8);
    //
    // auto motor9 = motor::MotorFactory::create(cfg);
    // auto motor2 = motor::eyou::EYOUFactory::createMotor(2, "EYOU_PP08");

    manager->addMotor(motor1);
    manager->addMotor(motor2);
    manager->addMotor(motor3);
    manager->addMotor(motor4);
    manager->addMotor(motor5);
    manager->addMotor(motor6);
    manager->addMotor(motor7);
    // manager->addMotor(motor8);
    // manager->addMotor(motor9);

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

    if (!manager->enableAll()) return 0;
    // manager->enableAll();

    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    using namespace robot::motor::eyou;

    // 6. 控制示例

    // motor1->enable();

    // 设置5Nm扭矩
    // motor1->setTorque(-1.2);
    // std::cout << "Set torque 5Nm, equivalent current: " << EYOUUnits::torqueToCurrent(5.0, 1200.0) << " mA\n";
    // eyou->setZeroPosition();

    motor1->setPosition(0.6, 25, 1);
    motor2->setPosition(0.6, 25, 1);
    motor3->setPosition(0.6, 25, 2);
    motor4->setPosition(-0.6, 10, 8);
    motor5->setPosition(0.6, 10, 12);
    motor6->setPosition(0.6, 5, 12);
    motor7->setPosition(0.6, 5, 12);

    std::this_thread::sleep_for(std::chrono::seconds(5));

    motor1->setPosition(1.57, 25, 1);
    motor2->setPosition(1.57, 25, 1);
    motor3->setPosition(1.57, 25, 2);
    motor4->setPosition(-1.57, 10, 8);
    motor5->setPosition(-1.57, 10, 12);
    motor6->setPosition(1.57, 5, 12);
    motor7->setPosition(1.57, 5, 12);

    std::this_thread::sleep_for(std::chrono::seconds(5));

    motor1->setPosition(0, 25, 1);
    motor2->setPosition(0, 25, 1);
    motor3->setPosition(0, 25, 2);
    motor4->setPosition(0, 10, 8);
    motor5->setPosition(0, 10, 12);
    motor6->setPosition(0, 5, 12);
    motor7->setPosition(0, 5, 12);

    std::this_thread::sleep_for(std::chrono::seconds(5));

    // 使用工厂方法创建扭矩命令

    // auto eyou1 = EYOUMotor::from(motor1);

    // auto torque_cmd = eyou->makeTorqueCmd(3.0);
    // std::cout << "Torque command: " << torque_cmd->getTorque() << " Nm\n";
    // eyou->command(*torque_cmd);

    // std::this_thread::sleep_for(std::chrono::seconds(2));
    // std::this_thread::sleep_for(std::chrono::milliseconds(10));

    manager->disableAll();

    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    // 停止
    manager->stop();
    receiver->stop();

    return 0;
}
