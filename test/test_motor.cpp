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
    auto openRes = socket->open("can0", false);
    if (openRes.isError()) {
        std::cerr << "Failed to open CAN\n";
        return -1;
    }

    auto router = std::make_shared<can::CANFrameRouter>();

    // 2. 创建电机管理器
    motor::MotorManager::Options opts;
    opts.status_poll_interval_ms = 10;

    auto manager = std::make_shared<motor::MotorManager>(socket, router, opts);

    // 3. 添加意优电机
    motor::eyou::EYOUFactory::registerMotorType();

    std::cout << "Registered driver types:\n";
    for (const auto& t : motor::MotorFactory::getRegisteredTypes()) {
        std::cout << "  " << t << "\n";
    }

    motor::MotorConfig cfg;
    cfg.id = 1;
    cfg.name = "EYOU_1";
    cfg.type = "EYOU_PP08";
    cfg.tx_can_id = 1;
    cfg.rx_can_id = 1;
    cfg.useStandardFrame(8);

    auto motor1 = motor::MotorFactory::create(cfg);
    auto motor2 = motor::eyou::EYOUFactory::createMotor(2, "EYOU_PP08");

    manager->addMotor(motor1);
    manager->addMotor(motor2);

    // 4. 启动接收和轮询
    auto receiver = std::make_shared<can::CANReceiver>(socket, router);
    receiver->start();
    manager->start();

    // 5. 设置状态回调
    manager->setGlobalStateCallback([](uint32_t id, const motor::MotorState &s) {
        std::cout << "Motor " << id << ": "
                << "pos=" << s.position << " rad, "
                << "torque=" << s.torque << " Nm\n";
    });

    using namespace robot::motor::eyou;

    // 6. 控制示例
    auto eyou = EYOUMotor::from(motor1);

    eyou->enable();
    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    // 设置5Nm扭矩
    eyou->setTorque(5.0);
    std::cout << "Set torque 5Nm, equivalent current: " << EYOUUnits::torqueToCurrent(5.0, 1200.0) << " mA\n";

    std::this_thread::sleep_for(std::chrono::seconds(2));

    // 使用工厂方法创建扭矩命令
    auto torque_cmd = eyou->makeTorqueCmd(3.0);
    std::cout << "Torque command: " << torque_cmd->getTorque() << " Nm\n";
    eyou->command(*torque_cmd);

    std::this_thread::sleep_for(std::chrono::seconds(2));

    // 停止
    manager->disableAll();
    manager->stop();
    receiver->stop();

    return 0;
}
