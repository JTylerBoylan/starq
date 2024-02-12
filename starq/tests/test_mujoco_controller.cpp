#include <stdio.h>
#include <future>

#include "starq/mujoco/mujoco_controller.hpp"

using namespace starq::mujoco;

int main()
{
    MuJoCo::Ptr mujoco = MuJoCo::getInstance();

    MuJoCoController::Ptr controller0 = std::make_shared<MuJoCoController>(mujoco, 0);
    MuJoCoController::Ptr controller1 = std::make_shared<MuJoCoController>(mujoco, 1);
    MuJoCoController::Ptr controller2 = std::make_shared<MuJoCoController>(mujoco, 2);
    MuJoCoController::Ptr controller3 = std::make_shared<MuJoCoController>(mujoco, 3);
    MuJoCoController::Ptr controller4 = std::make_shared<MuJoCoController>(mujoco, 4);
    MuJoCoController::Ptr controller5 = std::make_shared<MuJoCoController>(mujoco, 5);
    MuJoCoController::Ptr controller6 = std::make_shared<MuJoCoController>(mujoco, 6);
    MuJoCoController::Ptr controller7 = std::make_shared<MuJoCoController>(mujoco, 7);
    MuJoCoController::Ptr controller8 = std::make_shared<MuJoCoController>(mujoco, 8);
    MuJoCoController::Ptr controller9 = std::make_shared<MuJoCoController>(mujoco, 9);
    MuJoCoController::Ptr controller10 = std::make_shared<MuJoCoController>(mujoco, 10);
    MuJoCoController::Ptr controller11 = std::make_shared<MuJoCoController>(mujoco, 11);

    printf("Controller created\n");

    std::future<void> sim = std::async(std::launch::async, [&]
                                       { mujoco->open("/home/nvidia/starq_ws/src/models/unitree_a1/scene.xml"); });

    controller0->setControlMode(starq::ControlMode::POSITION);
    controller1->setControlMode(starq::ControlMode::POSITION);
    controller2->setControlMode(starq::ControlMode::POSITION);
    controller3->setControlMode(starq::ControlMode::POSITION);
    controller4->setControlMode(starq::ControlMode::POSITION);
    controller5->setControlMode(starq::ControlMode::POSITION);
    controller6->setControlMode(starq::ControlMode::POSITION);
    controller7->setControlMode(starq::ControlMode::POSITION);
    controller8->setControlMode(starq::ControlMode::POSITION);
    controller9->setControlMode(starq::ControlMode::POSITION);
    controller10->setControlMode(starq::ControlMode::POSITION);
    controller11->setControlMode(starq::ControlMode::POSITION);

    // positions: 0 0.9 -1.8 0 0.9 -1.8 0 0.9 -1.8 0 0.9 -1.8
    controller0->setPosition(0);
    controller1->setPosition(0.9);
    controller2->setPosition(-1.8);
    controller3->setPosition(0);
    controller4->setPosition(0.9);
    controller5->setPosition(-1.8);
    controller6->setPosition(0);
    controller7->setPosition(0.9);
    controller8->setPosition(-1.8);
    controller9->setPosition(0);
    controller10->setPosition(0.9);
    controller11->setPosition(-1.8);

    sim.wait();

    printf("Done\n");

    return 0;
}