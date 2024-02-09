#include <stdio.h>
#include <future>

#include "starq/mujoco/mujoco_controller.hpp"

using namespace starq::mujoco;

int main()
{
    MuJoCo::Ptr mujoco = MuJoCo::getInstance();

    MuJoCoController::Ptr controller = std::make_shared<MuJoCoController>(mujoco, 2);

    printf("Controller created\n");

    std::future<void> sim = std::async(std::launch::async, [&]
                                       { mujoco->open("/home/nvidia/starq_ws/src/models/unitree_a1/scene.xml"); });


    controller->setControlMode(starq::ControlMode::POSITION);
    controller->setGains(10, 1);
    controller->setPosition(0);

    sim.wait();

    printf("Done\n");

    return 0;
}