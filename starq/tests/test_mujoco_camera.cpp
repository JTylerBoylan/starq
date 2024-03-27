#include <stdio.h>

#include "starq/unitree/unitree_a1_mujoco_robot.hpp"
#include "starq/mujoco/mujoco_camera.hpp"

using namespace starq::mujoco;
using namespace starq::unitree;

int main(void)
{

    MuJoCo::Ptr mujoco = MuJoCo::getInstance();

    auto robot = std::make_shared<UnitreeA1MuJoCoRobot>();

    auto front_camera = std::make_shared<MuJoCoCamera>(mujoco, "front_camera");

    robot->startSimulation();

    auto &camera = robot->openCamera();

    camera.wait();
    robot->waitForSimulation();

    printf("Simulation closed\n");
    return 0;
}