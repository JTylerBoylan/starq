#include <stdio.h>
#include <unistd.h>
#include <future>

#include "starq/mujoco/mujoco.hpp"

using namespace starq::mujoco;

int main(void)
{

    MuJoCo::getInstance()->setMotorCount(12);
    MuJoCo::getInstance()->setMotorControl(0, 0);
    MuJoCo::getInstance()->setMotorControl(1, 0);
    MuJoCo::getInstance()->setMotorControl(2, 0);
    MuJoCo::getInstance()->setMotorControl(3, 0);
    MuJoCo::getInstance()->setMotorControl(4, 0);
    MuJoCo::getInstance()->setMotorControl(5, 0);
    MuJoCo::getInstance()->setMotorControl(6, 0);
    MuJoCo::getInstance()->setMotorControl(7, 0);
    MuJoCo::getInstance()->setMotorControl(8, 0);
    MuJoCo::getInstance()->setMotorControl(9, 0);
    MuJoCo::getInstance()->setMotorControl(10, 0);
    MuJoCo::getInstance()->setMotorControl(11, 0);

    MuJoCo::getInstance()->open("/home/nvidia/starq_ws/src/models/unitree_a1/scene.xml");

    return 0;
}