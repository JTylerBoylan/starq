#include <stdio.h>

#include "starq/mujoco/mujoco.hpp"

using namespace starq::mujoco;

void computeControlInput(const mjModel *m, mjData *d)
{
    (void)m;
    (void)d;
}

int main(void)
{

    MuJoCo::Ptr mujoco = MuJoCo::getInstance();

    mujoco->addMotorControlFunction(computeControlInput);

    mujoco->open("/home/nvidia/starq_ws/src/starq/models/unitree_a1/scene.xml");

    return 0;
}