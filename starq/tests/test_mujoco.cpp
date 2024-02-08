#include <stdio.h>
#include <unistd.h>
#include <future>

#include "starq/mujoco/mujoco.hpp"

using namespace starq::mujoco;

void computeControlInput(const mjModel *m, mjData *d)
{
    for (int i = 0; i < m->nu; i++)
    {
        d->ctrl[i] = 0;
    }
}

int main(void)
{

    MuJoCo::Ptr mujoco = MuJoCo::getInstance();

    mujoco->addMotorControlFunction(computeControlInput);

    mujoco->open("/home/nvidia/starq_ws/src/models/unitree_a1/scene.xml");

    return 0;
}