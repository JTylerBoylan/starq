#include <stdio.h>

#include "starq/mujoco/mujoco.hpp"

// Simplest MuJoCo script

// Callback function to compute control input
void computeControlInput(const mjModel *m, mjData *d)
{
    (void)m;
    (void)d;
    // empty
}

using namespace starq::mujoco;

int main(void)
{

    // Get MuJoCo singleton instance
    MuJoCo::Ptr mujoco = MuJoCo::getInstance();

    // Add control input callback function
    mujoco->addMotorControlFunction(computeControlInput);

    // Open MuJoCo model file (blocks until simulation is done)
    mujoco->open("/home/nvidia/starq_ws/src/starq/models/unitree_a1/scene.xml");

    printf("Done.\n");
    return 0;
}