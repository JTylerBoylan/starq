#include <stdio.h>
#include <unistd.h>

#include "starq/mujoco/mujoco_sim.hpp"

using namespace starq::mujoco;

int main(void)
{

    MuJoCoSim sim("/home/nvidia/starq_ws/src/models/unitree_a1/scene.xml");

    sim.open();

    while (sim.isRunning())
    {
        usleep(1E3); // sleep for 1ms
    }

    sim.close();

    return 0;

}