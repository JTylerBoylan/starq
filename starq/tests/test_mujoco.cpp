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

    // Launch simulation
    mujoco->load("/home/nvidia/starq_ws/src/starq/models/unitree_a1/scene.xml");
    mujoco->start();
    
    // Wait for the simulation to open
    while (!mujoco->isOpen())
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    printf("Simulation started\n");
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    // Wait for simulation to finish
    mujoco->wait();

    printf("Done.\n");
    return 0;
}