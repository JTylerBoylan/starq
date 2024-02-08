#include <stdio.h>
#include <unistd.h>
#include <future>

#include "starq/mujoco/mujoco_sim.hpp"

using namespace starq::mujoco;

int main(void)
{

    std::future<void> future = std::async(std::launch::async, []() {
        MuJoCo::getInstance()->open("/home/nvidia/starq_ws/src/models/unitree_a1/scene.xml");
    });

    future.wait();

    return 0;
}