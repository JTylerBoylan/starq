#include <stdio.h>

#include "starq/unitree/unitree_a1_mujoco_robot.hpp"
#include "starq/mujoco/mujoco_camera.hpp"

using namespace starq::mujoco;
using namespace starq::unitree;

int main(void)
{

    // Get MuJoCo singleton instance
    MuJoCo::Ptr mujoco = MuJoCo::getInstance();

    // Create Unitree A1 robot
    auto robot = std::make_shared<UnitreeA1MuJoCoRobot>();

    // Start simulation
    robot->startSimulation();

    // Get camera
    auto &camera = robot->openCamera();

    // Wait for camera to close
    camera.wait();
    printf("Camera closed\n");

    // Wait for simulation to close
    robot->waitForSimulation();
    printf("Simulation closed\n");

    printf("Done.\n");
    return 0;
}