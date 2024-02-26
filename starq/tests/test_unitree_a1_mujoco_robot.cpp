#include <stdio.h>

#include "starq/robots/unitree_a1_mujoco.hpp"

using namespace starq;

int main() {

    robots::UnitreeA1MuJoCoRobot robot;
    printf("UnitreeA1MuJoCoRobot created\n");

    robot.startSimulation();
    printf("Simulation started\n");

    auto foot_force = Eigen::Vector3f(0, 0, -100);
    for (uint8_t id = 0; id < UNITREE_A1_NUM_LEGS; id++)
    {
        robot.setFootForce(id, foot_force);
    }
    printf("Leg commands sent\n");

    robot.waitForSimulation();
    printf("Simulation closed\n");

    printf("Done\n");
    return 0;
}