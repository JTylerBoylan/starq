#include <stdio.h>

#include "starq/robots/unitree_a1_mujoco.hpp"

using namespace starq;

int main()
{

    robots::UnitreeA1MuJoCoRobot robot;
    printf("UnitreeA1MuJoCoRobot created\n");

    robot.startSimulation();
    printf("Simulation started\n");

    while (robot.isSimulationOpen())
    {
        // Position control
        auto foot_position = Eigen::Vector3f(0, UNITREE_A1_LENGTH_D, -0.2);
        for (uint8_t id = 0; id < UNITREE_A1_NUM_LEGS; id++)
        {
            robot.setFootPosition(id, foot_position);
        }
        std::this_thread::sleep_for(std::chrono::seconds(5));

        // Force control
        auto foot_force = Eigen::Vector3f(-100, 0, -250);
        for (uint8_t id = 0; id < UNITREE_A1_NUM_LEGS; id++)
        {
            robot.setFootForce(id, foot_force);
        }
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }

    robot.waitForSimulation();
    printf("Simulation closed\n");

    printf("Done\n");
    return 0;
}