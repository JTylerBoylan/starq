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
        const auto foot_position = Eigen::Vector3f(0, UNITREE_A1_LENGTH_D, -0.2);
        for (uint8_t id = 0; id < UNITREE_A1_NUM_LEGS; id++)
        {
            robot.setFootPosition(id, foot_position);
        }
        std::this_thread::sleep_for(std::chrono::seconds(5));

        // Force control
        const auto foot_force = Eigen::Vector3f(-100, 100, -250);
        for (uint8_t id = 0; id < UNITREE_A1_NUM_LEGS; id++)
        {
            robot.setFootForce(id, foot_force);
        }
        std::this_thread::sleep_for(std::chrono::seconds(1));

        const auto current_position = robot.getLocalization()->getCurrentPosition();
        const auto current_orientation = robot.getLocalization()->getCurrentOrientation();
        printf("Current position: %f %f %f\n", current_position.x(), current_position.y(), current_position.z());
        printf("Current orientation: %f %f %f\n", current_orientation.x(), current_orientation.y(), current_orientation.z());

        printf("\n");
    }

    robot.waitForSimulation();
    printf("Simulation closed\n");

    printf("Done\n");
    return 0;
}