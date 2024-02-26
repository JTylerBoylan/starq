#include <stdio.h>

#include "starq/robots/unitree_a1_mujoco.hpp"

using namespace starq;

int main() {

    robots::UnitreeA1MuJoCoRobot robot;
    printf("UnitreeA1MuJoCoRobot created\n");

    robot.startSimulation();
    printf("Simulation started\n");

    LegCommand leg_command;
    leg_command.delay = std::chrono::microseconds(0);
    leg_command.control_mode = ControlMode::TORQUE;
    leg_command.target_force = Eigen::Vector3f(0, 0, -100);

    for (uint32_t id = 0; id < 4; id++)
    {
        leg_command.leg_id = id;
        robot.getLegCommandPublisher()->sendCommand(std::make_shared<LegCommand>(leg_command));
    }
    printf("Leg commands sent\n");

    robot.waitForSimulation();
    printf("Simulation closed\n");

    printf("Done\n");
    return 0;
}