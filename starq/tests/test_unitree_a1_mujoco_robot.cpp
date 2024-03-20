#include <stdio.h>

#include "starq/unitree/unitree_a1_mujoco_robot.hpp"

using namespace starq;
using namespace starq::mpc;
using namespace starq::unitree;

int main()
{
    Gait::Ptr walk_gait = std::make_shared<Gait>();
    walk_gait->load("/home/nvidia/starq_ws/src/starq/gaits/walk.txt");
    walk_gait->setVelocity(Vector3(0.5, 0, 0), Vector3(0, 0, 0));
    walk_gait->setFrequency(3.0);
    printf("Walk Gait loaded\n");

    Gait::Ptr stand_gait = std::make_shared<Gait>();
    stand_gait->load("/home/nvidia/starq_ws/src/starq/gaits/stand.txt");
    stand_gait->setPose(Vector3(0, 0, UNITREE_A1_STAND_HEIGHT), Vector3(0, 0, 0));
    stand_gait->setFrequency(10.0);
    printf("Stand Gait loaded\n");

    UnitreeA1MuJoCoRobot robot;
    printf("UnitreeA1MuJoCoRobot created\n");

    robot.startSimulation();
    printf("Simulation started\n");

    for (uint8_t id = 0; id < UNITREE_A1_NUM_LEGS; id++)
    {
        robot.setFootPosition(id, robot.getRobotDynamics()->getDefaultFootLocations()[id]);
    }
    printf("Holding foot position for 5 seconds...\n");
    std::this_thread::sleep_for(std::chrono::seconds(5));

    printf("Starting MPC...\n");
    if (!robot.startMPC())
    {
        printf("Failed to start MPC.\n");
        return 1;
    }

    printf("Standing for 5 seconds...\n");
    robot.setNextGait(stand_gait);
    std::this_thread::sleep_for(std::chrono::seconds(5));

    printf("Walking...\n");
    robot.setNextGait(walk_gait);

    while (robot.isSimulationOpen())
    {

        const milliseconds time = robot.getLocalization()->getCurrentTime();
        const Vector3 position = robot.getLocalization()->getCurrentPosition();
        const Vector3 orientation = robot.getLocalization()->getCurrentOrientation();
        const Vector3 linear_velocity = robot.getLocalization()->getCurrentLinearVelocity();
        const Vector3 angular_velocity = robot.getLocalization()->getCurrentAngularVelocity();

        printf("----------------------------------------\n");
        printf("Time: %d\n", int(time.count()));
        printf("Position: %f %f %f\n", position.x(), position.y(), position.z());
        printf("Orientation: %f %f %f\n", orientation.x(), orientation.y(), orientation.z());
        printf("Linear velocity: %f %f %f\n", linear_velocity.x(), linear_velocity.y(), linear_velocity.z());
        printf("Angular velocity: %f %f %f\n", angular_velocity.x(), angular_velocity.y(), angular_velocity.z());
        printf("\n\n");

        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }

    robot.waitForSimulation();
    printf("Simulation closed\n");

    printf("Done\n");
    return 0;
}