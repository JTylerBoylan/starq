#include <stdio.h>

#include "starq/unitree/unitree_a1_mujoco_robot.hpp"

using namespace starq;
using namespace starq::mpc;
using namespace starq::unitree;

int main()
{

    // The robot object handles the configuration for various components of the robot, such as the
    //   the motor & leg controllers, robot parameters, leg kinematics, and localization.
    // It also contains utility functions for common robot operations, such as setting foot positions,
    //   velocities, and forces, loading trajectories, and running model predictive control.

    // Create Unitree A1 robot
    UnitreeA1MuJoCoRobot robot;
    printf("UnitreeA1MuJoCoRobot created\n");

    // Load walk gait from file
    Gait::Ptr walk_gait = std::make_shared<Gait>();
    walk_gait->load("/home/nvidia/starq_ws/src/starq/gaits/walk.txt");
    printf("Walk Gait loaded\n");

    // Set gait to walk forward at constant velocity
    walk_gait->setVelocity(Vector3(0.5, 0, 0), Vector3(0, 0, 0));

    // Set gait frequency
    walk_gait->setFrequency(3.0);

    // Load stand gait from file
    Gait::Ptr stand_gait = std::make_shared<Gait>();
    stand_gait->load("/home/nvidia/starq_ws/src/starq/gaits/stand.txt");
    printf("Stand Gait loaded\n");

    // Stand still at default height
    stand_gait->setPose(Vector3(0, 0, UNITREE_A1_STAND_HEIGHT), Vector3(0, 0, 0));

    // Run at high frequencies, as it is a static pose
    stand_gait->setFrequency(10.0);

    // Start simulation
    robot.startSimulation();
    printf("Simulation started\n");

    // Send feet to default positions
    for (uint8_t id = 0; id < UNITREE_A1_NUM_LEGS; id++)
    {
        robot.setFootPosition(id, robot.getRobotParameters()->getDefaultFootLocations()[id]);
    }
    printf("Holding foot position for 5 seconds...\n");

    // Wait for simulation to settle 
    std::this_thread::sleep_for(std::chrono::seconds(5));

    // Start MPC
    printf("Starting MPC...\n");
    if (!robot.startMPC())
    {
        printf("Failed to start MPC.\n");
        return 1;
    }

    // Start with the stand gait for 5 seconds
    printf("Standing for 5 seconds...\n");
    robot.setNextGait(stand_gait);
    std::this_thread::sleep_for(std::chrono::seconds(5));

    // Transition to walking
    printf("Walking...\n");
    robot.setNextGait(walk_gait);

    // Run while the simulation is open
    while (robot.isSimulationOpen())
    {

        // Get localization information
        const milliseconds time = robot.getLocalization()->getCurrentTime();
        const Vector3 position = robot.getLocalization()->getCurrentPosition();
        const Vector3 orientation = robot.getLocalization()->getCurrentOrientation();
        const Vector3 linear_velocity = robot.getLocalization()->getCurrentLinearVelocity();
        const Vector3 angular_velocity = robot.getLocalization()->getCurrentAngularVelocity();

        // Print localization information
        printf("----------------------------------------\n");
        printf("Time: %d\n", int(time.count()));
        printf("Position: %f %f %f\n", position.x(), position.y(), position.z());
        printf("Orientation: %f %f %f\n", orientation.x(), orientation.y(), orientation.z());
        printf("Linear velocity: %f %f %f\n", linear_velocity.x(), linear_velocity.y(), linear_velocity.z());
        printf("Angular velocity: %f %f %f\n", angular_velocity.x(), angular_velocity.y(), angular_velocity.z());
        printf("\n\n");

        // Wait for 50 ms
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }

    // Wait for simulation to close
    robot.waitForSimulation();
    printf("Simulation closed\n");

    printf("Done\n");
    return 0;
}