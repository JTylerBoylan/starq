#include <stdio.h>

#include "starq/starq/starq_robot.hpp"

using namespace starq;

int main(int argc, char **argv)
{

    // Check if the number of arguments is correct
    if (argc < 2 || argc > 4)
    {
        printf("Usage: %s <file> <frequency=1> <num_loops=1>\n", argv[0]);
        return 1;
    }

    // Parse arguments
    std::string file_name = argv[1];
    Float frequency = 1.0;
    int num_loops = 1;
    if (argc > 2)
    {
        frequency = std::stof(argv[2]);
    }
    if (argc > 3)
    {
        num_loops = std::stoi(argv[3]);
    }

    // Create a STARQ robot
    STARQRobot::Ptr STARQ = std::make_shared<STARQRobot>();

    // Switch to control
    STARQ->setStates(AxisState::CLOSED_LOOP_CONTROL);

    printf("Running trajectory from file: %s\n", file_name.c_str());
    printf("Frequency: %f\n", frequency);
    printf("Number of loops: %d\n", num_loops);

    // Countdown
    const int countdown = 5;
    for (int i = countdown; i > 0; i--)
    {
        printf("Starting in %d seconds...\n", i);
        usleep(1E6);
    }

    // Run trajectory
    for (int loop = 1; loop <= num_loops; loop++)
    {
        printf("Loop %d/%d\n", loop, num_loops);
        STARQ->runTrajectory("/home/nvidia/starq_ws/src/starq/trajectories/" + file_name, frequency, 1);

        // Wait for trajectory to finish
        STARQ->getTrajectoryController()->wait();
    }
    printf("Trajectory finished\n");

    // Switch to idle
    STARQ->setStates(AxisState::IDLE);

    return 0;
}