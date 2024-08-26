#include <stdio.h>

#include "starq/starq/starq_robot.hpp"

#define TRAJECTORY_FOLDER "/home/nvidia/starq_ws/starq/trajectories/"

using namespace starq;

void print_usage(char *name);
bool parse_args(int argc, char **argv, std::string &file_name, Float &frequency, int &num_loops);

int main(int argc, char **argv)
{

    // Parse arguments
    std::string file_name;
    Float frequency = 1.0;
    int num_loops = 1;
    if (!parse_args(argc, argv, file_name, frequency, num_loops))
    {
        print_usage(argv[0]);
        return 1;
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
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }

    // Run trajectory
    for (int loop = 1; loop <= num_loops; loop++)
    {
        printf("Loop %d/%d\n", loop, num_loops);
        if (!STARQ->runTrajectory(TRAJECTORY_FOLDER + file_name, frequency, 1))
        {
            printf("Failed to run trajectory\n");
            return 1;
        }

        // Wait for trajectory to finish
        STARQ->getTrajectoryController()->wait();
    }
    printf("Trajectory finished\n");

    // Switch to idle
    STARQ->setStates(AxisState::IDLE);

    return 0;
}

void print_usage(char *name)
{
    printf("! Run a trajectory from a file on the STARQ robot\n");
    printf("Usage: %s <file> <frequency> <num_loops>\n", name);
    printf("  file:       Trajectory file name (relative to the trajectories folder) (required)\n");
    printf("  frequency:  Trajectory loop frequency (default: 1.0)\n");
    printf("  num_loops:  Number of trajectory loops (default: 1)\n");
    printf("! Example: %s walk_test.txt 0.5 10\n", name);
}

bool parse_args(int argc, char **argv, std::string &file_name, Float &frequency, int &num_loops)
{
    if (argc < 2 || argc > 4)
    {
        return false;
    }

    file_name = argv[1];
    if (argc > 2)
    {
        frequency = std::stof(argv[2]);
    }
    if (argc > 3)
    {
        num_loops = std::stoi(argv[3]);
    }

    return true;
}