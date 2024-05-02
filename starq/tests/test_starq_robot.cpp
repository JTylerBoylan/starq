#include <stdio.h>
#include <fstream>

#include "starq/starq/starq_robot.hpp"

using namespace starq;

int main()
{

    // Create STARQ robot
    STARQRobot::Ptr STARQ = std::make_shared<STARQRobot>();
    printf("STARQRobot created\n");

    // Set axis state to closed loop control
    if (!STARQ->setState(AxisState::CLOSED_LOOP_CONTROL))
    {
        // Set axis state to idle on failure
        STARQ->setState(AxisState::IDLE);
        printf("Failed to set axis state.\n");
        return 1;
    }

    // Send legs to center position
    const Float center_z = -std::sqrt(STARQ_LINK_LENGTH_2*STARQ_LINK_LENGTH_2 - STARQ_LINK_LENGTH_1*STARQ_LINK_LENGTH_1);
    const Vector3 center_position(0, 0, center_z);
    for (uint32_t i = 0; i < 4; i++)
    {
        STARQ->setFootPosition(i, center_position);
    }

    // Open file for logging
    // std::ofstream log_file;
    // log_file.open("/home/nvidia/starq_ws/src/logging/20240502_StandTempTest.txt");

    // Get motors
    auto motors = STARQ->getMotors();

    const int time_s = 5;
    for (int t = 0; t < time_s; t++)
    {
        printf("-------------------------\n");
        printf("Time: %d s\n", t);

        // log_file << t << " ";

        for (size_t m = 0; m < motors.size(); m++)
        {
            // Cast as odrive motor
            auto odrive_motor = std::dynamic_pointer_cast<odrive::ODriveController>(motors[m]);

            // Get odrive temperature
            const Float temp = odrive_motor->getFETTemperature();

            // Print to console
            printf("[Motor %lu] FET Temp: %.2f\n", m, temp);

            // Log to file
            // log_file << temp << " ";
        }
        printf("-------------------------\n");
        // log_file << std::endl;

        usleep(1E6);
    }

    // log_file.close();

    // // Move legs in a circle
    // const float frequency = 0.5f;
    // const int resolution = 100;
    // const int num_cycles = 3;
    // const float radius = 0.025f;
    // for (int i = 0; i < num_cycles; i++)
    // {
    //     for (int j = 0; j < resolution; j++)
    //     {
    //         const float angle = 2.0f * M_PI * j / resolution;
    //         const Vector3 circle_position(radius * cos(angle), 0, radius * sin(angle));
    //         STARQ->setFootPosition(0, center_position + circle_position);
    //         STARQ->setFootPosition(1, center_position + circle_position);
    //         STARQ->setFootPosition(2, center_position + circle_position);
    //         STARQ->setFootPosition(3, center_position + circle_position);
    //         usleep(1E6 / frequency / resolution);
    //     }
    // }

    // Load trajectory from file
    STARQ->loadTrajectory("/home/nvidia/starq_ws/src/starq/trajectories/walk_test_2.txt");

    const int traj_cycles = 10;
    for (int i = 0; i < traj_cycles; i++)
    {
        // Start trajectory
        STARQ->startTrajectory();

        // Wait for trajectory to finish
        while(STARQ->getTrajectoryPublisher()->isRunning())
        {
            usleep(1E4);
        }
    }

    // Send legs to center position
    for (uint32_t i = 0; i < 4; i++)
    {
        STARQ->setFootPosition(i, center_position);
    }

    usleep(1E6);

    // Set axis state to idle
    STARQ->setState(AxisState::IDLE);

    // Cleanup STARQ robot
    STARQ->cleanup();

    printf("Done.\n");
    return 0;
}
