#include <stdio.h>

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
    const Vector3 center_position(0, 0, -std::sqrt(2) * 0.1f);
    for (uint32_t i = 0; i < 4; i++)
    {
        STARQ->setFootPosition(i, center_position);
    }

    const int time_s = 30;
    for (int t = 0; t < time_s; t++)
    {
        printf("-------------------------\n");
        printf("Time: %d s\n", t);
        for (auto motor : STARQ->getMotors())
        {
            // Cast as odrive motor
            auto odrive_motor = std::dynamic_pointer_cast<odrive::ODriveController>(motor);

            // Get odrive temperature
            const int id = odrive_motor->getCANID();
            const Float temp = odrive_motor->getFETTemperature();

            printf("[Motor %d] FET Temp: %.2f\n", id, temp);
        }
        printf("-------------------------\n");
        usleep(1E6);
    }

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

    // // Load trajectory from file
    // STARQ->loadTrajectory("/home/nvidia/starq_ws/src/starq/trajectories/walk_test.txt");

    // const int traj_cycles = 5;
    // for (int i = 0; i < traj_cycles; i++)
    // {
    //     // Start trajectory
    //     STARQ->startTrajectory();

    //     // Wait for trajectory to finish
    //     while(STARQ->getTrajectoryPublisher()->isRunning())
    //     {
    //         usleep(1E4);
    //     }
    // }

    // Set axis state to idle
    STARQ->setState(AxisState::IDLE);

    printf("Done.\n");
    return 0;
}