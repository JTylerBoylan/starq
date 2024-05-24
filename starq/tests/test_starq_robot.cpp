#include <stdio.h>
#include <fstream>

#include "starq/starq/starq_robot.hpp"

using namespace starq;

void run_circle_trajectory(STARQRobot::Ptr STARQ,
                           const int frequency, const int resolution, const int num_cycles, const float radius);

void run_file_trajectory(STARQRobot::Ptr STARQ, const std::string &file_name, const Float frequency, const int num_cycles);

void print_temperature_info(STARQRobot::Ptr STARQ, const Float rate);

int main()
{

    // Create STARQ robot
    STARQRobot::Ptr STARQ = std::make_shared<STARQRobot>();
    printf("STARQRobot created\n");

    // Set the motor gains
    const Float p_gain = 100.0;
    const Float v_gain = 0.05;
    const Float vi_gain = 0.15;
    STARQ->setGains(p_gain, v_gain, vi_gain);

    // Set axis state to closed loop control
    if (!STARQ->setStates(AxisState::CLOSED_LOOP_CONTROL))
    {
        // Set axis state to idle on failure
        STARQ->setStates(AxisState::IDLE);
        printf("Failed to set axis state.\n");
        return 1;
    }

    // Start printing temperature info
    const Float print_rate = 1.0; // Hz
    std::thread(print_temperature_info, STARQ, print_rate).detach();

    // Send legs to center (standing) position
    STARQ->goToDefaultFootLocations();

    // Hold stand position
    const float stand_duration = 5.0; // seconds
    usleep(1E6 * stand_duration);

    // Run trajectory from file
    const std::string file_name = "walk_test_2.txt";
    const int num_cycles = 10;
    const Float frequency = 1.5; // Hz
    run_file_trajectory(STARQ, file_name, frequency, num_cycles);

    // Move legs in a circle
    // const float frequency = 0.5f;
    // const int resolution = 100;
    // const int num_cycles = 3;
    // const float radius = 0.025f;
    // run_circle_trajectory(STARQ, frequency, resolution, num_cycles, radius);

    // Send legs to center position
    STARQ->goToDefaultFootLocations();

    usleep(1E6);

    // Set axis state to idle
    STARQ->setStates(AxisState::IDLE);

    printf("Done.\n");
    return 0;
}

void run_circle_trajectory(STARQRobot::Ptr STARQ,
                           const int frequency, const int resolution, const int num_cycles, const float radius)
{
    // Use default foot locations as center positions
    auto center_positions = STARQ->getRobotParameters()->getDefaultFootLocations();

    for (int i = 0; i < num_cycles; i++)
    {
        for (int j = 0; j < resolution; j++)
        {
            // Calculate circle position
            const float angle = 2.0f * M_PI * j / resolution;
            const Vector3 circle_position(radius * cos(angle), 0, radius * sin(angle));
            for (int k = 0; k < 4; k++)
            {
                // Send foot position
                STARQ->setFootPosition(k, center_positions[k] + circle_position);
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(time_t(1E6 / frequency / resolution)));
        }
    }
}

void run_file_trajectory(STARQRobot::Ptr STARQ, const std::string &file_name, const Float frequency, const int num_cycles)
{
    // Run trajectory
    STARQ->runTrajectory("/home/nvidia/starq_ws/src/starq/trajectories/" + file_name, frequency, num_cycles);

    // Wait for trajectory to finish
    while (STARQ->getTrajectoryController()->isRunning())
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
}

void print_temperature_info(STARQRobot::Ptr STARQ, const Float rate)
{
    auto motors = STARQ->getMotors();

    while (true)
    {
        printf("-------------------------\n");
        for (size_t m = 0; m < motors.size(); m++)
        {
            // Cast as odrive motor
            auto odrive_motor = std::dynamic_pointer_cast<odrive::ODriveController>(motors[m]);

            // Get odrive temperature
            const Float temp = odrive_motor->getFETTemperature();

            // Print to console
            printf("[Motor %lu] FET Temp: %.2f\n", m, temp);
        }
        printf("-------------------------\n");

        std::this_thread::sleep_for(std::chrono::milliseconds(time_t(1E3 / rate)));
    }
}
