#include <stdio.h>

#include "starq/leg_controller.hpp"
#include "starq/odrive/odrive_controller.hpp"
#include "starq/starq/starq_fivebar2d_leg_dynamics.hpp"

// Leg link lengths in meters (ET-Quad Leg)
#define LEG_LINK_1_LENGTH_M 0.05f
#define LEG_LINK_2_LENGTH_M 0.150f

// Gear ratios on motors (ET-Quad Hip)
#define GEAR_RATIO_A 6.0f
#define GEAR_RATIO_B 6.0f

using namespace starq;
using namespace starq::can;
using namespace starq::odrive;

int main(void)
{
    printf("Hello world!\n");

    // Create a CAN socket object on the can0 interface
    CANSocket::Ptr can_socket = std::make_shared<CANSocket>("can0");

    // Connect to the CAN interface
    if (can_socket->connect())
    {
        printf("Connected to CAN interface.\n");
    }
    else
    {
        printf("Failed to connect to CAN interface.\n");
        return 1;
    }

    // Create an ODrive socket object using the CAN socket
    ODriveSocket::Ptr odrive_socket = std::make_shared<ODriveSocket>(can_socket);

    // Create two ODrive controllers for the hip motors
    ODriveController::Ptr odrive_A = std::make_shared<ODriveController>(odrive_socket, 0);
    ODriveController::Ptr odrive_B = std::make_shared<ODriveController>(odrive_socket, 1);
    printf("Created ODrive controllers.\n");

    // Set gear ratios
    odrive_A->setGearRatio(GEAR_RATIO_A);
    odrive_B->setGearRatio(GEAR_RATIO_B);
    printf("Set gear ratios.\n");

    // Create LegDynamics object for five-bar 2D leg
    STARQFiveBar2DLegDynamics::Ptr fivebar_dynamics = std::make_shared<STARQFiveBar2DLegDynamics>(LEG_LINK_1_LENGTH_M,
                                                                                                  LEG_LINK_2_LENGTH_M);

    // Create LegController object from dynamics and ODrive controllers
    LegController::Ptr leg = std::make_shared<LegController>(fivebar_dynamics,
                                                             std::vector<MotorController::Ptr>{odrive_A, odrive_B});
    printf("Created leg controller.\n");

    // Set motor states to closed loop control
    if (!leg->setState(AxisState::CLOSED_LOOP_CONTROL))
        return 1;
    printf("Set axis state.\n");

    // Set control mode to position control
    if (!leg->setControlMode(ControlMode::POSITION))
        return 1;
    printf("Set control mode.\n");


    // Circular Trajectory
    // Center position of the leg space range
    const float center_x = 0.0f;
    const float center_y = -std::sqrt(2) * 0.1f;

    // Go to center position
    if (!leg->setFootPosition(Vector3(center_x, center_y, 0)))
        return 1;
    printf("Centered foot position.\n");

    // Circle parameters
    const int revolutions = 10;
    const float radius = 0.025f;
    const float frequency = 0.5f;

    // Move leg in a circular trajectory
    printf("Starting leg movement in circle of radius %.4f for %d revolutions at %.2f Hz.\n", radius, revolutions, frequency);
    for (float t = 0.0f; t < 2.0 * M_PI * revolutions; t += 2.0 * M_PI * frequency * 1e-3)
    {

        // Print current joint angles
        const Vector3 joint_angles = leg->getCurrentJointAngles();
        printf("Joint angles: (%f, %f)\n", joint_angles(0), joint_angles(1));


        // Get point on circle
        const float x_off = radius * std::cos(t);
        const float y_off = radius * std::sin(t);

        // Create foot position vector
        Vector3 foot_position;
        foot_position << center_x + x_off, center_y + y_off, 0;

        // Set foot position to point on circle
        printf("Setting foot position to (%f, %f)\n", foot_position(0), foot_position(1));
        if (!leg->setFootPosition(foot_position))
            return 1;

        // Sleep for 1 ms
        usleep(1000);
    }

    // Go back to center position
    if (!leg->setFootPosition(Vector3(center_x, center_y, 0)))
        return 1;
    printf("Centered foot position.\n");

    // Wait for leg to move
    sleep(1);

    // Set axis state to idle
    if (!leg->setState(AxisState::IDLE))
        return 1;
    printf("Set axis state to idle.\n");

    printf("Done.\n");
    return 0;
}