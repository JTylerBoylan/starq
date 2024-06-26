#include <stdio.h>

#include "starq/leg_controller.hpp"
#include "starq/odrive/odrive_controller.hpp"
#include "starq/starq/starq_fivebar2d_leg_dynamics.hpp"

// #define LEG_LINK_1_LENGTH_M 0.05f
// #define LEG_LINK_2_LENGTH_M 0.150f

// Leg link lengths in meters (STARQ Leg)
#define LEG_LINK_1_LENGTH_M 0.065f
#define LEG_LINK_2_LENGTH_M 0.2f

#define GEAR_RATIO 6.0f

using namespace starq;
using namespace starq::can;
using namespace starq::odrive;

int main(void)
{
    printf("Hello world!\n");

    CANSocket::Ptr can_socket = std::make_shared<CANSocket>("can0");

    if (can_socket->connect())
    {
        printf("Connected to CAN interface.\n");
    }
    else
    {
        printf("Failed to connect to CAN interface.\n");
        return 1;
    }

    ODriveSocket::Ptr odrive_socket = std::make_shared<ODriveSocket>(can_socket);

    ODriveController::Ptr odrive_0 = std::make_shared<ODriveController>(odrive_socket, 0);
    ODriveController::Ptr odrive_1 = std::make_shared<ODriveController>(odrive_socket, 1);
    ODriveController::Ptr odrive_2 = std::make_shared<ODriveController>(odrive_socket, 2);
    ODriveController::Ptr odrive_3 = std::make_shared<ODriveController>(odrive_socket, 3);
    printf("Created ODrive controllers.\n");

    odrive_0->setGearRatio(GEAR_RATIO);
    odrive_1->setGearRatio(GEAR_RATIO);
    odrive_2->setGearRatio(GEAR_RATIO);
    odrive_3->setGearRatio(GEAR_RATIO);
    printf("Set gear ratios.\n");

    STARQFiveBar2DLegDynamics::Ptr fivebar_dynamics = std::make_shared<STARQFiveBar2DLegDynamics>(LEG_LINK_1_LENGTH_M,
                                                                                                  LEG_LINK_2_LENGTH_M);

    LegController::Ptr leg_F = std::make_shared<LegController>(fivebar_dynamics,
                                                               std::vector<MotorController::Ptr>{odrive_0, odrive_1});
    LegController::Ptr leg_B = std::make_shared<LegController>(fivebar_dynamics,
                                                               std::vector<MotorController::Ptr>{odrive_2, odrive_3});
    printf("Created leg controllers.\n");

    if (!leg_F->setState(AxisState::CLOSED_LOOP_CONTROL) ||
        !leg_B->setState(AxisState::CLOSED_LOOP_CONTROL))
        return 1;
    printf("Set axis states.\n");

    if (!leg_F->setControlMode(ControlMode::POSITION) ||
        !leg_B->setControlMode(ControlMode::POSITION))
        return 1;
    printf("Set control modes.\n");

    const float center_x = 0.0f;
    const float center_z = -0.150f;

    if (!leg_F->setFootPosition(Vector3(center_x, 0, center_z)) ||
        !leg_B->setFootPosition(Vector3(center_x, 0, center_z)))
        return 1;
    printf("Centering foot positions...\n");

    sleep(1);

    const int revolutions = 10;
    const float radius = 0.035f;

    printf("Starting leg movement in circle.\n");
    for (float t = 0.0f; t <= revolutions * 2.0f * M_PI + 0.01; t += 0.01f)
    {
        const float x_off = radius * std::cos(t);
        const float z_off = radius * std::sin(t);

        Vector3 foot_position;
        foot_position << center_x + x_off, 0, center_z + z_off;

        printf("Setting foot position to (%f, %f)\n", foot_position.x(), foot_position.z());
        if (!leg_F->setFootPosition(foot_position) ||
            !leg_B->setFootPosition(foot_position))
            return 1;

        usleep(2500);

        const Vector3 joint_angles_F = leg_F->getCurrentJointAngles();
        const Vector3 joint_angles_B = leg_B->getCurrentJointAngles();

        printf("Joint angles: F: %f, %f, B: %f, %f\n",
               joint_angles_F(0), joint_angles_F(1),
               joint_angles_B(0), joint_angles_B(1));
    }

    if (!leg_F->setFootPosition(Vector3(center_x, 0, center_z)) ||
        !leg_B->setFootPosition(Vector3(center_x, 0, center_z)))
        return 1;
    printf("Centered foot positions.\n");

    sleep(1);

    if (!leg_F->setState(AxisState::IDLE) ||
        !leg_B->setState(AxisState::IDLE))
        return 1;
    printf("Set axis states to idle.\n");

    printf("Done.\n");
    return 0;
}