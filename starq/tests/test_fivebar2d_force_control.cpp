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
    printf("Set axis state to closed loop control.\n");

    // Set control mode to force control
    if (!leg->setControlMode(ControlMode::TORQUE))
        return 1;
    printf("Set to force control mode.\n");

    // Apply a constant force at the end effector
    const float force_x = 0.0f;
    const float force_z = -20.0f;
    printf("Applying Force: %f, %f\n", force_x, force_z);

    // Get current joint torques
    const Vector3 current_joint_angles = leg->getCurrentJointAngles();

    // Print the joint torques required to apply the force
    // Use Jacobian to convert force to joint torques (tau = J^T * F)
    Matrix3 jacobian;
    fivebar_dynamics->getJacobian(current_joint_angles, jacobian);
    const Vector3 joint_torque = jacobian.transpose() * Vector3(force_x, 0, force_z);
    printf("Joint torque: %f, %f\n", joint_torque(0), joint_torque(1));

    // Parameters
    const float duration = 10.0;   // seconds
    const float frequency = 100.0; // Hz

    // Apply force for duration
    for (float t = 0.0; t < duration; t += 1.0f / frequency)
    {
        // Set foot force
        if (!leg->setFootForce(Vector3(force_x, 0, force_z)))
            return 1;

        // Match the loop frequency
        usleep(1E6 / frequency);
    }
    printf("\n");

    // Set axis state to idle
    if (!leg->setState(AxisState::IDLE))
        return 1;
    printf("Set axis state to idle.\n");

    printf("Done.\n");
    return 0;
}