#include <stdio.h>

#include "starq/leg_controller.hpp"
#include "starq/odrive/odrive_controller.hpp"
#include "starq/starq/starq_fivebar2d_leg_dynamics.hpp"

#define LEG_LINK_1_LENGTH_M 0.05f
#define LEG_LINK_2_LENGTH_M 0.150f

#define GEAR_RATIO_A 6.0f
#define GEAR_RATIO_B 6.0f

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

    ODriveController::Ptr odrive_A = std::make_shared<ODriveController>(odrive_socket, 0);
    ODriveController::Ptr odrive_B = std::make_shared<ODriveController>(odrive_socket, 1);
    printf("Created ODrive controllers.\n");

    odrive_A->setGearRatio(GEAR_RATIO_A);
    odrive_B->setGearRatio(GEAR_RATIO_B);
    printf("Set gear ratios.\n");

    STARQFiveBar2DLegDynamics::Ptr fivebar_dynamics = std::make_shared<STARQFiveBar2DLegDynamics>(LEG_LINK_1_LENGTH_M,
                                                                                                  LEG_LINK_2_LENGTH_M);

    LegController::Ptr leg = std::make_shared<LegController>(fivebar_dynamics,
                                                             std::vector<MotorController::Ptr>{odrive_A, odrive_B});
    printf("Created leg controller.\n");

    if (!leg->setState(AxisState::CLOSED_LOOP_CONTROL))
        return 1;

    printf("Set axis state to closed loop control.\n");

    if (!leg->setControlMode(ControlMode::TORQUE))
        return 1;

    printf("Set to force control mode.\n");

    const float force_x = 0.0f;
    const float force_y = -20.0f;
    printf("Applying Force: %f, %f\n", force_x, force_y);

    const Vector3 current_joint_angles = leg->getCurrentJointAngles();

    Matrix3 jacobian;
    fivebar_dynamics->getJacobian(current_joint_angles, jacobian);
    const Vector3 joint_torque = jacobian.transpose() * Vector3(force_x, force_y, 0);

    printf("Joint torque: %f, %f\n", joint_torque(0), joint_torque(1));

    const float duration = 10.0;   // seconds
    const float frequency = 100.0; // Hz
    for (float t = 0.0; t < duration; t += 1.0f / frequency)
    {
        if (!leg->setFootForce(Vector3(force_x, force_y, 0)))
            return 1;
        usleep(1E6 / frequency);
    }

    printf("\n");

    if (!leg->setState(AxisState::IDLE))
        return 1;

    printf("Set axis state to idle.\n");

    printf("Done.\n");
    return 0;
}