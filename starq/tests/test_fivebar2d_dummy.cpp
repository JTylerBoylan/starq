#include <stdio.h>
#include <unistd.h>

#include "starq/leg_controller.hpp"
#include "starq/testing/dummy_motor_controller.hpp"
#include "starq/starq/starq_fivebar2d_leg_dynamics.hpp"

// Leg link lengths in meters (ET-Quad Leg)
#define LEG_LINK_1_LENGTH_M 0.05f
#define LEG_LINK_2_LENGTH_M 0.150f

using namespace starq;
using namespace starq::testing;

int main(void)
{
    printf("Hello world!\n");

    // Dummy controllers do nothing but print out the commands they receive
    // Used for testing without physical hardware and/or debugging leg kinematics

    // Create two dummy motor controllers
    DummyMotorController::Ptr dummy_A = std::make_shared<DummyMotorController>(0);
    DummyMotorController::Ptr dummy_B = std::make_shared<DummyMotorController>(1);
    printf("Created ODrive controllers.\n");

    // Create LegDynamics object for five-bar 2D leg
    STARQFiveBar2DLegDynamics::Ptr fivebar_dynamics = std::make_shared<STARQFiveBar2DLegDynamics>(LEG_LINK_1_LENGTH_M,
                                                                                                  LEG_LINK_2_LENGTH_M);

    // Create LegController object from dynamics and dummy controllers
    LegController::Ptr leg = std::make_shared<LegController>(fivebar_dynamics,
                                                             std::vector<MotorController::Ptr>{dummy_A, dummy_B});
    printf("Created leg controller.\n");

    printf("Starting leg movement in circular path.\n");
    printf("\n");
    for (float t = 0.0f; t <= 2.0f * M_PI + 0.01; t += 0.1f)
    {

        // Move the foot in a circular path
        const float center_x = 0.0f;
        const float center_z = -std::sqrt(2) * 0.1;
        const float x_off = 0.025f * std::cos(t);
        const float z_off = 0.025f * std::sin(t);

        // Create the foot position vector
        Vector3 foot_position;
        foot_position << center_x + x_off, 0, center_z + z_off;

        // Set the foot position
        leg->setFootPosition(foot_position);
        printf("Setting foot position to (%f, %f)\n", foot_position.x(), foot_position.z());

        // Print the current foot position estimate
        Vector3 pos_estimate;
        leg->getFootPositionEstimate(pos_estimate);
        printf("Foot position estimate: (%f, %f)\n", pos_estimate.x(), pos_estimate.z());

        // Print the current motor positions
        Vector3 joint_angles = leg->getCurrentJointAngles();
        printf("Motor positions: (%f, %f)\n", joint_angles(0), joint_angles(1));

        // Test force control
        const float force = -5.0;
        printf("Setting foot force to (%f, %f)\n", 0.0, force);

        // Set the foot force
        leg->setFootForce(Vector3(0.0, 0.0, force));

        // Print the motor torques
        Vector3 motor_torques = leg->getCurrentJointTorques();
        printf("Motor torques: (%f, %f)\n", motor_torques(0), motor_torques(1));

        // Sleep for 5 ms
        usleep(5000);
        printf("\n");
    }

    printf("Done.\n");
    return 0;
}