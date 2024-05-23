#include <stdio.h>
#include <unistd.h>

#include "starq/testing/dummy_motor_controller.hpp"
#include "starq/starq/starq_fivebar2d_leg_dynamics.hpp"
#include "starq/leg_command_publisher.hpp"
#include "starq/trajectory_controller.hpp"

// Leg ID
#define LEG_ID 0x0

// Leg link lengths in meters (ET-Quad Leg)
#define LEG_LINK_1_LENGTH_M 0.05f
#define LEG_LINK_2_LENGTH_M 0.150f

using namespace starq;
using namespace starq::testing;

int main(void)
{
    printf("Hello, world!\n");

    // Create dummy motor controllers
    DummyMotorController::Ptr dummy_A = std::make_shared<DummyMotorController>(0);
    DummyMotorController::Ptr dummy_B = std::make_shared<DummyMotorController>(1);
    printf("Created ODrive controllers.\n");

    // Create leg dynamics object for five-bar 2D leg
    STARQFiveBar2DLegDynamics::Ptr fivebar_dynamics = std::make_shared<STARQFiveBar2DLegDynamics>(LEG_LINK_1_LENGTH_M,
                                                                                                  LEG_LINK_2_LENGTH_M);

    // Create leg controller object from dynamics and dummy controllers
    LegController::Ptr leg_FL = std::make_shared<LegController>(fivebar_dynamics,
                                                                std::vector<MotorController::Ptr>{dummy_A, dummy_B});
    printf("Created leg controller.\n");

    // Create leg command publisher (runs in separate thread)
    LegCommandPublisher::Ptr publisher = std::make_shared<LegCommandPublisher>(std::vector<LegController::Ptr>{leg_FL});
    printf("Created leg command publisher.\n");

    // Go to center position
    const float center_x = 0.0;
    const float center_y = -0.15;

    // Create leg command
    LegCommand::Ptr cmd = std::make_shared<LegCommand>();
    cmd->leg_id = LEG_ID;
    cmd->control_mode = 0x3;
    cmd->target_position = Vector3(center_x, center_y, 0);
    cmd->target_velocity = Vector3::Zero();
    cmd->target_force = Vector3::Zero();

    // Send leg command
    publisher->sendCommand(cmd);
    printf("Pushed leg command.\n");

    // Wait for a second
    usleep(1000000);
    printf("Done.\n");
    return 0;
}