#include <stdio.h>
#include <unistd.h>

#include "starq/testing/dummy_motor_controller.hpp"
#include "starq/dynamics/starq_fivebar2d.hpp"
#include "starq/leg_command_publisher.hpp"
#include "starq/trajectory_file_reader.hpp"

#define LEG_ID 0x0

#define MOTOR_ID_0 0x0
#define MOTOR_ID_1 0x1

#define LEG_LINK_1_LENGTH_M 0.05f
#define LEG_LINK_2_LENGTH_M 0.150f

using namespace starq;
using namespace starq::testing;
using namespace starq::dynamics;

int main(void)
{
    printf("Hello, world!\n");

    DummyMotorController::Ptr dummy_A = std::make_shared<DummyMotorController>(0);
    DummyMotorController::Ptr dummy_B = std::make_shared<DummyMotorController>(1);
    printf("Created ODrive controllers.\n");

    STARQ_FiveBar2D::Ptr fivebar_dynamics = std::make_shared<STARQ_FiveBar2D>(LEG_LINK_1_LENGTH_M,
                                                                              LEG_LINK_2_LENGTH_M);

    LegController::Ptr leg_FL = std::make_shared<LegController>(fivebar_dynamics,
                                                                std::vector<MotorController::Ptr>{dummy_A, dummy_B});
    printf("Created leg controller.\n");

    LegCommandPublisher::Ptr publisher = std::make_shared<LegCommandPublisher>(std::vector<LegController::Ptr>{leg_FL});
    printf("Created leg command publisher.\n");

    const float center_x = 0.0;
    const float center_y = -0.15;

    LegCommand::Ptr cmd = std::make_shared<LegCommand>();
    cmd->leg_id = LEG_ID;
    cmd->control_mode = 0x3;
    cmd->target_position = Vector2f(center_x, center_y);
    cmd->target_velocity = VectorXf::Zero(2);
    cmd->target_force = VectorXf::Zero(2);

    publisher->sendCommand(cmd);
    printf("Pushed leg command.\n");

    usleep(1000000);

    printf("Done.\n");

    return 0;
}