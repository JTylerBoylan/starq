#include <stdio.h>

#include "starq/leg_controller.hpp"
#include "starq/odrive/odrive_controller.hpp"
#include "starq/dynamics/starq_fivebar2d.hpp"

#define LEG_LINK_1_LENGTH_M 0.05f
#define LEG_LINK_2_LENGTH_M 0.150f

#define GEAR_RATIO_A 6.0f
#define GEAR_RATIO_B 6.0f

using namespace starq;
using namespace starq::can;
using namespace starq::odrive;
using namespace starq::dynamics;

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

    STARQ_FiveBar2D::Ptr fivebar_dynamics = std::make_shared<STARQ_FiveBar2D>(LEG_LINK_1_LENGTH_M,
                                                                              LEG_LINK_2_LENGTH_M);

    LegController::Ptr leg = std::make_shared<LegController>(fivebar_dynamics,
                                                             std::vector<MotorController::Ptr>{odrive_A, odrive_B});
    printf("Created leg controller.\n");

    if (!leg->setState(AxisState::CLOSED_LOOP_CONTROL))
        return 1;
    printf("Set axis state.\n");

    if (!leg->setControlMode(ControlMode::POSITION))
        return 1;
    printf("Set control mode.\n");

    const float center_x = 0.0f;
    const float center_y = -std::sqrt(2) * 0.1f;

    if (!leg->setFootPosition(Vector2f(center_x, center_y)))
        return 1;
    printf("Centered foot position.\n");

    const int revolutions = 10;
    const float radius = 0.025f;

    printf("Starting leg movement in sine wave.\n");
    for (float t = 0.0f; t <= revolutions * 2.0f * M_PI + 0.01; t += 0.01f)
    {
        const float x_off = radius * std::cos(t);
        const float y_off = radius * std::sin(t);

        VectorXf foot_position(2);
        foot_position << center_x + x_off, center_y + y_off;

        printf("Setting foot position to (%f, %f)\n", foot_position(0), foot_position(1));
        if (!leg->setFootPosition(foot_position))
            return 1;

        usleep(2500);

        const VectorXf joint_angles = leg->getCurrentJointAngles();
        printf("Joint angles: (%f, %f)\n", joint_angles(0), joint_angles(1));
    }

    if (!leg->setFootPosition(Vector2f(center_x, center_y)))
        return 1;
    printf("Centered foot position.\n");

    sleep(1);

    if (!leg->setState(AxisState::IDLE))
        return 1;
    printf("Set axis state to idle.\n");

    printf("Done.\n");
    return 0;
}