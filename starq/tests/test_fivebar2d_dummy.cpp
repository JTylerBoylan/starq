#include <stdio.h>
#include <unistd.h>

#include "starq/leg_controller.hpp"
#include "starq/testing/dummy_motor_controller.hpp"
#include "starq/dynamics/starq_fivebar2d.hpp"

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
    printf("Hello world!\n");

    DummyMotorController::Ptr dummy_A = std::make_shared<DummyMotorController>(0);
    DummyMotorController::Ptr dummy_B = std::make_shared<DummyMotorController>(1);
    printf("Created ODrive controllers.\n");

    STARQ_FiveBar2D::Ptr fivebar_dynamics = std::make_shared<STARQ_FiveBar2D>(LEG_LINK_1_LENGTH_M,
                                                                              LEG_LINK_2_LENGTH_M);

    LegController::Ptr leg = std::make_shared<LegController>(fivebar_dynamics,
                                                             std::vector<MotorController::Ptr>{dummy_A, dummy_B});
    printf("Created leg controller.\n");

    printf("Starting leg movement in sine wave.\n");
    printf("\n");
    for (float t = 0.0f; t <= 2.0f * M_PI + 0.01; t += 0.1f)
    {
        const float center_x = 0.0f;
        const float center_y = -std::sqrt(2) * 0.1;

        const float x_off = 0.025f * std::cos(t);
        const float y_off = 0.025f * std::sin(t);

        Vector3 foot_position;
        foot_position << center_x + x_off, center_y + y_off, 0;

        printf("Setting foot position to (%f, %f)\n", foot_position(0), foot_position(1));

        leg->setFootPosition(foot_position);

        Vector3 pos_estimate;
        leg->getFootPositionEstimate(pos_estimate);

        printf("Foot position estimate: (%f, %f)\n", pos_estimate(0), pos_estimate(1));

        Vector3 joint_angles = leg->getCurrentJointAngles();

        printf("Motor positions: (%f, %f)\n", joint_angles(0), joint_angles(1));

        const float force = -5.0;

        printf("Setting foot force to (%f, %f)\n", 0.0, force);

        leg->setFootForce(Vector3(0.0, force, 0.0));

        Vector3 motor_torques = leg->getCurrentJointTorques();

        printf("Motor torques: (%f, %f)\n", motor_torques(0), motor_torques(1));

        printf("\n");

        usleep(5000);
    }

    return 0;
}