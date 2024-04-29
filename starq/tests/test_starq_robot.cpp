#include <stdio.h>

#include "starq/starq/starq_robot.hpp"

using namespace starq;

int main()
{

    // Create STARQ robot
    STARQRobot::Ptr STARQ = std::make_shared<STARQRobot>();
    printf("STARQRobot created\n");

    // Set axis state to closed loop control
    if (!STARQ->setState(AxisState::CLOSED_LOOP_CONTROL))
    {
        // Set axis state to idle on failure
        STARQ->setState(AxisState::IDLE);
        printf("Failed to set axis state.\n");
        return 1;
    }

    // Send legs to center position
    const Vector3 center_position(0.0f, -0.150f, 0.0f);
    for (uint32_t i = 0; i < 4; i++)
    {
        STARQ->setFootPosition(i, center_position);
    }

    // Wait for 5 seconds
    usleep(5E6);

    // Set axis state to idle
    STARQ->setState(AxisState::IDLE);

    printf("Done.\n");
    return 0;
}