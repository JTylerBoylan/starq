#include <stdio.h>

#include "starq/starq/starq_robot.hpp"

using namespace starq;

int main(void)
{
    // Create a STARQ robot
    STARQRobot::Ptr STARQ = std::make_shared<STARQRobot>();

    // Set states to closed loop control
    for (auto motor : STARQ->getMotors())
    {
        motor->setState(AxisState::CLOSED_LOOP_CONTROL);
    }

    // Set positions to 0
    for (auto motor : STARQ->getMotors())
    {
        motor->setPosition(0.0);
    }

    // Wait for 1 second
    std::this_thread::sleep_for(std::chrono::seconds(1));

    // Idle
    for (auto motor : STARQ->getMotors())
    {
        motor->setState(AxisState::IDLE);
    }

    printf("Zeroed motors\n");
    return 0;
}