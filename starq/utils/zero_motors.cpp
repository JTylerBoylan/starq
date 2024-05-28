#include <stdio.h>

#include "starq/starq/starq_robot.hpp"

using namespace starq;

int main(void)
{
    // Create a STARQ robot
    STARQRobot::Ptr STARQ = std::make_shared<STARQRobot>();

    // Ready motors
    STARQ->setStates(AxisState::CLOSED_LOOP_CONTROL);

    // Wait for 100ms
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    // Loop through all motors
    for (auto motor : STARQ->getMotors())
    {
        // Set motor position to 0
        motor->setPosition(0.0);
    }

    // Wait for 1000 ms
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));

    // Idle motors
    STARQ->setStates(AxisState::IDLE);

    printf("Zeroed motors\n");
    return 0;
}