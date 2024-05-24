#include <stdio.h>

#include "starq/starq/starq_robot.hpp"

using namespace starq;

int main(void)
{
    // Create a STARQ robot
    STARQRobot::Ptr STARQ = std::make_shared<STARQRobot>();

    // Ready motors
    STARQ->setStates(AxisState::CLOSED_LOOP_CONTROL);

    // Go to center position
    STARQ->goToDefaultFootLocations();

    // Wait for 500 ms
    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    // Idle motors
    STARQ->setStates(AxisState::IDLE);

    printf("Zeroed motors\n");
    return 0;
}