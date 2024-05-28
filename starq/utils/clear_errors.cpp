#include <stdio.h>

#include "starq/starq/starq_robot.hpp"

using namespace starq;

int main(void)
{
    printf("Clearing errors...\n");

    // Create a STARQ robot
    STARQRobot::Ptr STARQ = std::make_shared<STARQRobot>();

    // Loop through all motors
    for (auto motor : STARQ->getMotors())
    {
        // Cast to ODriveController
        auto odrv = std::dynamic_pointer_cast<odrive::ODriveController>(motor);

        // Clear errors
        odrv->clearErrors();
    }

    printf("Cleared errors\n");
    return 0;
}