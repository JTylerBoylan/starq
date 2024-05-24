#include <stdio.h>

#include "starq/starq/starq_robot.hpp"

using namespace starq;

int main(void)
{
    // Create a STARQ robot
    STARQRobot::Ptr STARQ = std::make_shared<STARQRobot>();

    printf("Motor info:\n");

    // Loop through all motors
    for (auto motor : STARQ->getMotors())
    {
        // Cast to ODriveController
        auto odrv = std::dynamic_pointer_cast<odrive::ODriveController>(motor);

        // Print info
        odrv->printInfo();
    }
    
    return 0;
}