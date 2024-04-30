#include <stdio.h>

#include "starq/starq/starq_robot.hpp"

using namespace starq;

int main()
{

    // Create STARQ robot
    STARQRobot::Ptr STARQ = std::make_shared<STARQRobot>();
    printf("STARQRobot created\n");

    // Loop through all motors
    for (auto motor : STARQ->getMotors())
    {
        // Cast as odrive motor
        auto odrive_motor = std::dynamic_pointer_cast<odrive::ODriveController>(motor);

        // Print motor info
        printf("-----------------------");
        odrive_motor->printInfo();
    }
    printf("-------------------------");

    printf("Done.\n");
    return 0;
}