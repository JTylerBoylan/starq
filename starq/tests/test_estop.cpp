#include <stdio.h>

#include "starq/starq/starq_robot.hpp"

using namespace starq;

int main()
{

    // Create STARQ robot
    STARQRobot::Ptr STARQ = std::make_shared<STARQRobot>();
    printf("STARQRobot created\n");

    // Sleep for 5 seconds
    // Ctrl+C to stop the program and test the emergency stop
    usleep(5000000);

    printf("Done.\n");
    return 0;
}