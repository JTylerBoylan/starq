#include <stdio.h>

#include "starq/odrive/odrive_controller.hpp"

using namespace starq;
using namespace starq::can;
using namespace starq::odrive;

#define CAN_ID 0

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

    ODriveController::Ptr odrive = std::make_shared<ODriveController>(odrive_socket, CAN_ID);

    if (!odrive->setState(AxisState::CLOSED_LOOP_CONTROL))
        return 1;
    printf("Set axis state to closed loop control.\n");

    if (!odrive->setControlMode(ControlMode::POSITION))
        return 1;
    printf("Set control mode to position control.\n");

    if (!odrive->setPosition(0.0f))
        return 1;
    printf("Set position to 0.\n");

    sleep(1);
    printf("--------------------\n");
    odrive->printInfo();

    const float position_increment = 0.2;
    for (float p = 0.0f; p <= 1.0f; p += position_increment)
    {
        printf("--------------------\n");
        printf("Setting position to %f\n", p);
        if (!odrive->setPosition(p))
            return 1;
        sleep(1);
        printf("--------------------\n");
        odrive->printInfo();
    }

    if (!odrive->setState(AxisState::IDLE))
        return 1;
    printf("Set axis state to idle.\n");

    printf("Done.\n");
    return 0;
}