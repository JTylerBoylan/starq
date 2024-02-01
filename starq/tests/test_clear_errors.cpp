#include <stdio.h>

#include "starq/odrive/odrive_controller.hpp"

using namespace starq;
using namespace starq::can;
using namespace starq::odrive;

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

    for (uint8_t i = 0; i < MAX_MOTOR_ID; i++)
    {
        ODriveController::Ptr odrive = std::make_shared<ODriveController>(odrive_socket, i);

        if (!odrive->clearErrors())
        {
            printf("Failed to clear errors for axis %d.\n", i);
            return 1;
        }
    }

    printf("Cleared errors for all axes.\n");

    printf("Done.\n");
    return 0;
}