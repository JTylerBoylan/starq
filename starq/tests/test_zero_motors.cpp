#include <stdio.h>

#include "starq/odrive/odrive_controller.hpp"

using namespace starq;
using namespace starq::can;
using namespace starq::odrive;

#define MAX_ODRIVE_ID 12

int main(void)
{
    printf("Hello world!\n");

    // Create a CAN socket object on the can0 interface
    CANSocket::Ptr can_socket_0 = std::make_shared<CANSocket>("can0");
    CANSocket::Ptr can_socket_1 = std::make_shared<CANSocket>("can1");

    // Connect to the CAN interface
    if (can_socket_0->connect() && can_socket_1->connect())
    {
        printf("Connected to CAN interface.\n");
    }
    else
    {
        printf("Failed to connect to CAN interface.\n");
        return 1;
    }

    // Create an ODrive socket object using the CAN socket
    ODriveSocket::Ptr odrive_socket_0 = std::make_shared<ODriveSocket>(can_socket_0);
    ODriveSocket::Ptr odrive_socket_1 = std::make_shared<ODriveSocket>(can_socket_1);

    // Loop through all motor IDs and clear errors
    for (uint8_t i = 0; i < MAX_ODRIVE_ID; i++)
    {
        // Clear errors
        if (!odrive_socket_0->setPosition(i, 0.0) ||
            !odrive_socket_1->setPosition(i, 0.0))
        {
            printf("Failed to clear errors for axis %d.\n", i);
        }
    }
    printf("Cleared errors for all axes.\n");

    printf("Done.\n");
    return 0;
}