#include <stdio.h>

#include "starq/odrive/odrive_controller.hpp"

using namespace starq;
using namespace starq::can;
using namespace starq::odrive;

int main(void)
{
    printf("Hello world!\n");

    // Create a CAN socket object on the can0 interface
    CANSocket::Ptr can_socket = std::make_shared<CANSocket>("can0");

    // Connect to the CAN interface
    if (can_socket->connect())
    {
        printf("Connected to CAN interface.\n");
    }
    else
    {
        printf("Failed to connect to CAN interface.\n");
        return 1;
    }

    // Create an ODrive socket object using the CAN socket
    ODriveSocket::Ptr odrive_socket = std::make_shared<ODriveSocket>(can_socket);

    // Loop through all motor IDs and clear errors
    for (uint8_t i = 0; i < MAX_MOTOR_ID; i++)
    {

        // Create an ODrive controller object for the current motor ID
        ODriveController::Ptr odrive = std::make_shared<ODriveController>(odrive_socket, i);

        // Clear errors
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