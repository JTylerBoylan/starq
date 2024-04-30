#include <stdio.h>

#include "starq/odrive/odrive_controller.hpp"

using namespace starq;
using namespace starq::can;
using namespace starq::odrive;

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
    for (uint8_t i = 0; i < MAX_MOTOR_ID; i++)
    {

        // Create an ODrive controller object for the current motor ID
        ODriveController::Ptr odrive_0 = std::make_shared<ODriveController>(odrive_socket_0, i);
        ODriveController::Ptr odrive_1 = std::make_shared<ODriveController>(odrive_socket_1, i);

        // Clear errors
        if (!odrive_0->clearErrors() || !odrive_1->clearErrors())
        {
            printf("Failed to clear errors for axis %d.\n", i);
            return 1;
        }
    }
    printf("Cleared errors for all axes.\n");

    printf("Done.\n");
    return 0;
}