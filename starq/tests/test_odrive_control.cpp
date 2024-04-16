#include <stdio.h>

#include "starq/odrive/odrive_controller.hpp"

using namespace starq;
using namespace starq::can;
using namespace starq::odrive;

#define CAN_ID 0

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

    // Create an ODrive controller object for CAN_ID
    ODriveController::Ptr odrive = std::make_shared<ODriveController>(odrive_socket, CAN_ID);

    // Switch the state to closed loop control
    if (!odrive->setState(AxisState::CLOSED_LOOP_CONTROL))
        return 1;
    printf("Set axis state to closed loop control.\n");

    // Set the control mode to position control
    if (!odrive->setControlMode(ControlMode::POSITION))
        return 1;
    printf("Set control mode to position control.\n");

    // Go to position 0
    if (!odrive->setPosition(0.0f))
        return 1;
    printf("Set position to 0.\n");

    // Wait to let the motor move
    sleep(1);
    printf("--------------------\n");

    // Print the ODrive info
    odrive->printInfo();

    // Move the motor to different positions
    const float position_increment = 0.2;
    for (float p = 0.0f; p <= 1.0f; p += position_increment)
    {
        printf("--------------------\n");
        printf("Setting position to %f\n", p);

        // Send the position command
        if (!odrive->setPosition(p))
            return 1;

        // Wait to let the motor move
        sleep(1);
        printf("--------------------\n");

        // Print the ODrive info
        odrive->printInfo();
    }

    // Switch the state back to idle
    if (!odrive->setState(AxisState::IDLE))
        return 1;
    printf("Set axis state to idle.\n");

    printf("Done.\n");
    return 0;
}