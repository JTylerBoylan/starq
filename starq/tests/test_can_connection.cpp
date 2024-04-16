#include <stdio.h>

#include "starq/can/can_socket.hpp"

#define NUM_FRAMES 10

int main(void)
{
    printf("Hello world!\n");

    // Create a CAN socket object on the can0 interface
    starq::can::CANSocket socket("can0");

    // Connect to the CAN interface
    if (socket.connect())
    {
        printf("Connected to CAN interface.\n");
    }
    else
    {
        printf("Failed to connect to CAN interface.\n");
        return 1;
    }

    printf("Publishing %d can frames...\n", NUM_FRAMES);

    // Receive CAN frames
    struct can_frame frame;
    for (int i = 0; i < NUM_FRAMES; i++)
    {
        socket.receive(frame);
        printf("[%d] CAN Recieved: ID: %d, Size: %d\n", i, frame.can_id, frame.can_dlc);
    }

    return 0;
}