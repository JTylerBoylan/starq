#include <stdio.h>

#include "starq/can/can_socket.hpp"

#define NUM_FRAMES 10

int main(void)
{
    printf("Hello world!\n");

    starq::can::CANSocket socket("can0");

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

    struct can_frame frame;
    for (int i = 0; i < NUM_FRAMES; i++)
    {
        socket.receive(frame);
        printf("[%d] CAN Recieved: ID: %d, Size: %d\n", i, frame.can_id, frame.can_dlc);
    }

    return 0;
}