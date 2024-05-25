#include <stdio.h>

#include "starq/starq/starq_robot.hpp"

using namespace starq;

int main(int argc, char **argv)
{

    // Check if the number of arguments is correct
    if (argc != 4)
    {
        printf("Usage: %s <p_gain> <v_gain> <vi_gain>\n", argv[0]);
        return 1;
    }

    // Parse arguments
    Float p_gain = std::stof(argv[1]);
    Float v_gain = std::stof(argv[2]);
    Float vi_gain = std::stof(argv[3]);

    // Create a STARQ robot
    STARQRobot::Ptr STARQ = std::make_shared<STARQRobot>();

    // Set the gains
    if (STARQ->setGains(p_gain, v_gain, vi_gain))
    {
        printf("Gains set to p: %f, v: %f, vi: %f\n", p_gain, v_gain, vi_gain);
    }
    else
    {
        printf("Failed to set gains\n");
    }

    return 0;
}