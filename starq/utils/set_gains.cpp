#include <stdio.h>

#include "starq/starq/starq_robot.hpp"

using namespace starq;

void print_usage(char *name);
bool parse_args(int argc, char **argv, Float &p_gain, Float &v_gain, Float &vi_gain);

int main(int argc, char **argv)
{

    // Parse arguments
    Float p_gain, v_gain, vi_gain;
    if (!parse_args(argc, argv, p_gain, v_gain, vi_gain))
    {
        print_usage(argv[0]);
        return 1;
    }

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

void print_usage(char *name)
{
    printf("! Set the gains of the STARQ robot\n");
    printf("Usage: %s <p_gain> <v_gain> <vi_gain>\n", name);
    printf("  p_gain:  Proportional gain (required)\n");
    printf("  v_gain:  Velocity gain (required)\n");
    printf("  vi_gain: Integral velocity gain (required)\n");
    printf("! Example: %s 50 0.15 0.30\n", name);
}

bool parse_args(int argc, char **argv, Float &p_gain, Float &v_gain, Float &vi_gain)
{
    if (argc != 4)
    {
        return false;
    }

    try
    {
        p_gain = std::stof(argv[1]);
        v_gain = std::stof(argv[2]);
        vi_gain = std::stof(argv[3]);
    }
    catch(const std::invalid_argument& e)
    {
        printf("! ERROR: Invalid arguments\n");
        return false;
    }

    return true;
}