#include <stdio.h>

#include "starq/ros2/ros2_joystick.hpp"
#include "starq/unitree/unitree_a1_mujoco_robot.hpp"

using namespace starq;
using namespace starq::mpc;
using namespace starq::unitree;

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<rclcpp::Node>("test_joystick_node");

    auto robot = std::make_shared<UnitreeA1MuJoCoRobot>();

    auto mpc_config = robot->getMPCConfiguration();

    auto joystick = std::make_shared<ros2::ROS2Joystick>(node, mpc_config);

    robot->startSimulation();

    for (uint8_t id = 0; id < UNITREE_A1_NUM_LEGS; id++)
    {
        robot->setFootPosition(id, robot->getRobotDynamics()->getDefaultFootLocations()[id]);
    }
    printf("Holding foot position for 5 seconds...\n");
    std::this_thread::sleep_for(std::chrono::seconds(5));

    printf("Starting MPC...\n");
    if (!robot->startMPC())
    {
        printf("Failed to start MPC.\n");
        return 1;
    }

    printf("Starting joystick...\n");
    rclcpp::spin(node);

    return 0;
}