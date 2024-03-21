#include <stdio.h>
#include <thread>

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

    auto &sim = robot->startSimulation();

    for (uint8_t id = 0; id < UNITREE_A1_NUM_LEGS; id++)
    {
        robot->setFootPosition(id, robot->getRobotDynamics()->getDefaultFootLocations()[id]);
    }
    RCLCPP_INFO(node->get_logger(), "Holding foot position for 5 seconds...\n");
    std::this_thread::sleep_for(std::chrono::seconds(5));

    RCLCPP_INFO(node->get_logger(), "Starting MPC");
    if (!robot->startMPC())
    {
        RCLCPP_INFO(node->get_logger(), "Failed to start MPC.\n");
        return 1;
    }

    RCLCPP_INFO(node->get_logger(), "Starting spin thread");
    std::thread spin_thread([&]()
                            { rclcpp::spin_until_future_complete(node, sim); });

    while (robot->isSimulationOpen())
    {
        Float freq = robot->getMPCController()->getFrequency();
        RCLCPP_INFO(node->get_logger(), "MPC frequency: %f", freq);
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }

    // Wait for the spin thread to finish
    spin_thread.join();
    robot->stopMPC();
    rclcpp::shutdown();
    return 0;
}