#include <stdio.h>
#include <thread>

#include "starq/ros2/ros2_mpc_joystick.hpp"
#include "starq/unitree/unitree_a1_mujoco_robot.hpp"

using namespace starq;
using namespace starq::mpc;
using namespace starq::unitree;

int main(int argc, char **argv)
{
    // Initialize ROS2
    rclcpp::init(argc, argv);

    // Create node for ROS2 components
    // Use RCLCPP_INFO instead of printf for logging in ROS2
    auto node = std::make_shared<rclcpp::Node>("test_joystick_node");
    RCLCPP_INFO(node->get_logger(), "Node created\n");

    // Create Unitree A1 robot
    auto robot = std::make_shared<UnitreeA1MuJoCoRobot>();
    RCLCPP_INFO(node->get_logger(), "UnitreeA1MuJoCoRobot created\n");

    // Set MPC parameters
    auto mpc_config = robot->getMPCConfiguration();
    mpc_config->setTimeStep(milliseconds(50));
    mpc_config->setWindowSize(21);

    // Create ROS2 Joystick
    // This maps the ROS2 Joy inputs to MPC commands
    auto joystick = std::make_shared<ros2::ROS2MPCJoystick>(node, mpc_config);
    RCLCPP_INFO(node->get_logger(), "ROS2 Joystick created\n");

    // Set MuJoCo frame rate
    MuJoCo::getInstance()->setFrameRate(30.0);

    // Start simulation
    robot->startSimulation();
    RCLCPP_INFO(node->get_logger(), "Simulation started\n");

    // Send legs to default positions
    for (uint8_t id = 0; id < UNITREE_A1_NUM_LEGS; id++)
    {
        robot->setFootPosition(id, robot->getRobotParameters()->getDefaultFootLocations()[id]);
    }

    // Wait for simulation to settle
    RCLCPP_INFO(node->get_logger(), "Holding foot position for 5 seconds...\n");
    std::this_thread::sleep_for(std::chrono::seconds(5));

    // Start MPC
    RCLCPP_INFO(node->get_logger(), "Starting MPC");
    if (!robot->startMPC())
    {
        RCLCPP_INFO(node->get_logger(), "Failed to start MPC.\n");
        return 1;
    }

    // Start ROS2 spin thread
    RCLCPP_INFO(node->get_logger(), "Starting spin thread");
    std::thread spin_thread([&]()
                            { rclcpp::spin(node); });

    while (robot->isSimulationOpen())
    {
        // Print MPC frequency
        const Float freq = robot->getMPCController()->getFrequency();
        RCLCPP_INFO(node->get_logger(), "MPC frequency: %f", freq);

        // Sleep for 500 ms
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }

    // Stop MPC
    robot->stopMPC();

    // Stop spinning
    spin_thread.join();

    // Shutdown ROS2
    rclcpp::shutdown();

    printf("Done.\n");
    return 0;
}