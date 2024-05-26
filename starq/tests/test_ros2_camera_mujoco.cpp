#include <stdio.h>
#include <thread>

#include "starq/ros2/ros2_mpc_joystick.hpp"
#include "starq/unitree/unitree_a1_mujoco_robot.hpp"
#include "starq/ros2/ros2_mujoco_camera.hpp"

using namespace starq;
using namespace starq::mpc;
using namespace starq::unitree;

int main(int argc, char **argv)
{
    // Initialize ROS2
    rclcpp::init(argc, argv);

    // Create node for ROS2 components
    // Use RCLCPP_INFO instead of printf for logging in ROS2
    auto node = std::make_shared<rclcpp::Node>("test_joystick_camera_node");
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

    // Create ROS2 MuJoCo Camera
    // This publishes the camera image to a ROS2 topic
    // Can be used for SLAM or other vision-based tasks
    auto front_camera = std::make_shared<ros2::ROS2MuJoCoCamera>(node, robot->getFrontCamera(), "/front_camera/image_raw");
    RCLCPP_INFO(node->get_logger(), "ROS2 camera created\n");

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

    // Load stand gait from file
    Gait::Ptr stand_gait = std::make_shared<Gait>();
    stand_gait->load("/home/nvidia/starq_ws/src/starq/gaits/stand.txt");
    printf("Stand Gait loaded\n");

    // Stand still at default height
    stand_gait->setPose(Vector3(0, 0, UNITREE_A1_STAND_HEIGHT), Vector3(0, 0, 0));
    stand_gait->setFrequency(10.0);

    // Start MPC
    robot->runMPCGait(stand_gait);
    printf("MPC started\n");

    // Start ROS2 spin thread
    RCLCPP_INFO(node->get_logger(), "Starting spin thread");
    std::future<void> sim = std::async(std::launch::async, [&]()
                                       { robot->waitForSimulation(); });
    std::thread spin_thread([&]()
                            { rclcpp::spin_until_future_complete(node, sim); });

    while (robot->isSimulationOpen())
    {
        // Print MPC frequency
        const Float freq = robot->getMPCController()->getFrequency();
        RCLCPP_INFO(node->get_logger(), "MPC frequency: %f", freq);

        // Sleep for 500 ms
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }

    // Wait for the spin thread to finish
    spin_thread.join();

    // Stop MPC
    robot->stopMPC();

    // Shutdown ROS2
    rclcpp::shutdown();

    printf("Done.\n");
    return 0;
}