#ifndef STARQ_ROS2__ROS2_JOYSTICK_HPP_
#define STARQ_ROS2__ROS2_JOYSTICK_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>

#include "starq/ros2/ros2_util.hpp"
#include "starq/mpc/mpc_configuration.hpp"

namespace starq::ros2
{

    class ROS2Joystick
    {

    public:
        using Ptr = std::shared_ptr<ROS2Joystick>;

        ROS2Joystick(rclcpp::Node::SharedPtr node, mpc::MPCConfiguration::Ptr mpc_configuration);

    private:
        void joyCallback(const sensor_msgs::msg::Joy::SharedPtr msg);

        rclcpp::Node::SharedPtr node_;
        rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
        mpc::MPCConfiguration::Ptr mpc_configuration_;

        mpc::Gait::Ptr stand_gait_;
        mpc::Gait::Ptr walk_gait_;
        mpc::Gait::Ptr crawl_gait_;

        mpc::Gait::Ptr current_gait_;

        Vector3 base_position_;
        Vector3 base_orientation_;
    };

};

#endif