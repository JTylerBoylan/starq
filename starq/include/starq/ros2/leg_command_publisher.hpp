#ifndef STARQ_ROS2__LEG_COMMAND_PUBLISHER_HPP_
#define STARQ_ROS2__LEG_COMMAND_PUBLISHER_HPP_

#include <rclcpp/rclcpp.hpp>

#include "starq/leg_command_publisher.hpp"
#include "starq/msg/leg_command.hpp"
#include "starq/ros2/ros2_util.hpp"

namespace starq::ros2
{

    class LegCommandPublisherROS2
    {

    public:
        using Ptr = std::shared_ptr<LegCommandPublisherROS2>;

        /// @brief Create a new LegCommandPublisherROS2 object
        /// @param node ROS2 node
        /// @param leg_command_publisher LegCommandPublisher object
        /// @param ns Namespace for the leg
        /// @param leg_names Names of the legs
        LegCommandPublisherROS2(rclcpp::Node::SharedPtr node,
                                LegCommandPublisher::Ptr leg_command_publisher,
                                const std::string &ns,
                                const std::vector<std::string> &leg_names);

    private:

        void legCommandCallback(const std::string &leg_name, const starq::msg::LegCommand::SharedPtr msg);

        rclcpp::Node::SharedPtr node_;
        LegCommandPublisher::Ptr leg_command_publisher_;
        std::string ns_;
        std::vector<std::string> leg_names_;

        rclcpp::Subscription<starq::msg::LegCommand>::SharedPtr leg_command_sub_;

    };
}

#endif