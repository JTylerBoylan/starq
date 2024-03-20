#ifndef STARQ__ROS2__LEG_CONTROLLER_HPP_
#define STARQ__ROS2__LEG_CONTROLLER_HPP_

#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>

#include "starq/leg_controller.hpp"
#include "starq/msg/leg_command.hpp"
#include "starq/msg/leg_state.hpp"
#include "starq/ros2/ros2_util.hpp"

#define LEG_CONTROLLER_STATE_PUBLISH_RATE 50

namespace starq::ros2
{

    class LegControllerROS2
    {

    public:
        using Ptr = std::shared_ptr<LegControllerROS2>;

        /// @brief Create a new LegControllerROS2 object
        /// @param node ROS2 node
        /// @param leg_controller LegController object
        /// @param ns Namespace for the leg
        /// @param leg_name Name of the leg
        LegControllerROS2(rclcpp::Node::SharedPtr node,
                          LegController::Ptr leg_controller,
                          const std::string &ns,
                          const std::string &leg_name);

    private:
        /// @brief Callback for leg command messages
        /// @param msg LegCommand message
        void legCommandCallback(const starq::msg::LegCommand::SharedPtr msg);

        /// @brief Publish leg state
        void publishStateCallback();

        rclcpp::Node::SharedPtr node_;
        LegController::Ptr leg_controller_;
        std::string ns_;
        std::string leg_name_;

        rclcpp::Subscription<starq::msg::LegCommand>::SharedPtr leg_command_sub_;
        rclcpp::Publisher<starq::msg::LegState>::SharedPtr leg_state_pub_;
        rclcpp::TimerBase::SharedPtr publish_state_timer_;
    };

}

#endif