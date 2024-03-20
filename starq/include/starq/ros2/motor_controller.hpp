#ifndef STARQ_ROS2__MOTOR_CONTROLLER_HPP_
#define STARQ_ROS2__MOTOR_CONTROLLER_HPP_

#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>

#include "starq/motor_controller.hpp"
#include "starq/msg/motor_command.hpp"
#include "starq/msg/motor_state.hpp"
#include "starq/ros2/ros2_util.hpp"

#define MOTOR_CONTROLLER_STATE_PUBLISH_RATE 50

namespace starq::ros2
{

    /// @brief ROS2 wrapper for MotorController
    class MotorControllerROS2
    {
    public:
        using Ptr = std::shared_ptr<MotorControllerROS2>;

        /// @brief Create a new MotorControllerROS2 object
        /// @param node ROS2 node
        /// @param motor_controller MotorController object
        /// @param ns Namespace for the motor
        /// @param motor_name Name of the motor
        MotorControllerROS2(rclcpp::Node::SharedPtr node,
                            MotorController::Ptr motor_controller,
                            const std::string &ns,
                            const std::string &motor_name);

    protected:
        rclcpp::Node::SharedPtr node_;
        MotorController::Ptr motor_controller_;
        std::string ns_;
        std::string motor_name_;

    private:
        /// @brief Callback for motor command messages
        /// @param msg MotorCommand message
        void motorCommandCallback(const starq::msg::MotorCommand::SharedPtr msg);

        /// @brief Publish motor state
        void publishStateCallback();

        rclcpp::Subscription<starq::msg::MotorCommand>::SharedPtr motor_command_sub_;
        rclcpp::Publisher<starq::msg::MotorState>::SharedPtr motor_state_pub_;
        rclcpp::TimerBase::SharedPtr publish_state_timer_;
    };

}

#endif