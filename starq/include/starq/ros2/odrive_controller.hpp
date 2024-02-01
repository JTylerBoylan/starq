#ifndef STARQ_ROS2__ODRIVE_CONTROLLER_HPP_
#define STARQ_ROS2__ODRIVE_CONTROLLER_HPP_

#include <rclcpp/rclcpp.hpp>

#include "starq/odrive/odrive_controller.hpp"
#include "starq/ros2/motor_controller.hpp"
#include "starq/msg/o_drive_info.hpp"

#define ODRIVE_CONTROLLER_INFO_PUBLISH_RATE 20

namespace starq::ros2 
{

    class ODriveControllerROS2 : public MotorControllerROS2
    {

    public:
        using Ptr = std::shared_ptr<ODriveControllerROS2>;

        /// @brief Create a new ODriveControllerROS2 object
        /// @param node ROS2 node
        /// @param odrive_controller ODriveController object
        /// @param motor_name Name of the motor
        ODriveControllerROS2(rclcpp::Node::SharedPtr node,
                             starq::odrive::ODriveController::Ptr odrive_controller,
                             const std::string &motor_name);

    private:

        /// @brief Publish ODrive info
        void publishODriveInfoCallback();

        starq::odrive::ODriveController::Ptr odrive_controller_;
        rclcpp::Publisher<starq::msg::ODriveInfo>::SharedPtr odrive_info_pub_;
        rclcpp::TimerBase::SharedPtr publish_odrive_info_timer_;

    };

}

#endif