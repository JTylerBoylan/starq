#include "starq/ros2/odrive_controller.hpp"

namespace starq::ros2
{

    ODriveControllerROS2::ODriveControllerROS2(rclcpp::Node::SharedPtr node,
                                               starq::odrive::ODriveController::Ptr odrive_controller,
                                               const std::string &motor_name)
        : MotorControllerROS2(node, odrive_controller, motor_name),
          odrive_controller_(odrive_controller)
    {
        odrive_info_pub_ = node_->create_publisher<starq::msg::ODriveInfo>(
            "/starq/motor/" + motor_name_ + "/odrive_info", getFastQoS());

        publish_odrive_info_timer_ = node_->create_wall_timer(
            std::chrono::milliseconds(ODRIVE_CONTROLLER_INFO_PUBLISH_RATE),
            std::bind(&ODriveControllerROS2::publishODriveInfoCallback, this));
    }

    void ODriveControllerROS2::publishODriveInfoCallback()
    {
        starq::msg::ODriveInfo msg;
        msg.axis_state = odrive_controller_->getAxisState();
        msg.axis_error = odrive_controller_->getAxisError();
        msg.iq_setpoint = odrive_controller_->getIqSetpoint();
        msg.iq_measured = odrive_controller_->getIqMeasured();
        msg.fet_temperature = odrive_controller_->getFETTemperature();
        msg.motor_temperature = odrive_controller_->getMotorTemperature();
        msg.dc_bus_voltage = odrive_controller_->getBusVoltage();
        msg.dc_bus_current = odrive_controller_->getBusCurrent();
        odrive_info_pub_->publish(msg);
    }

}