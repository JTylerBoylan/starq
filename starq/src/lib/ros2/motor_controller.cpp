#include "starq/ros2/motor_controller.hpp"

namespace starq::ros2
{

    MotorControllerROS2::MotorControllerROS2(rclcpp::Node::SharedPtr node,
                                             MotorController::Ptr motor_controller,
                                             const std::string &motor_name)
        : node_(node),
          motor_controller_(motor_controller),
          motor_name_(motor_name)
    {
        motor_command_sub_ = node_->create_subscription<starq::msg::MotorCommand>(
            "/starq/motor/" + motor_name_ + "/cmd", getFastQoS(),
            std::bind(&MotorControllerROS2::motorCommandCallback, this, std::placeholders::_1));

        motor_state_pub_ = node_->create_publisher<starq::msg::MotorState>(
            "/starq/motor" + motor_name_ + "/state", getFastQoS());

        publish_state_timer_ = node_->create_wall_timer(
            std::chrono::milliseconds(MOTOR_CONTROLLER_STATE_PUBLISH_RATE),
            std::bind(&MotorControllerROS2::publishStateCallback, this));
    }

    void MotorControllerROS2::motorCommandCallback(const starq::msg::MotorCommand::SharedPtr msg)
    {
        motor_controller_->setState(msg->axis_state);
        motor_controller_->setControlMode(msg->control_mode, msg->input_mode);

        const float input_position = msg->input_position;
        const float input_velocity = msg->input_velocity;
        const float input_torque = msg->input_torque;

        switch (msg->control_mode)
        {
            case starq::ControlMode::POSITION:
                if (!motor_controller_->setPosition(input_position, input_velocity, input_torque))
                {
                    RCLCPP_ERROR(node_->get_logger(), "Failed to set position for motor %s", motor_name_.c_str());
                }
                break;
            case starq::ControlMode::VELOCITY:
                if (!motor_controller_->setVelocity(input_velocity, input_torque))
                {
                    RCLCPP_ERROR(node_->get_logger(), "Failed to set velocity for motor %s", motor_name_.c_str());
                }
                break;
            case starq::ControlMode::TORQUE:
                if (!motor_controller_->setTorque(input_torque))
                {
                    RCLCPP_ERROR(node_->get_logger(), "Failed to set torque for motor %s", motor_name_.c_str());
                }
                break;
            default:
                break;
        }
    }

    void MotorControllerROS2::publishStateCallback()
    {
        starq::msg::MotorState msg;
        msg.position_estimate = motor_controller_->getPositionEstimate();
        msg.velocity_estimate = motor_controller_->getVelocityEstimate();
        msg.torque_estimate = motor_controller_->getTorqueEstimate();
        motor_state_pub_->publish(msg);
    }

}