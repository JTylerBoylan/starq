#include "starq/ros2/leg_controller.hpp"

namespace starq::ros2
{

    LegControllerROS2::LegControllerROS2(rclcpp::Node::SharedPtr node,
                                         LegController::Ptr leg_controller,
                                         const std::string &leg_name)
        : node_(node),
          leg_controller_(leg_controller),
          leg_name_(leg_name)
    {
        leg_command_sub_ = node_->create_subscription<starq::msg::LegCommand>(
            "/starq/leg/" + leg_name_ + "/cmd", getFastQoS(),
            std::bind(&LegControllerROS2::legCommandCallback, this, std::placeholders::_1));

        leg_state_pub_ = node_->create_publisher<starq::msg::LegState>(
            "/starq/leg/" + leg_name_ + "/state", getFastQoS());

        publish_state_timer_ = node_->create_wall_timer(
            std::chrono::milliseconds(1000 / LEG_CONTROLLER_STATE_PUBLISH_RATE),
            std::bind(&LegControllerROS2::publishStateCallback, this));
    }

    void LegControllerROS2::legCommandCallback(const starq::msg::LegCommand::SharedPtr msg)
    {
        Vector3f foot_position, foot_velocity, foot_force;
        ros2eigen(msg->input_position, foot_position);
        ros2eigen(msg->input_velocity, foot_velocity);
        ros2eigen(msg->input_force, foot_force);

        leg_controller_->setControlMode(msg->control_mode, msg->input_mode);

        switch (msg->control_mode)
        {
        case ControlMode::POSITION:
            leg_controller_->setFootPosition(foot_position, foot_velocity, foot_force);
            break;
        case ControlMode::VELOCITY:
            leg_controller_->setFootVelocity(foot_velocity, foot_force);
            break;
        case ControlMode::TORQUE:
            leg_controller_->setFootForce(foot_force);
            break;
        }
    }

    void LegControllerROS2::publishStateCallback()
    {
        VectorXf foot_position, foot_velocity, foot_force;
        if (!leg_controller_->getFootPositionEstimate(foot_position))
        {
            RCLCPP_ERROR(node_->get_logger(), "Failed to get foot position estimate for leg %s", leg_name_.c_str());
            return;
        }
        if (!leg_controller_->getFootVelocityEstimate(foot_velocity))
        {
            RCLCPP_ERROR(node_->get_logger(), "Failed to get foot velocity estimate for leg %s", leg_name_.c_str());
            return;
        }
        if (!leg_controller_->getFootForceEstimate(foot_force))
        {
            RCLCPP_ERROR(node_->get_logger(), "Failed to get foot force estimate for leg %s", leg_name_.c_str());
            return;
        }

        geometry_msgs::msg::Vector3 foot_position_ros, foot_velocity_ros, foot_force_ros;
        eigen2ros(foot_position, foot_position_ros);
        eigen2ros(foot_velocity, foot_velocity_ros);
        eigen2ros(foot_force, foot_force_ros);

        starq::msg::LegState msg;
        msg.position_estimate = foot_position_ros;
        msg.velocity_estimate = foot_velocity_ros;
        msg.force_estimate = foot_force_ros;
        leg_state_pub_->publish(msg);
    }

}