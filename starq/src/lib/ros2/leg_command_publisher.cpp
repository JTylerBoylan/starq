#include "starq/ros2/leg_command_publisher.hpp"

namespace starq::ros2
{

    LegCommandPublisherROS2::LegCommandPublisherROS2(rclcpp::Node::SharedPtr node,
                                                     LegCommandPublisher::Ptr leg_command_publisher,
                                                     const std::string &ns,
                                                     const std::vector<std::string> &leg_names)
        : node_(node),
          leg_command_publisher_(leg_command_publisher),
          ns_(ns),
          leg_names_(leg_names)
    {
        for (const auto &leg_name : leg_names_)
        {
            leg_command_sub_ = node_->create_subscription<starq::msg::LegCommand>(
                "/" + ns_ + "/leg_command_publisher/" + leg_name + "/cmd", getFastQoS(),
                [this, leg_name](const starq::msg::LegCommand::SharedPtr msg)
                {
                    this->legCommandCallback(leg_name, msg);
                });
        }
    }

    void LegCommandPublisherROS2::legCommandCallback(const std::string &leg_name, const starq::msg::LegCommand::SharedPtr msg)
    {
        const size_t leg_index = std::find(leg_names_.begin(), leg_names_.end(), leg_name) - leg_names_.begin();
        if (leg_index >= leg_names_.size())
        {
            RCLCPP_ERROR(node_->get_logger(), "Invalid leg name %s", leg_name.c_str());
            return;
        }

        LegCommand::Ptr leg_command = std::make_shared<LegCommand>();
        leg_command->leg_id = leg_index;
        leg_command->control_mode = msg->control_mode;
        leg_command->input_mode = msg->input_mode;
        ros2eigen(msg->input_position, leg_command->target_position);
        ros2eigen(msg->input_velocity, leg_command->target_velocity);
        ros2eigen(msg->input_force, leg_command->target_force);

        leg_command_publisher_->sendCommand(leg_command);
    }

}