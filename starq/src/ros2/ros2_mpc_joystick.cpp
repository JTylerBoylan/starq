#include "starq/ros2/ros2_mpc_joystick.hpp"

namespace starq::ros2
{

    ROS2MPCJoystick::ROS2MPCJoystick(rclcpp::Node::SharedPtr node, mpc::MPCConfiguration::Ptr mpc_configuration)
        : node_(node), mpc_configuration_(mpc_configuration)
    {
        walk_gait_ = std::make_shared<mpc::Gait>();
        walk_gait_->load("/home/nvidia/starq_ws/src/starq/gaits/walk.txt");
        walk_gait_->setVelocity(Vector3(0, 0, 0), Vector3(0, 0, 0));
        walk_gait_->setFrequency(3.0);

        const Float stand_height = mpc_configuration->getRobotParameters()->getStandingHeight();
        stand_gait_ = std::make_shared<mpc::Gait>();
        stand_gait_->load("/home/nvidia/starq_ws/src/starq/gaits/stand.txt");
        stand_gait_->setPose(Vector3(0, 0, stand_height), Vector3(0, 0, 0));
        stand_gait_->setFrequency(10.0);

        crawl_gait_ = std::make_shared<mpc::Gait>();
        crawl_gait_->load("/home/nvidia/starq_ws/src/starq/gaits/crawl.txt");
        crawl_gait_->setVelocity(Vector3(0, 0, 0), Vector3(0, 0, 0));
        crawl_gait_->setFrequency(3.0);

        current_gait_ = stand_gait_;
        mpc_configuration_->setNextGait(stand_gait_);

        base_position_ = Vector3::Zero();
        base_orientation_ = Vector3::Zero();

        joy_sub_ = node_->create_subscription<sensor_msgs::msg::Joy>(
            "joy", 10, std::bind(&ROS2MPCJoystick::joyCallback, this, std::placeholders::_1));
    }

    void ROS2MPCJoystick::joyCallback(const sensor_msgs::msg::Joy::SharedPtr msg)
    {
        if (msg->buttons[0] == 1)
        {
            mpc_configuration_->setNextGait(stand_gait_);
            current_gait_ = stand_gait_;
            base_position_ = mpc_configuration_->getLocalization()->getCurrentPosition();
            base_orientation_ = mpc_configuration_->getLocalization()->getCurrentOrientation();
            RCLCPP_INFO(node_->get_logger(), "Stand Gait selected");
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
        else if (msg->buttons[1] == 1)
        {
            mpc_configuration_->setNextGait(walk_gait_);
            current_gait_ = walk_gait_;
            RCLCPP_INFO(node_->get_logger(), "Walk Gait selected");
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
        else if (msg->buttons[2] == 1)
        {
            mpc_configuration_->setNextGait(crawl_gait_);
            current_gait_ = crawl_gait_;
            RCLCPP_INFO(node_->get_logger(), "Crawl Gait selected");
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }

        const int8_t left_axis_x = msg->axes[1] * 127;
        const int8_t left_axis_y = msg->axes[0] * 127;
        // const int8_t right_axis_x = msg->axes[2] * 127; // logitech
        const int8_t right_axis_x = msg->axes[3] * 127; // xbox

        if (current_gait_ == walk_gait_)
        {
            const Vector3 max_linear_velocity = walk_gait_->getMaxLinearVelocity();
            const Vector3 max_angular_velocity = walk_gait_->getMaxAngularVelocity();

            const Vector3 linear_velocity = (Vector3(left_axis_x, left_axis_y, 0) / 128.0).cwiseProduct(max_linear_velocity);
            const Vector3 angular_velocity = (Vector3(0, 0, right_axis_x) / 128.0).cwiseProduct(max_angular_velocity);

            // printf("Linear Velocity: %f %f %f\n", linear_velocity.x(), linear_velocity.y(), linear_velocity.z());
            // printf("Angular Velocity: %f %f %f\n", angular_velocity.x(), angular_velocity.y(), angular_velocity.z());

            walk_gait_->setVelocity(linear_velocity, angular_velocity);
        }
        else if (current_gait_ == stand_gait_)
        {
            const Vector3 max_position(0.05, 0.05, 0.05);
            const Vector3 max_orientation(0.1, 0.1, 0.1);

            const Float stand_height = mpc_configuration_->getRobotParameters()->getStandingHeight();

            Vector3 position = (Vector3(left_axis_x, left_axis_y, 0) / 128.0).cwiseProduct(max_position);
            Vector3 orientation = (Vector3(0, 0, right_axis_x) / 128.0).cwiseProduct(max_orientation);
            position += base_position_;
            orientation += base_orientation_;
            position.z() = stand_height;

            // printf("Position: %f %f %f\n", position.x(), position.y(), position.z());
            // printf("Orientation: %f %f %f\n", orientation.x(), orientation.y(), orientation.z());

            stand_gait_->setPose(position, orientation);
        }
        else if (current_gait_ == crawl_gait_)
        {
            const Vector3 max_linear_velocity = crawl_gait_->getMaxLinearVelocity();
            const Vector3 max_angular_velocity = crawl_gait_->getMaxAngularVelocity();

            const Vector3 linear_velocity = (Vector3(left_axis_x, left_axis_y, 0) / 128.0).cwiseProduct(max_linear_velocity);
            const Vector3 angular_velocity = (Vector3(0, 0, right_axis_x) / 128.0).cwiseProduct(max_angular_velocity);

            crawl_gait_->setVelocity(linear_velocity, angular_velocity);
        }
    }

}