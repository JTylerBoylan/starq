#include "starq/ros2/ros2_localization.hpp"

namespace starq::ros2
{
    ROS2Localization::ROS2Localization(rclcpp::Node::SharedPtr node, std::string odometry_topic)
        : node_(node),
          position_(Vector3::Zero()),
          orientation_(Vector3::Zero()),
          linear_velocity_(Vector3::Zero()),
          angular_velocity_(Vector3::Zero())
    {
        odometry_sub_ = node_->create_subscription<nav_msgs::msg::Odometry>(
            odometry_topic, 10,
            std::bind(&ROS2Localization::odometryCallback, this, std::placeholders::_1));
    }

    milliseconds ROS2Localization::getCurrentTime()
    {
        rclcpp::Time time = node_->now();
        return milliseconds(static_cast<time_t>(time.seconds() * 1E3 + time.nanoseconds() * 1E-6));
    }

    Vector3 ROS2Localization::getCurrentPosition()
    {
        return position_;
    }

    Vector3 ROS2Localization::getCurrentOrientation()
    {
        return orientation_;
    }

    Vector3 ROS2Localization::getCurrentLinearVelocity()
    {
        return linear_velocity_;
    }

    Vector3 ROS2Localization::getCurrentAngularVelocity()
    {
        return angular_velocity_;
    }

    void ROS2Localization::odometryCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        position_.x() = msg->pose.pose.position.x;
        position_.y() = msg->pose.pose.position.y;
        position_.z() = msg->pose.pose.position.z;

        Eigen::Quaternion<Float> q;
        q.x() = msg->pose.pose.orientation.x;
        q.y() = msg->pose.pose.orientation.y;
        q.z() = msg->pose.pose.orientation.z;
        q.w() = msg->pose.pose.orientation.w;
        quat2eul(q, orientation_);

        linear_velocity_.x() = msg->twist.twist.linear.x;
        linear_velocity_.y() = msg->twist.twist.linear.y;
        linear_velocity_.z() = msg->twist.twist.linear.z;

        angular_velocity_.x() = msg->twist.twist.angular.x;
        angular_velocity_.y() = msg->twist.twist.angular.y;
        angular_velocity_.z() = msg->twist.twist.angular.z;
    }

    void ROS2Localization::quat2eul(const Eigen::Quaternion<Float> &q, Vector3 &eul)
    {
        eul.x() = atan2(2 * (q.w() * q.x() + q.y() * q.z()), 1 - 2 * (q.x() * q.x() + q.y() * q.y()));
        eul.y() = asin(2 * (q.w() * q.y() - q.z() * q.x()));
        eul.z() = atan2(2 * (q.w() * q.z() + q.x() * q.y()), 1 - 2 * (q.y() * q.y() + q.z() * q.z()));
    }
}