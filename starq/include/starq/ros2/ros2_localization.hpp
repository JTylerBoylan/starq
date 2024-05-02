#ifndef STARQ_ROS2__ROS2_LOCALIZATION_HPP_
#define STARQ_ROS2__ROS2_LOCALIZATION_HPP_

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include "starq/slam/localization.hpp"

namespace starq::ros2
{
    class ROS2Localization : public slam::Localization
    {
    public:
        using Ptr = std::shared_ptr<ROS2Localization>;

        /// @brief Create a new ROS2Localization object.
        /// @param node The ROS2 node.
        /// @param odometry_topic The odometry topic.
        ROS2Localization(rclcpp::Node::SharedPtr node, std::string odometry_topic);

        /// @brief Get the current time.
        /// @return The current time in milliseconds.
        milliseconds getCurrentTime() override;

        /// @brief Get the current position of the robot.
        /// @return Position vector [m] (x, y, z) in the world frame.
        Vector3 getCurrentPosition() override;

        /// @brief Get the current orientation of the robot.
        /// @return Orientation vector [rad] (roll, pitch, yaw) in the world frame.
        Vector3 getCurrentOrientation() override;

        /// @brief Get the current linear velocity of the robot.
        /// @return Velocity vector [m/s] (x, y, z) in the world frame.
        Vector3 getCurrentLinearVelocity() override;

        /// @brief Get the current angular velocity of the robot.
        /// @return Angular velocity vector [rad/s] (roll, pitch, yaw) in the world frame.
        Vector3 getCurrentAngularVelocity() override;

    private:
        void odometryCallback(const nav_msgs::msg::Odometry::SharedPtr msg);

        void quat2eul(const Eigen::Quaternion<Float> &q, Vector3 &eul);

        rclcpp::Node::SharedPtr node_;
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_sub_;

        Vector3 position_;
        Vector3 orientation_;
        Vector3 linear_velocity_;
        Vector3 angular_velocity_;
    };
}

#endif