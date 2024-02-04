#ifndef STARQ_ROS2__ROS2_UTIL_HPP_
#define STARQ_ROS2__ROS2_UTIL_HPP_

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/vector3.hpp"

#include "eigen3/Eigen/Dense"

namespace starq::ros2
{

    inline rclcpp::QoS getFastQoS()
    {
        auto qos_fast = rclcpp::QoS(rclcpp::KeepLast(10));
        qos_fast.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);
        qos_fast.durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);
        qos_fast.history(RMW_QOS_POLICY_HISTORY_KEEP_LAST);
        qos_fast.deadline(std::chrono::milliseconds(100));
        return qos_fast;
    }

    inline rclcpp::QoS getReliableQoS()
    {
        auto qos_reliable = rclcpp::QoS(rclcpp::KeepLast(50));
        qos_reliable.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);
        qos_reliable.durability(RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL);
        qos_reliable.deadline(std::chrono::seconds(1));
        return qos_reliable;
    }

    inline void ros2eigen(const geometry_msgs::msg::Vector3 &ros, Eigen::Vector3f &eigen)
    {
        eigen.x() = ros.x;
        eigen.y() = ros.y;
        eigen.z() = ros.z;
    }

    inline void eigen2ros(const Eigen::VectorXf &eigen, geometry_msgs::msg::Vector3 &ros)
    {
        ros.x = eigen.x();
        ros.y = eigen.y();
        ros.z = eigen.z();
    }

}

#endif