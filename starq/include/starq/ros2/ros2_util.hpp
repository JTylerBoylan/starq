#ifndef STARQ_ROS2__ROS2_UTIL_HPP_
#define STARQ_ROS2__ROS2_UTIL_HPP_

#include <rclcpp/rclcpp.hpp>

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

}

#endif