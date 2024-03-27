#ifndef STARQ_ROS2__ROS2_MUJOCO_CAMERA_HPP_
#define STARQ_ROS2__ROS2_MUJOCO_CAMERA_HPP_

#include "starq/ros2/ros2_util.hpp"
#include "starq/mujoco/mujoco_camera.hpp"
#include "sensor_msgs/msg/image.hpp"

namespace starq::ros2
{

    /// @brief ROS2 MuJoCo camera class
    class ROS2MuJoCoCamera
    {
    public:
        using Ptr = std::shared_ptr<ROS2MuJoCoCamera>;

        /// @brief Constructor
        /// @param node ROS2 node
        /// @param mujoco_camera MuJoCo camera
        /// @param image_topic Image topic
        ROS2MuJoCoCamera(rclcpp::Node::SharedPtr node,
                         starq::mujoco::MuJoCoCamera::Ptr mujoco_camera,
                         std::string image_topic);

    private:
        /// @brief Timer callback
        void timerCallback();

        rclcpp::Node::SharedPtr node_;
        starq::mujoco::MuJoCoCamera::Ptr mujoco_camera_;

        rclcpp::TimerBase::SharedPtr timer_;
        rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
    };

}

#endif