#include "starq/ros2/ros2_mujoco_camera.hpp"

#include <iostream>

namespace starq::ros2
{

    ROS2MuJoCoCamera::ROS2MuJoCoCamera(rclcpp::Node::SharedPtr node,
                                       starq::mujoco::MuJoCoCamera::Ptr mujoco_camera,
                                       std::string image_topic)
        : node_(node), mujoco_camera_(mujoco_camera)
    {
        image_pub_ = node_->create_publisher<sensor_msgs::msg::Image>(image_topic, 10);

        timer_ = node_->create_wall_timer(std::chrono::milliseconds(1000 / 30),
                                          std::bind(&ROS2MuJoCoCamera::timerCallback, this));
    }

    void ROS2MuJoCoCamera::timerCallback()
    {

        if (!mujoco_camera_->isInitialized())
        {
            mujoco_camera_->initialize();
        }

        if (mujoco_camera_->isInitialized())
        {
            const auto mujoco = mujoco::MuJoCo::getInstance();

            mjrRect viewport = {0, 0, 0, 0};
            glfwGetFramebufferSize(mujoco_camera_->getWindow(), &viewport.width, &viewport.height);

            mjv_updateScene(mujoco->getModel(), mujoco->getData(),
                            mujoco_camera_->getOption(),
                            mujoco_camera_->getPerturb(),
                            mujoco_camera_->getCamera(),
                            mjCAT_ALL,
                            mujoco_camera_->getScene());

            mjr_render(viewport, mujoco_camera_->getScene(), mujoco_camera_->getContext());

            std::vector<uint8_t> image_data(viewport.width * viewport.height * 3);
            mjr_readPixels(image_data.data(), NULL, viewport, mujoco_camera_->getContext());

            // Flip the image vertically
            std::vector<uint8_t> flipped_image_data(viewport.width * viewport.height * 3);
            for (int y = 0; y < viewport.height; y++)
            {
                std::memcpy(&flipped_image_data[y * viewport.width * 3],
                            &image_data[(viewport.height - 1 - y) * viewport.width * 3],
                            viewport.width * 3);
            }

            auto image_msg = sensor_msgs::msg::Image();
            image_msg.header.stamp = node_->now();
            image_msg.header.frame_id = "camera";
            image_msg.width = viewport.width;
            image_msg.height = viewport.height;
            image_msg.encoding = "rgb8";
            image_msg.step = viewport.width * 3;
            image_msg.data = flipped_image_data;

            image_pub_->publish(image_msg);

            // Swap OpenGL buffers
            glfwSwapBuffers(mujoco_camera_->getWindow());

            // process pending GUI events, call GLFW callbacks
            glfwPollEvents();
        }
    }
}