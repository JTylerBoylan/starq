#ifndef STARQ_MUJOCO__MUJOCO_CAMERA_HPP_
#define STARQ_MUJOCO__MUJOCO_CAMERA_HPP_

#include "starq/mujoco/mujoco.hpp"

namespace starq::mujoco
{

    /// @brief MuJoCo camera class
    class MuJoCoCamera
    {
    public:
        using Ptr = std::shared_ptr<MuJoCoCamera>;

        /// @brief Constructor
        /// @param mujoco MuJoCo instance
        /// @param camera_name Camera name
        MuJoCoCamera(MuJoCo::Ptr mujoco, std::string camera_name);

        /// @brief Initialize camera
        void initialize();

        /// @brief Open camera display
        void open();

        /// @brief Close camera display
        void close();

        /// @brief Cleanup camera display
        void cleanup();

        /// @brief Check if camera is initialized
        /// @return True if camera is initialized
        bool isInitialized() { return initialized_; }

        /// @brief Check if camera display is open
        /// @return True if camera display is open
        bool isOpen() { return is_open_; }

        /// @brief Get GLFW window
        /// @return GLFW window
        GLFWwindow *getWindow() { return window_; }

        /// @brief Get MuJoCo camera
        /// @return MuJoCo camera
        mjvCamera *getCamera() { return &camera_; }

        /// @brief Get MuJoCo option
        /// @return MuJoCo option
        mjvOption *getOption() { return &option_; }

        /// @brief Get MuJoCo perturb
        /// @return MuJoCo perturb
        mjvPerturb *getPerturb() { return &perturb_; }

        /// @brief Get MuJoCo scene
        /// @return MuJoCo scene
        mjvScene *getScene() { return &scene_; }

        /// @brief Get MuJoCo context
        /// @return MuJoCo context
        mjrContext *getContext() { return &context_; }

    private:
        MuJoCo::Ptr mujoco_;
        std::string camera_name_;

        bool initialized_ = false;

        bool is_open_ = false;
        GLFWwindow *window_ = nullptr;
        
        mjvCamera camera_;
        mjvOption option_;
        mjvPerturb perturb_;
        mjvScene scene_;
        mjrContext context_;
    };

}

#endif