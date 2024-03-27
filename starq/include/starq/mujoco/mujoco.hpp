#ifndef STARQ_MUJOCO__MUJOCO_HPP
#define STARQ_MUJOCO__MUJOCO_HPP

#include <memory>
#include <string>
#include <vector>
#include <functional>

#include "GLFW/glfw3.h"
#include "mujoco/mujoco.h"

namespace starq::mujoco
{

    using MuJoCoControlFunction = std::function<void(const mjModel *model, mjData *data)>;

    /// @brief MuJoCo simulation class
    class MuJoCo
    {
    public:
        using Ptr = std::shared_ptr<MuJoCo>;

        /// @brief Get the static MuJoCo instance
        /// @return The MuJoCo instance
        static MuJoCo::Ptr getInstance();

        /// @brief Constructor
        MuJoCo();

        /// @brief Destructor
        ~MuJoCo();

        /// @brief Open the simulation window
        /// @return If the simulation was successfully opened
        void open(const std::string &model_path);

        /// @brief Close the simulation window
        /// @return If the simulation was successfully closed
        void close();

        /// @brief Check if the simulation is open
        /// @return If the simulation is open
        bool isOpen();

        /// @brief Add a motor control function
        /// @param control_function Motor control function
        void addMotorControlFunction(const MuJoCoControlFunction &control_function);

        /// @brief Set the simulation frame rate
        /// @param frame_rate Frame rate
        void setFrameRate(double frame_rate) { frame_rate_ = frame_rate; }

        /// @brief Get the MuJoCo model
        /// @return The MuJoCo model
        mjModel *getModel() const { return model_; }

        /// @brief Get the MuJoCo data
        /// @return The MuJoCo data
        mjData *getData() const { return data_; }

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

        /// @brief Get MuJoCo context
        /// @return MuJoCo context
        mjrContext *getContext() { return &context_; }

        /// @brief [Deleted]
        MuJoCo(const MuJoCo &) = delete;

        /// @brief [Deleted]
        MuJoCo &operator=(const MuJoCo &) = delete;

    private:
        /// @brief Load the MuJoCo model
        void loadModel(const std::string &model_path);

        /// @brief Initialize GLFW
        void initGLFW();

        /// @brief Setup GLFW and MuJoCo callbacks
        void setupCallbacks();

        /// @brief Cleanup GLFW and MuJoCo
        void cleanup();

        bool is_open_ = false;
        GLFWwindow *window_ = nullptr;
        double frame_rate_ = 60.0;

        mjModel *model_ = nullptr;
        mjData *data_ = nullptr;
        mjvCamera camera_;
        mjvPerturb perturb_;
        mjvOption option_;
        mjvScene scene_;
        mjrContext context_;

        struct
        {
            bool button_left = false;
            bool button_middle = false;
            bool button_right = false;
            double lastx = 0;
            double lasty = 0;
        } mouse_;

        std::vector<MuJoCoControlFunction> control_functions_;

        /// @brief GLFW key press callback
        static void keyPressCallback(GLFWwindow *window, int key, int scancode, int act, int mods);

        /// @brief GLFW mouse click callback
        static void mouseClickCallback(GLFWwindow *window, int button, int act, int mods);

        /// @brief GLFW mouse move callback
        static void mouseMoveCallback(GLFWwindow *window, double xpos, double ypos);

        /// @brief GLFW mouse scroll callback
        static void mouseScrollCallback(GLFWwindow *window, double xoffset, double yoffset);

        /// @brief MuJoCo control callback
        static void controlCallback(const mjModel *model, mjData *data);

        static MuJoCo::Ptr instance_;
    };

}

#endif