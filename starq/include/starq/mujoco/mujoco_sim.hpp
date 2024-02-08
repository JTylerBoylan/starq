#ifndef STARQ_MUJOCO__MUJOCO_SIM_HPP
#define STARQ_MUJOCO__MUJOCO_SIM_HPP

#include <memory>
#include <string>

#include "GLFW/glfw3.h"
#include "mujoco/mujoco.h"

namespace starq::mujoco
{

    /// @brief MuJoCo simulation class
    class MuJoCo
    {
    public:
        using Ptr = std::shared_ptr<MuJoCo>;

        static MuJoCo::Ptr getInstance();

        /// @brief Default constructor
        MuJoCo() = default;

        /// @brief Open the simulation window
        /// @return If the simulation was successfully opened
        void open(const std::string &model_path);

        /// @brief Close the simulation window
        /// @return If the simulation was successfully closed
        void close();

        /// @brief Check if the simulation is open
        /// @return If the simulation is open
        bool isOpen() const;

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

        static void keyPressCallback(GLFWwindow *window, int key, int scancode, int act, int mods);
        static void mouseClickCallback(GLFWwindow *window, int button, int act, int mods);
        static void mouseMoveCallback(GLFWwindow *window, double xpos, double ypos);
        static void mouseScrollCallback(GLFWwindow *window, double xoffset, double yoffset);
        static void controlCallback(const mjModel *model, mjData *data);

        bool is_open_ = false;
        GLFWwindow *window_ = nullptr;

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

        static MuJoCo::Ptr instance_;
    };

    MuJoCo::Ptr MuJoCo::instance_ = nullptr;
}

#endif