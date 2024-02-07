#ifndef STARQ_MUJOCO__MIJOCO_SIM_HPP
#define STARQ_MUJOCO__MIJOCO_SIM_HPP

#include <memory>
#include <thread>
#include <mutex>
#include <string>

#include "GLFW/glfw3.h"
#include "mujoco/mujoco.h"

namespace starq::mujoco
{

    /// @brief MuJoCo simulation class
    class MuJoCoSim
    {
    public:
        using Ptr = std::shared_ptr<MuJoCoSim>;

        /// @brief Create a new MuJoCo simulation
        /// @param model_path Path to the MuJoCo model file
        MuJoCoSim(const std::string &model_path);

        /// @brief Destructor
        ~MuJoCoSim();

        /// @brief Open the simulation window
        /// @return If the simulation was successfully opened
        bool open();

        /// @brief Close the simulation window
        /// @return If the simulation was successfully closed
        bool close();

        /// @brief Check if the simulation is initialized
        /// @return If the simulation is initialized
        bool isInitialized() const { return initialized_; }

        /// @brief Check if the simulation is running
        /// @return If the simulation is running
        bool isRunning() const { return running_; }

    private:
        void run();

        void key_press(GLFWwindow *window, int key, int scancode, int act, int mods);
        void mouse_click(GLFWwindow *window, int button, int act, int mods);
        void mouse_move(GLFWwindow *window, double xpos, double ypos);
        void mouse_scroll(GLFWwindow *window, double xoffset, double yoffset);

        static void key_press_callback(GLFWwindow *window, int key, int scancode, int act, int mods);
        static void mouse_click_callback(GLFWwindow *window, int button, int act, int mods);
        static void mouse_move_callback(GLFWwindow *window, double xpos, double ypos);
        static void mouse_scroll_callback(GLFWwindow *window, double xoffset, double yoffset);

        bool initialized_;
        bool running_;

        mjModel *model_;
        mjData *data_;
        mjvCamera camera_;
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

        std::mutex mutex_;
    };

}

#endif