#include "starq/mujoco/mujoco_sim.hpp"

#include <iostream>
#include <functional>

namespace starq::mujoco
{
    MuJoCoSim::MuJoCoSim(const std::string &model_path)
        : initialized_(false), running_(false),
          model_(nullptr), data_(nullptr)
    {
        char error[1000];
        model_ = mj_loadXML(model_path.c_str(), nullptr, error, 1000);

        if (!model_)
        {
            std::cerr << "Error loading model: " << error << std::endl;
            return;
        }

        data_ = mj_makeData(model_);

        if (!data_)
        {
            std::cerr << "Error making data" << std::endl;
            return;
        }

        mjv_makeScene(model_, &scene_, 1000);

        mjv_defaultCamera(&camera_);
        mjv_defaultOption(&option_);
        mjr_defaultContext(&context_);

        if (!glfwInit())
        {
            std::cerr << "Error initializing GLFW" << std::endl;
            return;
        }

        initialized_ = true;
    }

    MuJoCoSim::~MuJoCoSim()
    {
        mj_deleteData(data_);
        mj_deleteModel(model_);
    }

    bool MuJoCoSim::open()
    {
        if (!initialized_)
        {
            std::cerr << "MuJoCoSim not initialized" << std::endl;
            return false;
        }

        if (running_)
        {
            std::cerr << "MuJoCoSim already running" << std::endl;
            return false;
        }

        running_ = true;
        std::thread(&MuJoCoSim::run, this).detach();
        return true;
    }

    bool MuJoCoSim::close()
    {
        running_ = false;

        mjv_freeScene(&scene_);
        mjr_freeContext(&context_);

        return true;
    }

    void MuJoCoSim::run()
    {
        GLFWwindow *window = glfwCreateWindow(1200, 900, "MuJoCo", nullptr, nullptr);
        glfwMakeContextCurrent(window);
        glfwSwapInterval(1);

        mjv_makeScene(model_, &scene_, 1000);
        mjr_makeContext(model_, &context_, mjFONTSCALE_150);

        glfwSetWindowUserPointer(window, this);
        glfwSetKeyCallback(window, key_press_callback);
        glfwSetCursorPosCallback(window, mouse_move_callback);
        glfwSetMouseButtonCallback(window, mouse_click_callback);
        glfwSetScrollCallback(window, mouse_scroll_callback);

        while (!glfwWindowShouldClose(window) && running_)
        {
            mjtNum simstart = data_->time;
            while (data_->time - simstart < 1.0 / 60.0)
            {
                mj_step(model_, data_);
            }

            mjrRect viewport = {0, 0, 0, 0};
            glfwGetFramebufferSize(window, &viewport.width, &viewport.height);

            mjv_updateScene(model_, data_, &option_, NULL, &camera_, mjCAT_ALL, &scene_);
            mjr_render(viewport, &scene_, &context_);

            glfwSwapBuffers(window);

            glfwPollEvents();
        }

        running_ = false;

        glfwDestroyWindow(window);
    }

    void MuJoCoSim::key_press(GLFWwindow *window, int key, int scancode, int act, int mods)
    {
        (void)window;
        (void)scancode;
        (void)mods;
        if (act == GLFW_PRESS && key == GLFW_KEY_BACKSPACE)
        {
            mj_resetData(model_, data_);
            mj_forward(model_, data_);
        }
    }

    void MuJoCoSim::mouse_click(GLFWwindow *window, int button, int act, int mods)
    {
        (void)mods;
        (void)button;
        (void)act;
        mouse_.button_left = glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS;
        mouse_.button_middle = glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_MIDDLE) == GLFW_PRESS;
        mouse_.button_right = glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_RIGHT) == GLFW_PRESS;

        glfwGetCursorPos(window, &mouse_.lastx, &mouse_.lasty);
    }

    void MuJoCoSim::mouse_move(GLFWwindow *window, double xpos, double ypos)
    {
        if (!mouse_.button_left && !mouse_.button_middle && !mouse_.button_right)
            return;

        const double dx = xpos - mouse_.lastx;
        const double dy = ypos - mouse_.lasty;
        mouse_.lastx = xpos;
        mouse_.lasty = ypos;

        int width, height;
        glfwGetWindowSize(window, &width, &height);

        bool mod_shift = (glfwGetKey(window, GLFW_KEY_LEFT_SHIFT) == GLFW_PRESS) ||
                         (glfwGetKey(window, GLFW_KEY_RIGHT_SHIFT) == GLFW_PRESS);

        mjtMouse action;
        if (mouse_.button_right)
        {
            action = mod_shift ? mjMOUSE_MOVE_H : mjMOUSE_MOVE_V;
        }
        else if (mouse_.button_left)
        {
            action = mod_shift ? mjMOUSE_ROTATE_H : mjMOUSE_ROTATE_V;
        }
        else
        {
            action = mjMOUSE_ZOOM;
        }

        mjv_moveCamera(model_, action, dx / height, dy / height, &scene_, &camera_);
    }

    void MuJoCoSim::mouse_scroll(GLFWwindow *window, double xoffset, double yoffset)
    {
        (void)window;
        (void)xoffset;
        mjv_moveCamera(model_, mjMOUSE_ZOOM, 0, 0.05 * yoffset, &scene_, &camera_);
    }

    void MuJoCoSim::key_press_callback(GLFWwindow *window, int key, int scancode, int act, int mods)
    {
        MuJoCoSim *sim = static_cast<MuJoCoSim *>(glfwGetWindowUserPointer(window));
        if (sim)
            sim->key_press(window, key, scancode, act, mods);
    }

    void MuJoCoSim::mouse_click_callback(GLFWwindow *window, int button, int act, int mods)
    {
        MuJoCoSim *sim = static_cast<MuJoCoSim *>(glfwGetWindowUserPointer(window));
        if (sim)
            sim->mouse_click(window, button, act, mods);
    }

    void MuJoCoSim::mouse_move_callback(GLFWwindow *window, double xpos, double ypos)
    {
        MuJoCoSim *sim = static_cast<MuJoCoSim *>(glfwGetWindowUserPointer(window));
        if (sim)
            sim->mouse_move(window, xpos, ypos);
    }

    void MuJoCoSim::mouse_scroll_callback(GLFWwindow *window, double xoffset, double yoffset)
    {
        MuJoCoSim *sim = static_cast<MuJoCoSim *>(glfwGetWindowUserPointer(window));
        if (sim)
            sim->mouse_scroll(window, xoffset, yoffset);
    }

}
