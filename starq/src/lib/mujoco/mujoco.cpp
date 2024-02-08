#include "starq/mujoco/mujoco.hpp"

#include <iostream>
#include <functional>

namespace starq::mujoco
{

    MuJoCo::Ptr MuJoCo::getInstance()
    {
        if (!instance_)
        {
            instance_ = std::make_shared<MuJoCo>();
        }
        return instance_;
    }

    MuJoCo::MuJoCo()
    {
    }

    MuJoCo::~MuJoCo()
    {
    }

    void MuJoCo::open(const std::string &model_path)
    {

        if (is_open_)
        {
            std::cerr << "MuJoCo is already open" << std::endl;
            return;
        }

        loadModel(model_path);

        initGLFW();

        setupCallbacks();

        is_open_ = true;
        while (!glfwWindowShouldClose(window_))
        {
            mjtNum simstart = data_->time;
            while (data_->time - simstart < 1.0 / 60.0)
            {
                mj_step(model_, data_);
            }

            mjrRect viewport = {0, 0, 0, 0};
            glfwGetFramebufferSize(window_, &viewport.width, &viewport.height);

            mjv_updateScene(model_, data_, &option_, NULL, &camera_, mjCAT_ALL, &scene_);
            mjr_render(viewport, &scene_, &context_);

            glfwSwapBuffers(window_);

            glfwPollEvents();
        }

        cleanup();
    }

    void MuJoCo::close()
    {
        glfwSetWindowShouldClose(window_, GLFW_TRUE);
    }

    bool MuJoCo::isOpen()
    {
        return is_open_;
    }

    void MuJoCo::loadModel(const std::string &model_path)
    {
        char error[1000];
        model_ = mj_loadXML(model_path.c_str(), nullptr, error, 1000);

        if (!model_)
        {
            std::cerr << "Error loading MuJoCo model: " << error << std::endl;
            return;
        }

        data_ = mj_makeData(model_);

        if (!data_)
        {
            mj_deleteModel(model_);
            model_ = nullptr;
            std::cerr << "Error making MuJoCo model data" << std::endl;
            return;
        }
    }

    void MuJoCo::initGLFW()
    {

        if (!glfwInit())
        {
            std::cerr << "Error initializing GLFW" << std::endl;
            return;
        }

        window_ = glfwCreateWindow(1200, 900, "MuJoCo", nullptr, nullptr);

        if (!window_)
        {
            std::cerr << "Error creating GLFW window" << std::endl;
            return;
        }

        glfwMakeContextCurrent(window_);
        glfwSwapInterval(1);

        mjv_defaultCamera(&camera_);
        mjv_defaultPerturb(&perturb_);
        mjv_defaultOption(&option_);
        mjr_defaultContext(&context_);

        mjv_makeScene(model_, &scene_, 1000);
        mjr_makeContext(model_, &context_, mjFONTSCALE_150);
    }

    void MuJoCo::setupCallbacks()
    {
        glfwSetKeyCallback(window_, keyPressCallback);
        glfwSetCursorPosCallback(window_, mouseMoveCallback);
        glfwSetMouseButtonCallback(window_, mouseClickCallback);
        glfwSetScrollCallback(window_, mouseScrollCallback);

        mjcb_control = controlCallback;
    }

    void MuJoCo::cleanup()
    {
        if (!is_open_) return;

        if (window_)
        {
            glfwDestroyWindow(window_);
            window_ = nullptr;
        }

        mjv_freeScene(&scene_);
        mjr_freeContext(&context_);

        if (data_)
        {
            mj_deleteData(data_);
            data_ = nullptr;
        }

        if (model_)
        {
            mj_deleteModel(model_);
            model_ = nullptr;
        }

        glfwTerminate();
        is_open_ = false;
    }

    void MuJoCo::keyPressCallback(GLFWwindow *window, int key, int scancode, int act, int mods)
    {
        (void)window;
        (void)scancode;
        (void)mods;
        if (act == GLFW_PRESS && key == GLFW_KEY_BACKSPACE)
        {
            mj_resetData(instance_->model_, instance_->data_);
            mj_forward(instance_->model_, instance_->data_);
        }
    }

    void MuJoCo::mouseClickCallback(GLFWwindow *window, int button, int act, int mods)
    {
        (void)mods;
        (void)button;
        (void)act;
        instance_->mouse_.button_left = glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS;
        instance_->mouse_.button_middle = glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_MIDDLE) == GLFW_PRESS;
        instance_->mouse_.button_right = glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_RIGHT) == GLFW_PRESS;

        glfwGetCursorPos(window, &instance_->mouse_.lastx, &instance_->mouse_.lasty);
    }

    void MuJoCo::mouseMoveCallback(GLFWwindow *window, double xpos, double ypos)
    {
        if (!instance_->mouse_.button_left && !instance_->mouse_.button_middle && !instance_->mouse_.button_right)
            return;

        const double dx = xpos - instance_->mouse_.lastx;
        const double dy = ypos - instance_->mouse_.lasty;
        instance_->mouse_.lastx = xpos;
        instance_->mouse_.lasty = ypos;

        int width, height;
        glfwGetWindowSize(window, &width, &height);

        bool mod_shift = (glfwGetKey(window, GLFW_KEY_LEFT_SHIFT) == GLFW_PRESS) ||
                         (glfwGetKey(window, GLFW_KEY_RIGHT_SHIFT) == GLFW_PRESS);

        mjtMouse action;
        if (instance_->mouse_.button_right)
        {
            action = mod_shift ? mjMOUSE_MOVE_H : mjMOUSE_MOVE_V;
        }
        else if (instance_->mouse_.button_left)
        {
            action = mod_shift ? mjMOUSE_ROTATE_H : mjMOUSE_ROTATE_V;
        }
        else
        {
            action = mjMOUSE_ZOOM;
        }

        mjv_moveCamera(instance_->model_, action, dx / height, dy / height, &instance_->scene_, &instance_->camera_);
    }

    void MuJoCo::mouseScrollCallback(GLFWwindow *window, double xoffset, double yoffset)
    {
        (void)window;
        (void)xoffset;
        mjv_moveCamera(instance_->model_, mjMOUSE_ZOOM, 0, 0.05 * yoffset, &instance_->scene_, &instance_->camera_);
    }

    void MuJoCo::addMotorControlFunction(const MuJoCoControlFunction &control_function)
    {
        control_functions_.push_back(control_function);
    }

    void MuJoCo::controlCallback(const mjModel *model, mjData *data)
    {
        for (const auto &control_function : instance_->control_functions_)
        {
            control_function(model, data);
        }
    }

}
