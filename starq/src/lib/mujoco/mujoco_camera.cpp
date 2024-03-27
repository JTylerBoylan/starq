#include "starq/mujoco/mujoco_camera.hpp"

#include <iostream>

namespace starq::mujoco
{

    MuJoCoCamera::MuJoCoCamera(MuJoCo::Ptr mujoco, std::string camera_name)
        : mujoco_(mujoco),
          camera_name_(camera_name)
    {
    }

    void MuJoCoCamera::open()
    {
        if (is_open_)
        {
            std::cerr << "Camera is already open" << std::endl;
            return;
        }

        if (mujoco_->getModel() == nullptr)
        {
            std::cerr << "Simulation is not opened" << std::endl;
            return;
        }

        camera_.type = mjCAMERA_FIXED;
        camera_.fixedcamid = mj_name2id(mujoco_->getModel(), mjOBJ_CAMERA, camera_name_.c_str());

        mjv_defaultOption(&option_);
        mjv_defaultPerturb(&perturb_);
        mjv_defaultScene(&scene_);
        mjr_defaultContext(&context_);

        window_ = glfwCreateWindow(1200, 800, (camera_name_ + " Video").c_str(), NULL, NULL);
        glfwMakeContextCurrent(window_);
        glfwSwapInterval(1);

        mjv_makeScene(mujoco_->getModel(), &scene_, 1000);
        mjr_makeContext(mujoco_->getModel(), &context_, mjFONTSCALE_150);

        is_open_ = true;
        while (!glfwWindowShouldClose(window_))
        {
            mjrRect viewport = {0, 0, 0, 0};
            glfwGetFramebufferSize(window_, &viewport.width, &viewport.height);

            mjv_updateScene(mujoco_->getModel(), mujoco_->getData(), &option_, NULL, &camera_, mjCAT_ALL, &scene_);
            mjr_render(viewport, &scene_, &context_);

            // Swap OpenGL buffers
            glfwSwapBuffers(window_);

            // process pending GUI events, call GLFW callbacks
            glfwPollEvents();
        }
        is_open_ = false;

        glfwDestroyWindow(window_);
        mjv_freeScene(&scene_);
        mjr_freeContext(&context_);
    }

    void MuJoCoCamera::close()
    {
        if (!is_open_)
        {
            std::cerr << "Camera is already closed" << std::endl;
            return;
        }

        is_open_ = false;
    }

    bool MuJoCoCamera::isOpen()
    {
        return is_open_;
    }

}