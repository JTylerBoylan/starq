#ifndef STARQ_MUJOCO__MUJOCO_CAMERA_HPP_
#define STARQ_MUJOCO__MUJOCO_CAMERA_HPP_

#include "starq/mujoco/mujoco.hpp"

namespace starq::mujoco
{

    class MuJoCoCamera
    {
    public:
        using Ptr = std::shared_ptr<MuJoCoCamera>;

        MuJoCoCamera(MuJoCo::Ptr mujoco, std::string camera_name);

        void open();

        void close();

        bool isOpen();

    private:
        MuJoCo::Ptr mujoco_;
        std::string camera_name_;

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