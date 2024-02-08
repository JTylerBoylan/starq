#ifndef STARQ_MUJOCO__MUJOCO_CONTROL_FUNCTION_HPP
#define STARQ_MUJOCO__MUJOCO_CONTROL_FUNCTION_HPP

#include <memory>

namespace starq::mujoco
{

    class MuJoCoControlFunction 
    {
    public:
        using Ptr = std::shared_ptr<MuJoCoControlFunction>;

        virtual void control() = 0;
    };
}


#endif