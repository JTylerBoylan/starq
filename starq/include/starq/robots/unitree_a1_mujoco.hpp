#ifndef STARQ_ROBOTS__UNITREE_A1_MUJOCO_HPP_
#define STARQ_ROBOTS__UNITREE_A1_MUJOCO_HPP_

#include "starq/mujoco/mujoco_controller.hpp"
#include "starq/mujoco/mujoco_localization.hpp"
#include "starq/dynamics/unitree_rrr.hpp"

#include "starq/robot.hpp"

namespace starq::robots
{
    using namespace starq::mujoco;
    using namespace starq::dynamics;

    class UnitreeA1MuJoCoRobot : public Robot
    {
    public:
        using Ptr = std::shared_ptr<UnitreeA1MuJoCoRobot>;

        UnitreeA1MuJoCoRobot();

    private:
        MuJoCo::Ptr mujoco_;
    };

}

#endif