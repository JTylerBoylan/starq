#ifndef STARQ_MPC__MPC_PLANNER_HPP_
#define STARQ_MPC__MPC_PLANNER_HPP_

#include <memory>

#include "starq/mpc/mpc_types.hpp"
#include "starq/mpc/gait_sequencer.hpp"

namespace starq::mpc
{

    class MPCPlanner
    {
    public:
        using Ptr = std::shared_ptr<MPCPlanner>;

        MPCPlanner();

        ~MPCPlanner();

        void setDesiredVelocity(const Vector3f &linear_velocity,
                                const Vector3f &angular_velocity);

        MPCConfiguration getPlan();

    private:
        Vector3f desired_linear_velocity_;
        Vector3f desired_angular_velocity_;
        GaitSequencer::Ptr gait_sequencer_;
        MPCConfiguration mpc_configuration_;
    };

}

#endif