#ifndef STARQ_MPC__MPC_TYPES_HPP_
#define STARQ_MPC__MPC_TYPES_HPP_

#include <vector>
#include <memory>

#include "starq/types.hpp"

namespace starq::mpc
{
    struct ReferenceState
    {
        Vector3 position;
        Vector3 orientation;
        Vector3 linear_velocity;
        Vector3 angular_velocity;
    };
    using ReferenceTrajectory = std::vector<ReferenceState>;

    struct ReferenceWeights
    {
        Vector3 position;
        Vector3 orientation;
        Vector3 linear_velocity;
        Vector3 angular_velocity;
    };

    using ForceWeights = Vector3;

    using StanceState = std::vector<bool>;
    using StanceTrajectory = std::vector<StanceState>;

    using FootholdState = std::vector<Vector3>;
    using FootholdTrajectory = std::vector<FootholdState>;

    using FootForceState = std::vector<std::pair<bool, Vector3>>;
    using FootForceTrajectory = std::vector<FootForceState>;

    struct MPCSolution
    {
        using Ptr = std::shared_ptr<MPCSolution>;
        
        int exit_flag;
        std::chrono::microseconds run_time;
        std::chrono::microseconds setup_time;
        std::chrono::microseconds solve_time;

        ReferenceTrajectory x_star;
        FootForceTrajectory u_star;
    };

}

#endif