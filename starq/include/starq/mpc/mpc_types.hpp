#ifndef STARQ_MPC__MPC_TYPES_HPP_
#define STARQ_MPC__MPC_TYPES_HPP_

#include <vector>
#include <memory>
#include <chrono>

#include "eigen3/Eigen/Dense"

namespace starq::mpc
{
    using namespace std::chrono;
    using namespace Eigen;

    struct ReferenceState
    {
        Vector3f position;
        Vector3f orientation;
        Vector3f linear_velocity;
        Vector3f angular_velocity;
    };
    using ReferenceTrajectory = std::vector<ReferenceState>;

    struct ReferenceWeights
    {
        Vector3f position;
        Vector3f orientation;
        Vector3f linear_velocity;
        Vector3f angular_velocity;
    };

    using ForceWeights = Vector3f;

    using StanceState = std::vector<bool>;
    using StanceTrajectory = std::vector<StanceState>;

    using FootholdState = std::vector<Vector3f>;
    using FootholdTrajectory = std::vector<FootholdState>;

    using FootForceState = std::vector<std::pair<bool, Vector3f>>;
    using FootForceTrajectory = std::vector<FootForceState>;

    struct MPCSolution
    {
        int exit_flag;
        microseconds run_time;
        microseconds setup_time;
        microseconds solve_time;

        ReferenceTrajectory x_star;
        FootForceTrajectory u_star;
    };

}

#endif