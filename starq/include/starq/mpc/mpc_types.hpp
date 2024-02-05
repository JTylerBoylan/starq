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

    inline const milliseconds getCurrentTime()
    {
        return duration_cast<milliseconds>(system_clock::now().time_since_epoch());
    }

    struct CenterOfMassState
    {
        Vector3f position;
        Vector3f orientation;
        Vector3f velocity;
        Vector3f angular_velocity;
    };
    using CenterOfMassTrajectory = std::vector<CenterOfMassState>;

    using StanceState = std::vector<bool>;
    using StanceTrajectory = std::vector<StanceState>;

    using FootholdState = std::vector<Vector3f>;
    using FootholdTrajectory = std::vector<FootholdState>;

    struct MPCConfiguration
    {
        float mass;
        Matrix3f inertia;
        Vector3f gravity;

        float time_step;
        size_t window_size;
        uint8_t number_of_legs;
        CenterOfMassTrajectory com_trajectory;
        StanceTrajectory stance_trajectory;
        FootholdTrajectory foothold_trajectory;
    };

}

#endif