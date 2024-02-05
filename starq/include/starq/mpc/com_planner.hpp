#ifndef STARQ_MPC__COM_PLANNER_HPP_
#define STARQ_MPC__COM_PLANNER_HPP_

#include <memory>
#include "starq/mpc/mpc_types.hpp"

namespace starq::mpc
{
    class CenterOfMassPlanner
    {
    public:
        using Ptr = std::shared_ptr<CenterOfMassPlanner>;

        CenterOfMassPlanner();

        ~CenterOfMassPlanner();

        void setInitialState(const CenterOfMassState &initial_state)
        {
            initial_state_ = initial_state;
        }

        void setVelocity(const Vector3f &linear_velocity, const Vector3f &angular_velocity)
        {
            linear_velocity_ = linear_velocity;
            angular_velocity_ = angular_velocity;
        }

        CenterOfMassTrajectory getCenterOfMassTrajectory(const float time_step, const size_t window_size) const
        {
            CenterOfMassTrajectory com_trajectory;
            com_trajectory.reserve(window_size);
            com_trajectory.push_back(initial_state_);

            CenterOfMassState com_state = initial_state_;
            com_state.velocity = linear_velocity_;
            com_state.angular_velocity = angular_velocity_;
            for (int i = 1; i < window_size; i++)
            {
                com_state.orientation.z() += angular_velocity_.z() * time_step;
                com_state.position.x() += linear_velocity_.x() * std::cos(com_state.orientation.z()) * time_step;
                com_state.position.y() += linear_velocity_.y() * std::sin(com_state.orientation.z()) * time_step;
                com_trajectory.push_back(com_state);
            }
        }

    private:
        CenterOfMassState initial_state_;
        Vector3f linear_velocity_;
        Vector3f angular_velocity_;
    };
}

#endif