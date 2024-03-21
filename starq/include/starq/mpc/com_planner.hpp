#ifndef STARQ_MPC__COM_PLANNER_HPP_
#define STARQ_MPC__COM_PLANNER_HPP_

#include <memory>
#include "starq/mpc/mpc_types.hpp"
#include "starq/mpc/gait.hpp"
#include "starq/slam/localization.hpp"
#include "starq/robot_dynamics.hpp"

namespace starq::mpc
{
    /// @brief CenterOfMassPlanner class
    class CenterOfMassPlanner
    {
    public:
        using Ptr = std::shared_ptr<CenterOfMassPlanner>;

        /// @brief Create a new CenterOfMassPlanner object
        /// @param localization The localization object
        CenterOfMassPlanner(starq::slam::Localization::Ptr localization,
                            starq::RobotDynamics::Ptr robot_dynamics);

        /// @brief Destroy the CenterOfMassPlanner object
        ~CenterOfMassPlanner();

        /// @brief Configure the MPC
        /// @param N The mpc horizon
        /// @param dt The mpc time step
        /// @param gait_seq The gait sequence
        /// @param ref_traj The reference trajectory
        /// @return True if the MPC is configured successfully, false otherwise
        bool configure(const size_t N, const milliseconds dt,
                       const GaitSequence &gait_seq, ReferenceTrajectory &ref_traj) const;

    private:

        Vector3 getWorldAngularVelocity(const Vector3 &orientation, const Vector3 &angular_velocity) const;

        starq::slam::Localization::Ptr localization_;
        starq::RobotDynamics::Ptr robot_dynamics_;
    };
}

#endif