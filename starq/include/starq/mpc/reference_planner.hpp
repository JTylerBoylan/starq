#ifndef STARQ_MPC__REFERENCE_PLANNER_HPP_
#define STARQ_MPC__REFERENCE_PLANNER_HPP_

#include <memory>
#include "starq/mpc/mpc_types.hpp"
#include "starq/mpc/gait.hpp"
#include "starq/slam/localization.hpp"
#include "starq/robot_parameters.hpp"

namespace starq::mpc
{
    /// @brief ReferencePlanner class
    class ReferencePlanner
    {
    public:
        using Ptr = std::shared_ptr<ReferencePlanner>;

        /// @brief Create a new ReferencePlanner object
        /// @param localization The localization object
        ReferencePlanner(starq::slam::Localization::Ptr localization,
                            starq::RobotParameters::Ptr robot_dynamics);

        /// @brief Destroy the ReferencePlanner object
        ~ReferencePlanner();

        /// @brief Configure the MPC
        /// @param N The mpc horizon
        /// @param dt The mpc time step
        /// @param gait_seq The gait sequence
        /// @param ref_traj The reference trajectory
        /// @return True if the MPC is configured successfully, false otherwise
        bool configure(const size_t N, const milliseconds dt,
                       const GaitSequence &gait_seq, ReferenceTrajectory &ref_traj) const;

    private:
        starq::slam::Localization::Ptr localization_;
        starq::RobotParameters::Ptr robot_parameters_;
    };
}

#endif