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
        bool configure(const size_t N, const milliseconds dt,
                       const GaitSequence &gait_seq, ReferenceTrajectory &ref_traj) const;

    private:
        starq::slam::Localization::Ptr localization_;
        starq::RobotDynamics::Ptr robot_dynamics_;
    };
}

#endif