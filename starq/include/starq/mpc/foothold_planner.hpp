#ifndef STARQ_MPC__FOOTHOLD_PLANNER_HPP_
#define STARQ_MPC__FOOTHOLD_PLANNER_HPP_

#include <memory>
#include "starq/mpc/mpc_types.hpp"
#include "starq/mpc/gait.hpp"
#include "starq/leg_controller.hpp"
#include "starq/robot_dynamics.hpp"
#include "starq/slam/localization.hpp"

namespace starq::mpc
{

    /// @brief FootholdPlanner class
    class FootholdPlanner
    {
    public:
        using Ptr = std::shared_ptr<FootholdPlanner>;

        /// @brief Create a new FootholdPlanner object
        FootholdPlanner(std::vector<LegController::Ptr> legs,
                        starq::RobotDynamics::Ptr robot_dynamics,
                        starq::slam::Localization::Ptr localization);

        /// @brief Destroy the FootholdPlanner object
        ~FootholdPlanner();

        /// @brief Configure the MPC
        /// @param N The mpc horizon
        /// @param stance_traj The stance trajectory
        /// @param gait_seq The gait sequence
        /// @param ref_traj The reference trajectory
        /// @param foothold_traj The foothold trajectory
        /// @return True if the MPC is configured successfully, false otherwise
        bool configure(const size_t N,
                       const StanceTrajectory &stance_traj, const GaitSequence &gait_seq,
                       const ReferenceTrajectory &ref_traj, FootholdTrajectory &foothold_traj) const;

    private:
        std::vector<LegController::Ptr> legs_;
        starq::RobotDynamics::Ptr robot_dynamics_;
        starq::slam::Localization::Ptr localization_;
    };

}

#endif