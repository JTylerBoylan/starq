#ifndef STARQ_MPC__FOOTHOLD_PLANNER_HPP_
#define STARQ_MPC__FOOTHOLD_PLANNER_HPP_

#include <memory>
#include "starq/mpc/mpc_types.hpp"

#include "starq/leg_controller.hpp"
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
                        starq::slam::Localization::Ptr localization);

        /// @brief Destroy the FootholdPlanner object
        ~FootholdPlanner();

        /// @brief Configure the MPC
        /// @param config The MPC configuration
        /// @return True if the MPC was configured, false otherwise
        bool configure(MPCConfiguration &config) const;

    private:
        std::vector<LegController::Ptr> legs_;
        starq::slam::Localization::Ptr localization_;
    };

}

#endif