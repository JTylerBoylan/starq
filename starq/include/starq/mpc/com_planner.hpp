#ifndef STARQ_MPC__COM_PLANNER_HPP_
#define STARQ_MPC__COM_PLANNER_HPP_

#include <memory>
#include "starq/mpc/mpc_types.hpp"

#include "starq/slam/localization.hpp"

namespace starq::mpc
{
    /// @brief CenterOfMassPlanner class
    class CenterOfMassPlanner
    {
    public:
        using Ptr = std::shared_ptr<CenterOfMassPlanner>;

        /// @brief Create a new CenterOfMassPlanner object
        /// @param localization The localization object
        CenterOfMassPlanner(starq::slam::Localization::Ptr localization);

        /// @brief Destroy the CenterOfMassPlanner object
        ~CenterOfMassPlanner();

        /// @brief Configure the MPC
        /// @param config The MPC configuration
        /// @return True if the MPC was configured, false otherwise
        bool configure(MPCConfiguration &config) const;

    private:
        starq::slam::Localization::Ptr localization_;
    };
}

#endif