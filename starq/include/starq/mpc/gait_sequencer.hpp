#ifndef STARQ_MPC__GAIT_SEQUENCER_HPP_
#define STARQ_MPC__GAIT_SEQUENCER_HPP_

#include <memory>

#include "starq/slam/localization.hpp"
#include "starq/mpc/gait.hpp"

namespace starq::mpc
{
    /// @brief GaitSequencer class
    class GaitSequencer
    {
    public:
        using Ptr = std::shared_ptr<GaitSequencer>;

        /// @brief Create a new GaitSequencer object
        /// @param localization The localization object
        GaitSequencer(starq::slam::Localization::Ptr localization);

        /// @brief Destroy the GaitSequencer object
        ~GaitSequencer();

        /// @brief Set the next gait pattern
        /// @param gait The next gait pattern
        void setNextGait(Gait::Ptr gait);

        /// @brief Synchronize the gait sequencer
        /// @return True if the gait sequencer was synchronized, false otherwise
        bool sync();

        /// @brief Configure the MPC
        /// @param config The MPC configuration
        /// @return True if the MPC was configured, false otherwise
        bool configure(MPCConfiguration &config) const;

        /// @brief Get the stance state
        /// @param time The time in milliseconds
        /// @return The stance state
        StanceState getStanceState(const milliseconds &time) const;

        /// @brief Get the gait at a specific time
        /// @param time The time in milliseconds
        /// @return The gait
        Gait::Ptr getGait(const milliseconds &time) const;

    private:
        starq::slam::Localization::Ptr localization_;

        milliseconds start_time_;

        Gait::Ptr current_gait_;
        Gait::Ptr next_gait_;
    };
}

#endif