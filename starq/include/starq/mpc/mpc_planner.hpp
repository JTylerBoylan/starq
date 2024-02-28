#ifndef STARQ_MPC__MPC_PLANNER_HPP_
#define STARQ_MPC__MPC_PLANNER_HPP_

#include <memory>

#include "starq/leg_controller.hpp"
#include "starq/slam/localization.hpp"

#include "starq/mpc/mpc_types.hpp"
#include "starq/mpc/gait_sequencer.hpp"
#include "starq/mpc/com_planner.hpp"
#include "starq/mpc/foothold_planner.hpp"

namespace starq::mpc
{

    /// @brief MPCPlanner class
    class MPCPlanner
    {
    public:
        using Ptr = std::shared_ptr<MPCPlanner>;

        /// @brief Create a new MPCPlanner object
        /// @param legs The leg controllers
        /// @param localization The localization object
        MPCPlanner(std::vector<LegController::Ptr> legs,
                   starq::slam::Localization::Ptr localization);

        /// @brief Destroy the MPCPlanner object
        ~MPCPlanner();

        /// @brief Set the mass
        /// @param mass The mass in kg
        void setMass(const float &mass);

        /// @brief Set the inertia
        /// @param inertia The inertia matrix in kg*m^2
        void setInertia(const Matrix3f &inertia);

        /// @brief Set the gravity
        /// @param gravity The gravity vector in m/s^2
        void setGravity(const Vector3f &gravity);

        /// @brief Set the time step
        /// @param time_step The time step in milliseconds
        void setTimeStep(const milliseconds &time_step);

        /// @brief Set the window size
        /// @param window_size The window size
        void setWindowSize(const size_t &window_size);

        /// @brief Set the next gait pattern
        /// @param gait The next gait pattern
        void setNextGait(Gait::Ptr gait);

        /// @brief Get the configuration
        /// @param config The MPC configuration to be filled
        /// @return True if the configuration was obtained, false otherwise
        bool getConfiguration(MPCConfiguration &config);

    private:
        GaitSequencer::Ptr gait_sequencer_;
        CenterOfMassPlanner::Ptr com_planner_;
        FootholdPlanner::Ptr foothold_planner_;

        float mass_;
        Matrix3f inertia_;
        Vector3f gravity_;

        milliseconds time_step_;
        size_t window_size_;
    };

}

#endif