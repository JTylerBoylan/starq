#ifndef STARQ_MPC__MPC_CONFIGURATION_HPP_
#define STARQ_MPC__MPC_CONFIGURATION_HPP_

#include "starq/mpc/mpc_types.hpp"
#include "starq/mpc/gait.hpp"

#include "starq/robot_parameters.hpp"
#include "starq/slam/localization.hpp"
#include "starq/leg_controller.hpp"

#include "starq/mpc/gait_sequencer.hpp"
#include "starq/mpc/reference_planner.hpp"
#include "starq/mpc/foothold_planner.hpp"

namespace starq::mpc
{

    /// @brief MPCConfiguration class
    class MPCConfiguration
    {
    public:
        using Ptr = std::shared_ptr<MPCConfiguration>;

        /// @brief Create a new MPCConfiguration object
        /// @param leg_controllers The leg controllers
        /// @param robot_parameters The robot parameters
        /// @param localization The localization
        MPCConfiguration(const std::vector<LegController::Ptr> &leg_controllers,
                         const RobotParameters::Ptr &robot_parameters,
                         const slam::Localization::Ptr &localization);

        /// @brief Destroy the MPCConfiguration object
        ~MPCConfiguration();

        /// @brief Get the leg controllers
        /// @return The leg controllers
        std::vector<LegController::Ptr> getLegControllers() const;

        /// @brief Get the robot parameters
        /// @return The robot parameters
        RobotParameters::Ptr getRobotParameters() const;

        /// @brief Get the localization
        /// @return The localization
        slam::Localization::Ptr getLocalization() const;

        /// @brief Set the time step
        /// @param time_step The time step in milliseconds
        void setTimeStep(const milliseconds &time_step);

        /// @brief Set the window size
        /// @param window_size The window size
        void setWindowSize(const size_t &window_size);

        /// @brief Set the next gait
        /// @param gait The gait
        void setNextGait(const Gait::Ptr &gait);

        /// @brief Check if the MPC configuration is ready
        /// @return If the MPC configuration is ready
        bool isReady() const;

        /// @brief Update the MPC configuration
        /// @return True if the MPC configuration was updated, false otherwise
        bool update();

        /// @brief Get the time step
        /// @return The time step in seconds
        float getTimeStep() const;

        /// @brief Get the window size
        /// @return The window size
        size_t getWindowSize() const;

        /// @brief Get the stance state at a specific node
        /// @param node The node
        /// @return The stance state
        StanceState getStanceState(const int node) const;

        /// @brief Get the reference state at a specific node
        /// @param node The node
        /// @return The reference state
        ReferenceState getReferenceState(const int node) const;

        /// @brief Get the foothold state at a specific node
        /// @param node The node
        /// @return The foothold state
        FootholdState getFootholdState(const int node) const;

        /// @brief Get the gait at a specific node
        /// @param node The node
        /// @return The gait
        Gait::Ptr getGait(const int node) const;

        /// @brief Get the number of legs at a specific node
        /// @param node The node
        /// @return The number of legs
        size_t getNumberOfLegs(const int node) const;

        /// @brief Get the reference weights at a specific node
        /// @param node The node
        /// @return The reference weights
        ReferenceWeights getReferenceWeights(const int node) const;

        /// @brief Get the force weights at a specific node
        /// @param node The node
        /// @return The force weights
        ForceWeights getForceWeights(const int node) const;

        /// @brief Get the gravity
        /// @return The gravity
        Vector3 getGravity() const;

        /// @brief Get the friction coefficient
        /// @return The friction coefficient
        Float getFrictionCoefficient() const;

        /// @brief Get the minimum force in the z direction
        /// @return The minimum force in the z direction
        Float getForceZMin() const;

        /// @brief Get the maximum force in the z direction
        /// @return The maximum force in the z direction
        Float getForceZMax() const;

        /// @brief Get the mass
        /// @return The mass
        Float getMass() const;

        /// @brief Get the inertia
        /// @return The inertia
        Matrix3 getInertia() const;

    private:
        const std::vector<LegController::Ptr> leg_controllers_;
        const RobotParameters::Ptr robot_parameters_;
        const slam::Localization::Ptr localization_;

        bool is_ready_;

        GaitSequencer::Ptr gait_sequencer_;
        ReferencePlanner::Ptr reference_planner_;
        FootholdPlanner::Ptr foothold_planner_;

        milliseconds time_step_;
        size_t window_size_;

        StanceTrajectory stance_trajectory_;
        GaitSequence gait_sequence_;
        ReferenceTrajectory reference_trajectory_;
        FootholdTrajectory foothold_trajectory_;
        std::vector<size_t> n_legs_;
    };

}

#endif