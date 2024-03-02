#ifndef STARQ_MPC__MPC_PLANNER_HPP_
#define STARQ_MPC__MPC_PLANNER_HPP_

#include <memory>

#include "starq/robot.hpp"

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
        /// @param robot The robot
        MPCPlanner(starq::Robot::Ptr robot);

        /// @brief Destroy the MPCPlanner object
        ~MPCPlanner();

        /// @brief Set the time step
        /// @param time_step The time step in milliseconds
        void setTimeStep(const milliseconds &time_step);

        /// @brief Set the window size
        /// @param window_size The window size
        void setWindowSize(const size_t &window_size);

        /// @brief Set the weight for the state
        /// @param position_weights Weight for the position
        /// @param orientation_weights Weight for the orientation
        /// @param linear_velocity_weights Weight for the linear velocity
        /// @param angular_velocity_weights Weight for the angular velocity
        void setStateWeights(const Vector3f &position_weights,
                             const Vector3f &orientation_weights,
                             const Vector3f &linear_velocity_weights,
                             const Vector3f &angular_velocity_weights);

        /// @brief Set the weight for the control
        /// @param force_weights Weight for the force
        void setControlWeights(const Vector3f &force_weights);

        /// @brief Set the control bounds
        /// @param fz_min Minimum Z force
        /// @param fz_max Maximum Z force
        void setControlBounds(const float &fz_min,
                              const float &fz_max);

        /// @brief Set the next gait pattern
        /// @param gait The next gait pattern
        void setNextGait(Gait::Ptr gait);

        /// @brief Get the configuration
        /// @param config The MPC configuration to be filled
        /// @return True if the configuration was obtained, false otherwise
        bool getConfiguration(MPCConfiguration &config);

    private:
        starq::Robot::Ptr robot_;

        milliseconds time_step_;
        size_t window_size_;

        Vector3f position_weights_;
        Vector3f orientation_weights_;
        Vector3f linear_velocity_weights_;
        Vector3f angular_velocity_weights_;
        Vector3f force_weights_;

        float fz_min_;
        float fz_max_;

        GaitSequencer::Ptr gait_sequencer_;
        CenterOfMassPlanner::Ptr com_planner_;
        FootholdPlanner::Ptr foothold_planner_;
    };

}

#endif