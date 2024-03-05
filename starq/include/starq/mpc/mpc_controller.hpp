#ifndef STARQ_MPC__MPC_CONTROLLER_HPP_
#define STARQ_MPC__MPC_CONTROLLER_HPP_

#include "starq/thread_runner.hpp"
#include "starq/mpc/mpc_configuration.hpp"
#include "starq/mpc/mpc_solver.hpp"
#include "starq/slam/localization.hpp"
#include "starq/robot_dynamics.hpp"
#include "starq/leg_command_publisher.hpp"
#include "starq/trajectory_publisher.hpp"

namespace starq::mpc
{
    using namespace std::chrono;

    /// @brief MPC controller class
    class MPCController : public ThreadRunner
    {
    public:
        using Ptr = std::shared_ptr<MPCController>;

        /// @brief Create a new MPC controller
        MPCController(const MPCConfiguration::Ptr config,
                      const MPCSolver::Ptr solver,
                      const slam::Localization::Ptr localization,
                      const RobotDynamics::Ptr robot_dynamics,
                      const LegCommandPublisher::Ptr leg_command_publisher,
                      const TrajectoryPublisher::Ptr trajectory_publisher);

        /// @brief Destroy the MPC controller
        ~MPCController();

        /// @brief Set the stop on fail flag.
        /// @param stop_on_fail Stop on fail flag.
        void setStopOnFail(const bool stop_on_fail) { stop_on_fail_ = stop_on_fail; }

        /// @brief Set the sleep duration between commands.
        /// @param sleep_duration_us Sleep duration in microseconds.
        void setSleepDuration(const microseconds sleep_duration_us) { sleep_duration_us_ = sleep_duration_us; }

    private:
        void run() override;

        void sendFootForce(const uint8_t leg_id, const Vector3f &force);

        void sendSwingTrajectory(const uint8_t leg_id);

        MPCConfiguration::Ptr config_;
        MPCSolver::Ptr solver_;
        slam::Localization::Ptr localization_;
        RobotDynamics::Ptr robot_dynamics_;
        LegCommandPublisher::Ptr leg_command_publisher_;
        TrajectoryPublisher::Ptr trajectory_publisher_;

        bool stop_on_fail_;
        microseconds sleep_duration_us_;

        FootForceState last_force_state_;
    };

};

#endif