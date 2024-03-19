#ifndef STARQ__ROBOT_HPP_
#define STARQ__ROBOT_HPP_

#include "starq/motor_controller.hpp"
#include "starq/leg_controller.hpp"
#include "starq/leg_command_publisher.hpp"
#include "starq/trajectory_file_reader.hpp"
#include "starq/trajectory_publisher.hpp"
#include "starq/slam/localization.hpp"
#include "starq/robot_dynamics.hpp"
#include "starq/mpc/mpc_controller.hpp"

namespace starq
{

    /// @brief Robot class
    class Robot
    {
    public:
        using Ptr = std::shared_ptr<Robot>;

        /// @brief Setup the robot
        void setup();

        /// @brief Set the position of the foot
        /// @param leg_id The leg id
        /// @param position The target position [m] (x, y, z) in the hip frame
        /// @return True if successful
        bool setFootPosition(const uint8_t &leg_id, const Vector3 &position);

        /// @brief Set the velocity of the foot
        /// @param leg_id The leg id
        /// @param velocity The target velocity [m/s] (x, y, z) in the hip frame
        /// @return True if successful
        bool setFootVelocity(const uint8_t &leg_id, const Vector3 &velocity);

        /// @brief Set the force of the foot
        /// @param leg_id The leg id
        /// @param force The target force [N] (x, y, z) in the hip frame
        /// @return True if successful
        bool setFootForce(const uint8_t &leg_id, const Vector3 &force);

        /// @brief Load a 3D trajectory from a file
        /// @param file The file path
        /// @return True if successful
        bool loadTrajectory(const std::string &file);

        /// @brief Start the trajectory
        /// @return True if successful
        bool startTrajectory();

        /// @brief Run a trajectory
        /// @param trajectory The trajectory
        /// @return True if successful
        bool runTrajectory(const std::vector<LegCommand::Ptr> &trajectory);

        /// @brief Start the model predictive control
        /// @return True if successful
        bool startMPC();

        /// @brief Stop the model predictive control
        /// @return True if successful
        bool stopMPC();

        /// @brief Set the next gait for the model predictive control
        /// @param gait The gait
        void setNextGait(mpc::Gait::Ptr gait);

        /// @brief Get the motor controllers
        /// @return The motor controllers
        std::vector<MotorController::Ptr> getMotors() const { return motors_; }

        /// @brief Get the leg controllers
        /// @return The leg controllers
        std::vector<LegController::Ptr> getLegs() const { return legs_; }

        /// @brief Get the localization
        /// @return The localization
        slam::Localization::Ptr getLocalization() const { return localization_; }

        /// @brief Get the robot dynamics
        /// @return The robot dynamics
        RobotDynamics::Ptr getRobotDynamics() const { return robot_dynamics_; }

        /// @brief Get the model predictive control solver
        /// @return The model predictive control solver
        mpc::MPCSolver::Ptr getMPCSolver() const { return mpc_solver_; }

        /// @brief Get the leg command publisher
        /// @return The leg command publisher
        LegCommandPublisher::Ptr getLegCommandPublisher() const { return publisher_; }

        /// @brief Get the trajectory file reader
        /// @return The trajectory file reader
        TrajectoryFileReader::Ptr getTrajectoryFileReader() const { return trajectory_file_reader_; }

        /// @brief Get the trajectory publisher
        /// @return The trajectory publisher
        TrajectoryPublisher::Ptr getTrajectoryPublisher() const { return trajectory_publisher_; }

        /// @brief Get the model predictive control configuration
        /// @return The model predictive control configuration
        mpc::MPCConfiguration::Ptr getMPCConfiguration() const { return mpc_configuration_; }

        /// @brief Get the model predictive control controller
        /// @return The model predictive control controller
        mpc::MPCController::Ptr getMPCController() const { return mpc_controller_; }

    protected:

        /// @brief Setup the motor controllers
        virtual void setupMotors() = 0;

        /// @brief Setup the leg controllers
        virtual void setupLegs() = 0;

        /// @brief Setup the localization
        virtual void setupLocalization() = 0;

        /// @brief Setup the robot dynamics
        virtual void setupRobotDynamics() = 0;

        /// @brief Setup the model predictive control solver
        virtual void setupMPCSolver() = 0;

        /// @brief Setup the leg command publisher
        virtual void setupLegCommandPublisher();

        /// @brief Setup the trajectory file reader
        virtual void setupTrajectoryFileReader();

        /// @brief Setup the trajectory publisher
        virtual void setupTrajectoryPublisher();

        /// @brief Setup the model predictive control configuration
        virtual void setupMPCConfiguration();

        /// @brief Setup the model predictive control controller
        virtual void setupMPCController();

        std::vector<MotorController::Ptr> motors_;
        std::vector<LegController::Ptr> legs_;
        slam::Localization::Ptr localization_;
        RobotDynamics::Ptr robot_dynamics_;
        mpc::MPCSolver::Ptr mpc_solver_;

        LegCommandPublisher::Ptr publisher_;
        TrajectoryFileReader::Ptr trajectory_file_reader_;
        TrajectoryPublisher::Ptr trajectory_publisher_;
        mpc::MPCConfiguration::Ptr mpc_configuration_;
        mpc::MPCController::Ptr mpc_controller_;

    };

}

#endif