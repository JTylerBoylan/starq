#ifndef STARQ__ROBOT_HPP_
#define STARQ__ROBOT_HPP_

#include "starq/motor_controller.hpp"
#include "starq/leg_controller.hpp"
#include "starq/leg_command_publisher.hpp"
#include "starq/trajectory_controller.hpp"
#include "starq/slam/localization.hpp"
#include "starq/robot_parameters.hpp"
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

        /// @brief Set the state for all motors
        /// @param state The state
        /// @return True if successful
        bool setStates(AxisState state);

        /// @brief Set the gear ratios for all motors
        /// @param gear_ratio The gear ratio
        /// @return True if successful
        bool setGearRatios(const Float gear_ratio);

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

        /// @brief Send all feet to the default position
        /// @return True if successful
        bool goToDefaultFootLocations();

        /// @brief Run a trajectory
        /// @param trajectory The trajectory
        /// @param frequency The frequency [Hz]
        /// @param num_loops The number of loops
        /// @return True if successful
        bool runTrajectory(const Trajectory &trajectory, const Float frequency = 1.0, const std::size_t num_loops = 1);

        /// @brief Run a trajectory from a file
        /// @param file_path The file path
        /// @param frequency The frequency [Hz]
        /// @param num_loops The number of loops
        /// @return True if successful
        bool runTrajectory(const std::string &file_path, const Float frequency = 1.0, const std::size_t num_loops = 1);

        /// @brief Set the next trajectory
        /// @param trajectory The trajectory
        /// @param frequency The frequency [Hz]
        /// @return True if successful
        bool setNextTrajectory(const Trajectory &trajectory, const Float frequency = 1.0);

        /// @brief Set the next trajectory from a file
        /// @param file_path The file path
        /// @param frequency The frequency [Hz]
        /// @return True if successful
        bool setNextTrajectory(const std::string &file_path, const Float frequency = 1.0);

        /// @brief Stop the trajectory
        /// @return True if successful
        bool stopTrajectory();

        /// @brief Run the model predictive control
        /// @param gait The gait to run
        bool runMPCGait(mpc::Gait::Ptr gait);

        /// @brief Stop the model predictive control
        /// @return True if successful
        bool stopMPC();

        /// @brief Get the motor controllers
        /// @return The motor controllers
        std::vector<MotorController::Ptr> getMotors() const { return motors_; }

        /// @brief Get the leg controllers
        /// @return The leg controllers
        std::vector<LegController::Ptr> getLegs() const { return legs_; }

        /// @brief Get the localization
        /// @return The localization
        slam::Localization::Ptr getLocalization() const { return localization_; }

        /// @brief Get the robot parameters
        /// @return The robot parameters
        RobotParameters::Ptr getRobotParameters() const { return robot_parameters_; }

        /// @brief Get the model predictive control solver
        /// @return The model predictive control solver
        mpc::MPCSolver::Ptr getMPCSolver() const { return mpc_solver_; }

        /// @brief Get the leg command publisher
        /// @return The leg command publisher
        LegCommandPublisher::Ptr getLegCommandPublisher() const { return publisher_; }

        /// @brief Get the trajectory controller
        /// @return The trajectory controller
        TrajectoryController::Ptr getTrajectoryController() const { return trajectory_controller_; }

        /// @brief Get the model predictive control configuration
        /// @return The model predictive control configuration
        mpc::MPCConfiguration::Ptr getMPCConfiguration() const { return mpc_configuration_; }

        /// @brief Get the model predictive control controller
        /// @return The model predictive control controller
        mpc::MPCController::Ptr getMPCController() const { return mpc_controller_; }

    protected:
        /// @brief Setup the motor controllers
        virtual void setupMotorControllers() = 0;

        /// @brief Setup the leg controllers
        virtual void setupLegControllers() = 0;

        /// @brief Setup the localization
        virtual void setupLocalization() = 0;

        /// @brief Setup the robot dynamics
        virtual void setupRobotParameters() = 0;

        /// @brief Setup the model predictive control solver
        virtual void setupMPCSolver() = 0;

        /// @brief Setup the leg command publisher
        virtual void setupLegCommandPublisher();

        /// @brief Setup the trajectory controller
        virtual void setupTrajectoryController();

        /// @brief Setup the model predictive control configuration
        virtual void setupMPCConfiguration();

        /// @brief Setup the model predictive control controller
        virtual void setupMPCController();

        std::vector<MotorController::Ptr> motors_;
        std::vector<LegController::Ptr> legs_;
        slam::Localization::Ptr localization_;
        RobotParameters::Ptr robot_parameters_;
        mpc::MPCSolver::Ptr mpc_solver_;

        LegCommandPublisher::Ptr publisher_;
        TrajectoryController::Ptr trajectory_controller_;
        mpc::MPCConfiguration::Ptr mpc_configuration_;
        mpc::MPCController::Ptr mpc_controller_;
    };

}

#endif