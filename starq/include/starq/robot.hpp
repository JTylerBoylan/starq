#ifndef STARQ__ROBOT_HPP_
#define STARQ__ROBOT_HPP_

#include "starq/motor_controller.hpp"
#include "starq/leg_controller.hpp"
#include "starq/leg_command_publisher.hpp"
#include "starq/trajectory_file_reader.hpp"
#include "starq/trajectory_publisher.hpp"
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

        /// @brief Cleanup the robot
        void cleanup();

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

        /// @brief Load a 3D trajectory from a file
        /// @param file The file path
        /// @param frequency The frequency of the trajectory
        /// @return True if successful
        bool loadTrajectory(const std::string &file, const Float frequency = 1.0);

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

        /// @brief Get the robot parameters
        /// @return The robot parameters
        RobotParameters::Ptr getRobotParameters() const { return robot_parameters_; }

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
        RobotParameters::Ptr robot_parameters_;
        mpc::MPCSolver::Ptr mpc_solver_;

        LegCommandPublisher::Ptr publisher_;
        TrajectoryFileReader::Ptr trajectory_file_reader_;
        TrajectoryPublisher::Ptr trajectory_publisher_;
        mpc::MPCConfiguration::Ptr mpc_configuration_;
        mpc::MPCController::Ptr mpc_controller_;
    };

}

#endif