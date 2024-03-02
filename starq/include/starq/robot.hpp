#ifndef STARQ__ROBOT_HPP_
#define STARQ__ROBOT_HPP_

#include "starq/motor_controller.hpp"
#include "starq/leg_controller.hpp"
#include "starq/leg_command_publisher.hpp"
#include "starq/trajectory_file_reader.hpp"
#include "starq/trajectory_publisher.hpp"
#include "starq/slam/localization.hpp"

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
        bool setFootPosition(const uint8_t &leg_id, const Eigen::Vector3f &position);

        /// @brief Set the velocity of the foot
        /// @param leg_id The leg id
        /// @param velocity The target velocity [m/s] (x, y, z) in the hip frame
        /// @return True if successful
        bool setFootVelocity(const uint8_t &leg_id, const Eigen::Vector3f &velocity);

        /// @brief Set the force of the foot
        /// @param leg_id The leg id
        /// @param force The target force [N] (x, y, z) in the hip frame
        /// @return True if successful
        bool setFootForce(const uint8_t &leg_id, const Eigen::Vector3f &force);

        /// @brief Load a 2D trajectory from a file
        /// @param file The file path
        /// @return True if successful
        bool load2DTrajectory(const std::string &file);

        /// @brief Load a 3D trajectory from a file
        /// @param file The file path
        /// @return True if successful
        bool load3DTrajectory(const std::string &file);

        /// @brief Start the trajectory
        /// @return True if successful
        bool startTrajectory();

        /// @brief Run a trajectory
        /// @param trajectory The trajectory
        /// @return True if successful
        bool runTrajectory(const std::vector<LegCommand::Ptr> &trajectory);

        /// @brief Get the body mass
        /// @return The body mass [kg]
        float getBodyMass() const { return mass_; }

        /// @brief Get the body inertia
        /// @return The body inertia [kg m^2] (Ixx, Iyy, Izz)
        Matrix3f getBodyInertia() const { return inertia_; }

        /// @brief Get the gravity vector
        /// @return The gravity vector [m/s^2] (x, y, z)
        Vector3f getGravity() const { return gravity_; }

        /// @brief Get the body height
        /// @return The body height [m]
        float getBodyHeight() const { return height_; }

        /// @brief Get the foot friction
        /// @return The foot friction [N/(m/s)]
        float getFootFriction() const { return foot_friction_; }

        /// @brief Get the hip locations
        /// @return The hip locations [m] (x, y, z) in the body frame
        std::vector<Vector3f> getHipLocations() const { return hip_locations_; }

        /// @brief Get the motor controllers
        /// @return The motor controllers
        std::vector<MotorController::Ptr> getMotors() const { return motors_; }

        /// @brief Get the leg controllers
        /// @return The leg controllers
        std::vector<LegController::Ptr> getLegs() const { return legs_; }

        /// @brief Get the localization
        /// @return The localization
        slam::Localization::Ptr getLocalization() const { return localization_; }

        /// @brief Get the leg command publisher
        /// @return The leg command publisher
        LegCommandPublisher::Ptr getLegCommandPublisher() const { return publisher_; }

        /// @brief Get the trajectory file reader
        /// @return The trajectory file reader
        TrajectoryFileReader::Ptr getTrajectoryFileReader() const { return trajectory_file_reader_; }

        /// @brief Get the trajectory publisher
        /// @return The trajectory publisher
        TrajectoryPublisher::Ptr getTrajectoryPublisher() const { return trajectory_publisher_; }

    protected:
        /// @brief Setup the robot parameters (mass, inertia, gravity, hip locations)
        virtual void setupParams() = 0;

        /// @brief Setup the motor controllers
        virtual void setupMotors() = 0;

        /// @brief Setup the leg controllers
        virtual void setupLegs() = 0;

        /// @brief Setup the localization
        virtual void setupLocalization() = 0;

        /// @brief Setup the leg command publisher
        virtual void setupLegCommandPublisher();

        /// @brief Setup the trajectory file reader
        virtual void setupTrajectoryFileReader();

        /// @brief Setup the trajectory publisher
        virtual void setupTrajectoryPublisher();

        float mass_;
        Matrix3f inertia_;
        Vector3f gravity_;
        float height_;
        float foot_friction_;
        std::vector<Vector3f> hip_locations_;
        std::vector<MotorController::Ptr> motors_;
        std::vector<LegController::Ptr> legs_;
        slam::Localization::Ptr localization_;

        LegCommandPublisher::Ptr publisher_;
        TrajectoryFileReader::Ptr trajectory_file_reader_;
        TrajectoryPublisher::Ptr trajectory_publisher_;
    };

}

#endif