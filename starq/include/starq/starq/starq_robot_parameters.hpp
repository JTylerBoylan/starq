#ifndef STARQ_STARQ__ROBOT_PARAMETERS_HPP_
#define STARQ_STARQ__ROBOT_PARAMETERS_HPP_

#include "starq/robot_parameters.hpp"

#define STARQ_MASS 13.0
#define STARQ_INERTIA 0.5, 0, 0, 0, 2.0, 0, 0, 0, 1.0
#define STARQ_GRAVITY 0.0, 0.0, -9.81
#define STARQ_FOOT_FRICTION 0.8

#define STARQ_HIP_OFFSET_X 0.35
#define STARQ_HIP_OFFSET_Y 0.15

#define STARQ_LINK_LENGTH_1 0.065
#define STARQ_LINK_LENGTH_2 0.2

#define STARQ_STAND_HEIGHT 0.18914

#define STARQ_Z_FORCE_MIN 10
#define STARQ_Z_FORCE_MAX 250

#define STARQ_IS_2D true

#define STARQ_MOTOR_GEAR_RATIO 6.0
#define STARQ_MOTOR_P_GAIN 100.0
#define STARQ_MOTOR_V_GAIN 0.05
#define STARQ_MOTOR_VI_GAIN 0.15
#define STARQ_MOTOR_MAX_VELOCITY 25.0
#define STARQ_MOTOR_MAX_CURRENT 14.0

namespace starq
{

    class STARQRobotParameters : public RobotParameters
    {
    public:
        using Ptr = std::shared_ptr<STARQRobotParameters>;

        /// @brief Create a STARQ robot parameters object
        STARQRobotParameters();

        /// @brief Get the body mass
        /// @return The body mass
        Float getBodyMass() const override;

        /// @brief Get the body inertia
        /// @return The body inertia
        Matrix3 getBodyInertia() const override;

        /// @brief Get the gravity vector
        /// @return The gravity vector
        Vector3 getGravity() const override;

        /// @brief Get the foot friction
        /// @return The foot friction
        Float getFootFriction() const override;

        /// @brief Get the standing height
        /// @return The standing height
        Float getStandingHeight() const override;

        /// @brief Get the hip locations
        /// @return The hip locations
        std::vector<Vector3> getHipLocations() const override;

        /// @brief Get the default foot locations
        /// @return The default foot locations
        std::vector<Vector3> getDefaultFootLocations() const override;

        /// @brief Get the minimum force in the z direction
        /// @return The minimum force in the z direction
        Float getForceZMin() const override;

        /// @brief Get the maximum force in the z direction
        /// @return The maximum force in the z direction
        Float getForceZMax() const override;

        /// @brief Check if the robot is 2D
        /// @return If the robot is 2D
        bool is2D() const override;

    private:
        Float body_mass_;
        Matrix3 body_inertia_;
        Vector3 gravity_;
        Float foot_friction_;
        Float standing_height_;
        std::vector<Vector3> hip_locations_;
        std::vector<Vector3> default_foot_locations_;
        Float force_z_min_;
        Float force_z_max_;
        bool is_2d_;
    };
}

#endif