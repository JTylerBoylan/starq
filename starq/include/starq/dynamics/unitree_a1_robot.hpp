#ifndef STARQ_DYNAMICS__UNITREE_A1_ROBOT_HPP_
#define STARQ_DYNAMICS__UNITREE_A1_ROBOT_HPP_

#include "starq/robot_dynamics.hpp"

#define UNITREE_A1_MASS 11.75
#define UNITREE_A1_INERTIA 0.016, 0, 0, 0, 0.038, 0, 0, 0, 0.046
#define UNITREE_A1_GRAVITY 0.0, 0.0, -9.81
#define UNITREE_A1_FOOT_FRICTION 0.8

#define UNITREE_A1_HIP_OFFSET_X 0.183
#define UNITREE_A1_HIP_OFFSET_Y 0.047

#define UNITREE_A1_LENGTH_D 0.08505
#define UNITREE_A1_LENGTH_LT 0.2
#define UNITREE_A1_LENGTH_LC 0.2

#define UNITREE_A1_STAND_HEIGHT 0.27

namespace starq::dynamics
{

    /// @brief UnitreeA1RobotDynamics class
    class UnitreeA1RobotDynamics : public RobotDynamics
    {
    public:
        using Ptr = std::shared_ptr<UnitreeA1RobotDynamics>;

        /// @brief Create a new UnitreeA1RobotDynamics object
        UnitreeA1RobotDynamics();

        /// @brief Destroy the UnitreeA1RobotDynamics object
        ~UnitreeA1RobotDynamics();

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
    };

}

#endif