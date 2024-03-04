#ifndef STARQ_DYNAMICS__UNITREE_A1_ROBOT_HPP_
#define STARQ_DYNAMICS__UNITREE_A1_ROBOT_HPP_

#include "starq/robot_dynamics.hpp"

#define UNITREE_A1_MASS 11.75
#define UNITREE_A1_INERTIA 0.016, 0, 0, 0, 0.038, 0, 0, 0, 0.046
#define UNITREE_A1_GRAVITY 0.0, 0.0, -9.81
#define UNITREE_A1_HEIGHT 0.27
#define UNITREE_A1_FOOT_FRICTION 0.8

#define UNITREE_A1_HIP_OFFSET_X 0.183
#define UNITREE_A1_HIP_OFFSET_Y 0.047

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
        float getBodyMass() const override;

        /// @brief Get the body inertia
        /// @return The body inertia
        Matrix3f getBodyInertia() const override;

        /// @brief Get the gravity vector
        /// @return The gravity vector
        Vector3f getGravity() const override;

        /// @brief Get the body height
        /// @return The body height
        float getBodyHeight() const override;

        /// @brief Get the foot friction
        /// @return The foot friction
        float getFootFriction() const override;

        /// @brief Get the hip locations
        /// @return The hip locations
        std::vector<Vector3f> getHipLocations() const override;

        /// @brief Get the minimum force in the z direction
        /// @return The minimum force in the z direction
        float getForceZMin() const override;

        /// @brief Get the maximum force in the z direction
        /// @return The maximum force in the z direction
        float getForceZMax() const override;

    private:
        float body_mass_;
        Matrix3f body_inertia_;
        Vector3f gravity_;
        float body_height_;
        float foot_friction_;
        std::vector<Vector3f> hip_locations_;
        float force_z_min_;
        float force_z_max_;
    };

}

#endif