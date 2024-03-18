#ifndef STARQ__LEG_CONTROLLER_HPP_
#define STARQ__LEG_CONTROLLER_HPP_

#include <vector>

#include "starq/motor_controller.hpp"
#include "starq/leg_dynamics.hpp"

namespace starq
{

    using MotorList = std::vector<MotorController::Ptr>;

    /// @brief Uses LegDynamics to convert leg commands into motor commands
    class LegController
    {

    public:
        using Ptr = std::shared_ptr<LegController>;

        /// @brief Create a leg controller.
        /// @param dynamics Leg dynamics.
        /// @param motor_controllers Motor controllers.
        LegController(const starq::LegDynamics::Ptr dynamics,
                      const MotorList motor_controllers);

        /// @brief Destroy the leg controller.
        ~LegController();

        /// @brief Set the state for a leg.
        /// @param state State.
        /// @return If the command was sent successfully.
        virtual bool setState(const uint32_t state);

        /// @brief Set the control mode for a leg.
        /// @param control_mode Control mode.
        /// @param input_mode Input mode. (default: 0x1)
        /// @return If the command was sent successfully.
        virtual bool setControlMode(const uint32_t control_mode, const uint32_t input_mode = 0x1);

        /// @brief Set the foot position for a leg.
        /// @param foot_position Foot position.
        /// @param foot_velocity_ff Foot velocity feedforward.
        /// @param foot_torque_ff Foot torque feedforward.
        /// @return If the command was sent successfully.
        virtual bool setFootPosition(const Vector3 &foot_position,
                                     const Vector3 &foot_velocity_ff = Vector3::Zero(),
                                     const Vector3 &foot_torque_ff = Vector3::Zero());

        /// @brief Set the foot velocity for a leg.
        /// @param foot_velocity Foot velocity.
        /// @param foot_torque_ff Foot torque feedforward.
        /// @return  If the command was sent successfully.
        virtual bool setFootVelocity(const Vector3 &foot_velocity,
                                     const Vector3 &foot_torque_ff = Vector3::Zero());

        /// @brief Set the foot force for a leg.
        /// @param foot_force Foot force.
        /// @return If the command was sent successfully.
        virtual bool setFootForce(const Vector3 &foot_force);

        /// @brief Get the foot position estimate for a leg.
        /// @param foot_position Foot position.
        /// @return If the position was retrieved successfully.
        virtual bool getFootPositionEstimate(Vector3 &foot_position);

        /// @brief Get the foot velocity estimate for a leg.
        /// @param foot_velocity Foot velocity.
        /// @return If the velocity was retrieved successfully.
        virtual bool getFootVelocityEstimate(Vector3 &foot_velocity);

        /// @brief Get the foot force estimate for a leg.
        /// @param foot_force Foot force.
        /// @return If the force was retrieved successfully.
        virtual bool getFootForceEstimate(Vector3 &foot_force);

        /// @brief Get the current joint angles for a leg.
        /// @return Vector of joint angles.
        Vector3 getCurrentJointAngles();

        /// @brief Get the current joint velocities for a leg.
        /// @return Vector of joint velocities.
        Vector3 getCurrentJointVelocities();

        /// @brief Get the current joint torques for a leg.
        /// @return Vector of joint torques.
        Vector3 getCurrentJointTorques();

        /// @brief Set the joint angles for a leg.
        /// @param joint_angles Joint angles.
        /// @param joint_velocity_ff Joint velocity feedforward.
        /// @param joint_torque_ff Joint torque feedforward.
        /// @return If the command was sent successfully.
        bool setJointAngles(const Vector3 &joint_angles,
                            const Vector3 &joint_velocity_ff,
                            const Vector3 &joint_torque_ff);

        /// @brief Set the joint velocities for a leg.
        /// @param joint_velocities Joint velocities.
        /// @param joint_torque_ff Joint torque feedforward.
        /// @return If the command was sent successfully.
        bool setJointVelocities(const Vector3 &joint_velocities,
                                const Vector3 &joint_torque_ff);

        /// @brief Set the joint torques for a leg.
        /// @param joint_torques Joint torques.
        /// @return If the command was sent successfully.
        bool setJointTorques(const Vector3 &joint_torques);

    protected:
        const starq::LegDynamics::Ptr dynamics_;
        const std::vector<MotorController::Ptr> motor_controllers_;
    };

}

#endif