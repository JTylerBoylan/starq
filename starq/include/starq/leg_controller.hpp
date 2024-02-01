#ifndef STARQ__LEG_CONTROLLER_HPP_
#define STARQ__LEG_CONTROLLER_HPP_

#include <vector>

#include "starq/motor_controller.hpp"
#include "starq/leg_dynamics.hpp"

namespace starq
{
    using namespace Eigen;

    class LegController
    {

    public:
        using Ptr = std::shared_ptr<LegController>;

        /// @brief Create a leg controller.
        /// @param controller Motor controller.
        LegController(const starq::LegDynamics::Ptr dynamics,
                      const std::vector<MotorController::Ptr> motor_controllers);

        /// @brief Destroy the leg controller.
        ~LegController();

        /// @brief Set the state for a leg.
        /// @param state State.
        /// @return If the command was sent successfully.
        bool setState(const uint32_t state);

        /// @brief Set the control mode for a leg.
        /// @param control_mode Control mode.
        /// @param input_mode Input mode. (default: 0x1)
        /// @return If the command was sent successfully.
        bool setControlMode(const uint32_t control_mode, const uint32_t input_mode = 0x1);

        /// @brief Set the foot position for a leg.
        /// @param foot_position Foot position.
        /// @param foot_velocity_ff Foot velocity feedforward.
        /// @param foot_torque_ff Foot torque feedforward.
        /// @return If the command was sent successfully.
        bool setFootPosition(const VectorXf &foot_position,
                             const VectorXf &foot_velocity_ff = VectorXf(),
                             const VectorXf &foot_torque_ff = VectorXf());

        /// @brief Set the foot velocity for a leg.
        /// @param foot_velocity Foot velocity.
        /// @param foot_torque_ff Foot torque feedforward.
        /// @return  If the command was sent successfully.
        bool setFootVelocity(const VectorXf &foot_velocity,
                             const VectorXf &foot_torque_ff = VectorXf());

        /// @brief Set the foot force for a leg.
        /// @param foot_force Foot force.
        /// @return If the command was sent successfully.
        bool setFootForce(const VectorXf &foot_force);

        /// @brief Get the foot position estimate for a leg.
        /// @param foot_position Foot position.
        /// @return If the position was retrieved successfully.
        bool getFootPositionEstimate(VectorXf &foot_position);

        /// @brief Get the foot velocity estimate for a leg.
        /// @param foot_velocity Foot velocity.
        /// @return If the velocity was retrieved successfully.
        bool getFootVelocityEstimate(VectorXf &foot_velocity);

        /// @brief Get the foot force estimate for a leg.
        /// @param foot_force Foot force.
        /// @return If the force was retrieved successfully.
        bool getFootForceEstimate(VectorXf &foot_force);

        /// @brief Get the current joint angles for a leg.
        /// @return Vector of joint angles.
        VectorXf getCurrentJointAngles();

        /// @brief Get the current joint velocities for a leg.
        /// @return Vector of joint velocities.
        VectorXf getCurrentJointVelocities();

        /// @brief Get the current joint torques for a leg.
        /// @return Vector of joint torques.
        VectorXf getCurrentJointTorques();

        /// @brief Set the joint angles for a leg.
        /// @param joint_angles Joint angles.
        /// @param joint_velocity_ff Joint velocity feedforward.
        /// @param joint_torque_ff Joint torque feedforward.
        /// @return If the command was sent successfully.
        bool setJointAngles(const VectorXf &joint_angles,
                            const VectorXf &joint_velocity_ff,
                            const VectorXf &joint_torque_ff);

        /// @brief Set the joint velocities for a leg.
        /// @param joint_velocities Joint velocities.
        /// @param joint_torque_ff Joint torque feedforward.
        /// @return If the command was sent successfully.
        bool setJointVelocities(const VectorXf &joint_velocities,
                                const VectorXf &joint_torque_ff);

        /// @brief Set the joint torques for a leg.
        /// @param joint_torques Joint torques.
        /// @return If the command was sent successfully.
        bool setJointTorques(const VectorXf &joint_torques);

    private:
        const starq::LegDynamics::Ptr dynamics_;
        const std::vector<MotorController::Ptr> motor_controllers_;
    };

}

#endif