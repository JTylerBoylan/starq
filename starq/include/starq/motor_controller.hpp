#ifndef STARQ__MOTOR_CONTROLLER_HPP_
#define STARQ__MOTOR_CONTROLLER_HPP_

#include <memory>

#include "starq/types.hpp"

#define MAX_MOTOR_ID 0x3F

namespace starq
{

    /// @brief Abstract class for motor controllers. Used as a template for derived classes.
    class MotorController
    {
    public:
        using Ptr = std::shared_ptr<MotorController>;

        /// @brief Set the gear ratio
        /// @param gear_ratio The gear ratio to set the motor to.
        /// @return If the gear ratio was set successfully.
        virtual bool setGearRatio(const Float gear_ratio) = 0;

        /// @brief Set the motor state.
        /// @param state The state to set the motor to.
        /// @return If the command was sent successfully.
        virtual bool setState(const uint32_t state) = 0;

        /// @brief Set the control mode.
        /// @param control_mode The control mode to set the motor to.
        /// @param input_mode The input mode to set the motor to. (default: 0x1)
        /// @return If the command was sent successfully.
        virtual bool setControlMode(const uint32_t control_mode, const uint32_t input_mode = 0x1) = 0;

        /// @brief Set the position.
        /// @param pos The position to set the motor to. [rad]
        /// @param vel_ff The velocity feedforward to set the motor to. [rad/s] (default: 0)
        /// @param torque_ff The torque feedforward to set the motor to. [N-m] (default: 0)
        /// @return If the command was sent successfully.
        virtual bool setPosition(const Float pos, const Float vel_ff = 0.F, const Float torque_ff = 0.F) = 0;

        /// @brief Set the velocity.
        /// @param vel The velocity to set the motor to. [rad/s]
        /// @param torque_ff The torque feedforward to set the motor to. [N-m] (default: 0)
        /// @return If the command was sent successfully.
        virtual bool setVelocity(const Float vel, const Float torque_ff = 0.F) = 0;

        /// @brief Set the torque.
        /// @param torque The torque to set the motor to. [N-m]
        /// @return If the command was sent successfully.
        virtual bool setTorque(const Float torque) = 0;

        /// @brief Get the encoder position estimate.
        /// @return The encoder position estimate in radians.
        virtual Float getPositionEstimate() = 0;

        /// @brief Get the encoder velocity estimate.
        /// @return The encoder velocity estimate in radians per second.
        virtual Float getVelocityEstimate() = 0;

        /// @brief Get the controller torque estimate.
        /// @return The controller torque estimate in Newton-meters.
        virtual Float getTorqueEstimate() = 0;
    };

}

#endif