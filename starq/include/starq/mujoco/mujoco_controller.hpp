#ifndef STARQ_MUJOCO__MUJOCO_CONTROLLER_HPP
#define STARQ_MUJOCO__MUJOCO_CONTROLLER_HPP

#include <memory>

#include "starq/motor_controller.hpp"
#include "starq/mujoco/mujoco.hpp"

namespace starq::mujoco
{

    class MuJoCoController : public MotorController
    {
    public:
        using Ptr = std::shared_ptr<MuJoCoController>;

        /// @brief Create a MuJoCoController
        /// @param mujoco MuJoCo instance
        /// @param motor_id Motor ID
        MuJoCoController(MuJoCo::Ptr mujoco, const int motor_id);

        /// @brief Set the gear ratio
        /// @param gear_ratio The gear ratio to set the motor to.
        /// @return If the gear ratio was set successfully.
        bool setGearRatio(const float gear_ratio) override;

        /// @brief Set the motor state.
        /// @param state The state to set the motor to.
        /// @return If the command was sent successfully.
        bool setState(const uint32_t state) override;

        /// @brief Set the control mode.
        /// @param control_mode The control mode to set the motor to.
        /// @param input_mode The input mode to set the motor to. (default: 0x1)
        /// @return If the command was sent successfully.
        bool setControlMode(const uint32_t control_mode, const uint32_t input_mode = 0x1) override;

        /// @brief Set the position.
        /// @param pos The position to set the motor to. [rad]
        /// @param vel_ff The velocity feedforward to set the motor to. [rad/s] (default: 0)
        /// @param torque_ff The torque feedforward to set the motor to. [N-m] (default: 0)
        /// @return If the command was sent successfully.
        bool setPosition(const float pos, const float vel_ff = 0.F, const float torque_ff = 0.F) override;

        /// @brief Set the velocity.
        /// @param vel The velocity to set the motor to. [rad/s]
        /// @param torque_ff The torque feedforward to set the motor to. [N-m] (default: 0)
        /// @return If the command was sent successfully.
        bool setVelocity(const float vel, const float torque_ff = 0.F) override;

        /// @brief Set the torque.
        /// @param torque The torque to set the motor to. [N-m]
        /// @return If the command was sent successfully.
        bool setTorque(const float torque) override;

        /// @brief Get the encoder position estimate.
        /// @return The encoder position estimate in radians.
        float getPositionEstimate() override;

        /// @brief Get the encoder velocity estimate.
        /// @return The encoder velocity estimate in radians per second.
        float getVelocityEstimate() override;

        /// @brief Get the controller torque estimate.
        /// @return The controller torque estimate in Newton-meters.
        float getTorqueEstimate() override;

        /// @brief Set the position gain.
        /// @param pos_gain The position gain to set the motor to.
        /// @param vel_gain The velocity gain to set the motor to.
        /// @param integrator_gain The velocity integrator gain to set the motor to.
        /// @return If the command was sent successfully.
        bool setGains(const float pos_gain, const float vel_gain, const float integrator_gain);

    private:
        /// @brief Control the motor
        void controlMotor(const mjModel *model, mjData *data);

        MuJoCo::Ptr mujoco_;
        int motor_id_;

        struct
        {
            float gear_ratio = 1.0;

            uint32_t state = 0x0;
            uint32_t control_mode = ControlMode::TORQUE;
            uint32_t input_mode = 0x1;

            float pos_cmd = 0.f;
            float vel_ff_cmd = 0.f, torq_ff_cmd = 0.f;
            float vel_cmd = 0.f;
            float torq_cmd = 0.f;

            float pos_gain = 1.0f;
            float vel_gain = 0.5f;
            float int_gain = 0.5f;

            float torq_integral = 0.f;

            float pos_est, vel_est, torq_est;
        } state_;
    };

}

#endif