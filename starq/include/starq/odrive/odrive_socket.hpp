#ifndef STARQ_ODRIVE__ODRIVE_SOCKET_HPP_
#define STARQ_ODRIVE__ODRIVE_SOCKET_HPP_

#include "starq/can/can_socket.hpp"
#include "starq/thread_runner.hpp"

#include <thread>
#include <mutex>

namespace starq::odrive
{

    /// @brief Send ODrive commands and listen for ODrive information over CAN bus
    class ODriveSocket : public ThreadRunner
    {
    public:
        using Ptr = std::shared_ptr<ODriveSocket>;

        /// @brief Create an ODrive socket.
        /// @param socket CAN socket to use for communication.
        ODriveSocket(const starq::can::CANSocket::Ptr socket);

        /// @brief Destroy the ODrive socket.
        ~ODriveSocket();

        /// @brief Set the axis state.
        /// @param can_id CAN ID of the axis.
        /// @param state Axis state.
        /// @return If the command was sent successfully.
        bool setAxisState(const uint8_t can_id, const uint32_t state);

        /// @brief Set the control mode.
        /// @param can_id CAN ID of the axis.
        /// @param control_mode Control mode.
        /// @param input_mode Input mode.
        /// @return If the command was sent successfully.
        bool setControlMode(const uint8_t can_id, const uint32_t control_mode, const uint32_t input_mode = 0x1);

        /// @brief Set the limits.
        /// @param can_id CAN ID of the axis.
        /// @param velocity_limit Velocity limit.
        /// @param current_limit Current limit.
        /// @return If the command was sent successfully.
        bool setLimits(const uint8_t can_id, const float velocity_limit, const float current_limit);

        /// @brief Set the position gain.
        /// @param can_id CAN ID of the axis.
        /// @param pos_gain Position gain.
        /// @return If the command was sent successfully.
        bool setPosGain(const uint8_t can_id, const float pos_gain);

        /// @brief Set the velocity gains.
        /// @param can_id CAN ID of the axis.
        /// @param vel_gain Velocity gain.
        /// @param vel_integrator_gain Velocity integrator gain.
        /// @return If the command was sent successfully.
        bool setVelGains(const uint8_t can_id, const float vel_gain, const float vel_integrator_gain);

        /// @brief Set the position.
        /// @param can_id CAN ID of the axis.
        /// @param pos Position.
        /// @param vel_ff Feedforward velocity.
        /// @param torque_ff Feedforward torque.
        /// @return If the command was sent successfully.
        bool setPosition(const uint8_t can_id, const float pos, const float vel_ff = 0.F, const float torque_ff = 0.F);

        /// @brief Set the velocity.
        /// @param can_id CAN ID of the axis.
        /// @param vel Velocity.
        /// @param torque_ff Feedforward torque.
        /// @return If the command was sent successfully.
        bool setVelocity(const uint8_t can_id, const float vel, const float torque_ff = 0.F);

        /// @brief Set the torque.
        /// @param can_id CAN ID of the axis.
        /// @param torque Torque.
        /// @return If the command was sent successfully.
        bool setTorque(const uint8_t can_id, const float torque);

        /// @brief Clear errors.
        /// @param can_id CAN ID of the axis.
        /// @return If the command was sent successfully.
        bool clearErrors(const uint8_t can_id);

        /// @brief Get the axis error.
        /// @param can_id CAN ID of the axis.
        /// @return Axis error.
        uint32_t getAxisError(const uint8_t can_id);

        /// @brief Get the axis state.
        /// @param can_id CAN ID of the axis.
        /// @return Axis state.
        uint8_t getAxisState(const uint8_t can_id);

        /// @brief Get the Iq setpoint.
        /// @param can_id CAN ID of the axis.
        /// @return Iq setpoint.
        float getIqSetpoint(const uint8_t can_id);

        /// @brief Get the Iq measured.
        /// @param can_id CAN ID of the axis.
        /// @return Iq measured.
        float getIqMeasured(const uint8_t can_id);

        /// @brief Get the FET temperature.
        /// @param can_id CAN ID of the axis.
        /// @return FET temperature.
        float getFETTemperature(const uint8_t can_id);

        /// @brief Get the motor temperature.
        /// @param can_id CAN ID of the axis.
        /// @return Motor temperature.
        float getMotorTemperature(const uint8_t can_id);

        /// @brief Get the bus voltage.
        /// @param can_id CAN ID of the axis.
        /// @return Bus voltage.
        float getBusVoltage(const uint8_t can_id);

        /// @brief Get the bus current.
        /// @param can_id CAN ID of the axis.
        /// @return Bus current.
        float getBusCurrent(const uint8_t can_id);

        /// @brief Get the position estimate.
        /// @param can_id CAN ID of the axis.
        /// @return Position estimate.
        float getPosEstimate(const uint8_t can_id);

        /// @brief Get the velocity estimate.
        /// @param can_id CAN ID of the axis.
        /// @return Velocity estimate.
        float getVelEstimate(const uint8_t can_id);

        /// @brief Get the torque estimate.
        /// @param can_id CAN ID of the axis.
        /// @return Torque estimate.
        float getTorqueEstimate(const uint8_t can_id);

    private:

        /// @brief Run the listener. (Called by the poll thread.)
        void run() override;

        /// @brief Get the arbitration ID.
        /// @param can_id CAN ID of the axis.
        /// @param cmd_id Command ID.
        /// @return Arbitration ID.
        inline uint32_t getArbitrationID(const uint8_t can_id, const uint8_t cmd_id) const
        {
            return (uint32_t)(can_id << 5) | cmd_id;
        }

        /// @brief Get the CAN ID.
        /// @param arb_id Arbitration ID.
        /// @return CAN ID.
        uint8_t getCanID(const uint32_t arb_id) const
        {
            return (uint8_t)((arb_id >> 5) & 0b111111);
        }

        /// @brief Get the command ID.
        /// @param arb_id Arbitration ID.
        /// @return Command ID.
        uint8_t getCommandID(const uint32_t arb_id) const
        {
            return (uint8_t)(arb_id & 0b11111);
        }

        const starq::can::CANSocket::Ptr socket_;

        struct
        {
            uint32_t axis_error = 0;
            uint8_t axis_state = 0;
            float iq_setpoint = 0.0f;
            float iq_measured = 0.0f;
            float fet_temperature = 0.0f;
            float motor_temperature = 0.0f;
            float bus_voltage = 0.0f;
            float bus_current = 0.0f;
            float pos_estimate = 0.0f;
            float vel_estimate = 0.0f;
            float torque_estimate = 0.0f;
        } info_[MAX_CAN_ID + 1];
    };

}

#endif