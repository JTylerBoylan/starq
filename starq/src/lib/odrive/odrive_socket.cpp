#include "starq/odrive/odrive_socket.hpp"

#include <iostream>

namespace starq::odrive
{

    ODriveSocket::ODriveSocket(const starq::can::CANSocket::Ptr socket)
        : socket_(socket)
    {
        start();
    }

    ODriveSocket::~ODriveSocket()
    {
        stop();
    }

    bool ODriveSocket::setAxisState(const uint8_t can_id, const uint32_t state)
    {
        const int cmd_id = 0x007;
        const uint32_t arb_id = getArbitrationID(can_id, cmd_id);

        uint8_t data[4];
        std::memcpy(data, &state, sizeof(state));
        if (!socket_->send(arb_id, data, 4))
        {
            std::cerr << "Could not send axis state." << std::endl;
            return false;
        }
        return true;
    }

    bool ODriveSocket::setControlMode(const uint8_t can_id, const uint32_t control_mode, const uint32_t input_mode)
    {
        const int cmd_id = 0x00b;
        const uint32_t arb_id = getArbitrationID(can_id, cmd_id);

        uint8_t data[8];
        std::memcpy(data, &control_mode, sizeof(control_mode));
        std::memcpy(data + 4, &input_mode, sizeof(input_mode));
        if (!socket_->send(arb_id, data, 8))
        {
            std::cerr << "Could not send control mode." << std::endl;
            return false;
        }
        return true;
    }

    bool ODriveSocket::setLimits(const uint8_t can_id, const float velocity_limit, const float current_limit)
    {
        const int cmd_id = 0x00f;
        const uint32_t arb_id = getArbitrationID(can_id, cmd_id);

        uint8_t data[8];
        std::memcpy(data, &velocity_limit, sizeof(velocity_limit));
        std::memcpy(data + 4, &current_limit, sizeof(current_limit));
        if (!socket_->send(arb_id, data, 8))
        {
            std::cerr << "Could not send limits." << std::endl;
            return false;
        }
        return true;
    }

    bool ODriveSocket::setPosGain(const uint8_t can_id, const float pos_gain)
    {
        const int cmd_id = 0x01a;
        const uint32_t arb_id = getArbitrationID(can_id, cmd_id);

        uint8_t data[4];
        std::memcpy(data, &pos_gain, sizeof(pos_gain));
        if (!socket_->send(arb_id, data, 4))
        {
            std::cerr << "Could not send position gain." << std::endl;
            return false;
        }
        return true;
    }

    bool ODriveSocket::setVelGains(const uint8_t can_id, const float vel_gain, const float vel_integrator_gain)
    {
        const int cmd_id = 0x01b;
        const uint32_t arb_id = getArbitrationID(can_id, cmd_id);

        uint8_t data[8];
        std::memcpy(data, &vel_gain, sizeof(vel_gain));
        std::memcpy(data + 4, &vel_integrator_gain, sizeof(vel_integrator_gain));
        if (!socket_->send(arb_id, data, 8))
        {
            std::cerr << "Could not send velocity gains." << std::endl;
            return false;
        }
        return true;
    }

    bool ODriveSocket::setPosition(const uint8_t can_id, const float pos, const float vel_ff, const float torque_ff)
    {
        const int cmd_id = 0x00c;
        const uint32_t arb_id = getArbitrationID(can_id, cmd_id);

        const int16_t vel_ff_int = (int16_t)(vel_ff * 1E3F);
        const int16_t torque_ff_int = (int16_t)(torque_ff * 1E3F);

        uint8_t data[8];
        std::memcpy(data, &pos, sizeof(pos));
        std::memcpy(data + 4, &vel_ff, sizeof(vel_ff_int));
        std::memcpy(data + 6, &torque_ff, sizeof(torque_ff_int));
        if (!socket_->send(arb_id, data, 8))
        {
            std::cerr << "Could not send position." << std::endl;
            return false;
        }
        return true;
    }

    bool ODriveSocket::setVelocity(const uint8_t can_id, const float vel, const float torque_ff)
    {
        const int cmd_id = 0x00d;
        const uint32_t arb_id = getArbitrationID(can_id, cmd_id);

        uint8_t data[8];
        std::memcpy(data, &vel, sizeof(vel));
        std::memcpy(data + 4, &torque_ff, sizeof(torque_ff));
        if (!socket_->send(arb_id, data, 8))
        {
            std::cerr << "Could not send velocity." << std::endl;
            return false;
        }
        return true;
    }

    bool ODriveSocket::setTorque(const uint8_t can_id, const float torque)
    {
        const int cmd_id = 0x00e;
        const uint32_t arb_id = getArbitrationID(can_id, cmd_id);

        uint8_t data[4];
        std::memcpy(data, &torque, sizeof(torque));
        if (!socket_->send(arb_id, data, 4))
        {
            std::cerr << "Could not send torque." << std::endl;
            return false;
        }
        return true;
    }

    bool ODriveSocket::clearErrors(const uint8_t can_id)
    {
        const int cmd_id = 0x018;
        const uint32_t arb_id = getArbitrationID(can_id, cmd_id);

        uint8_t data[4];
        if (!socket_->send(arb_id, data, 4))
        {
            std::cerr << "Could not clear errors." << std::endl;
            return false;
        }
        return true;
    }

    uint32_t ODriveSocket::getAxisError(const uint8_t can_id)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        return info_[can_id].axis_error;
    }

    uint8_t ODriveSocket::getAxisState(const uint8_t can_id)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        return info_[can_id].axis_state;
    }

    float ODriveSocket::getIqSetpoint(const uint8_t can_id)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        return info_[can_id].iq_setpoint;
    }

    float ODriveSocket::getIqMeasured(const uint8_t can_id)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        return info_[can_id].iq_measured;
    }

    float ODriveSocket::getFETTemperature(const uint8_t can_id)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        return info_[can_id].fet_temperature;
    }

    float ODriveSocket::getMotorTemperature(const uint8_t can_id)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        return info_[can_id].motor_temperature;
    }

    float ODriveSocket::getBusVoltage(const uint8_t can_id)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        return info_[can_id].bus_voltage;
    }

    float ODriveSocket::getBusCurrent(const uint8_t can_id)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        return info_[can_id].bus_current;
    }

    float ODriveSocket::getPosEstimate(const uint8_t can_id)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        return info_[can_id].pos_estimate;
    }

    float ODriveSocket::getVelEstimate(const uint8_t can_id)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        return info_[can_id].vel_estimate;
    }

    float ODriveSocket::getTorqueEstimate(const uint8_t can_id)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        return info_[can_id].torque_estimate;
    }

    void ODriveSocket::run()
    {
        while (isRunning())
        {

            struct can_frame frame;
            if (socket_->receive(frame) < 0)
            {
                std::cerr << "Error listening on ODrive socket." << std::endl;
                continue;
            }

            uint8_t can_id = getCanID(frame.can_id);
            uint8_t cmd_id = getCommandID(frame.can_id);

            if (can_id > MAX_CAN_ID)
            {
                std::cerr << "Invalid CAN ID." << std::endl;
                continue;
            }

            switch (cmd_id)
            {
            case 0x001:
                // Heartbeat
                uint32_t axis_error;
                uint8_t axis_state;
                std::memcpy(&axis_error, frame.data, sizeof(axis_error));
                std::memcpy(&axis_state, frame.data + 4, sizeof(axis_state));
                {
                    std::lock_guard<std::mutex> lock(mutex_);
                    info_[can_id].axis_error = axis_error;
                    info_[can_id].axis_state = axis_state;
                }
                break;
            case 0x014:
                // Get_Iq
                float iq_setpoint, iq_measured;
                std::memcpy(&iq_setpoint, frame.data, sizeof(iq_setpoint));
                std::memcpy(&iq_measured, frame.data + 4, sizeof(iq_measured));
                {
                    std::lock_guard<std::mutex> lock(mutex_);
                    info_[can_id].iq_setpoint = iq_setpoint;
                    info_[can_id].iq_measured = iq_measured;
                }
                break;
            case 0x015:
                // Get_Temperature
                float fet_temperature, motor_temperature;
                std::memcpy(&fet_temperature, frame.data, sizeof(fet_temperature));
                std::memcpy(&motor_temperature, frame.data + 4, sizeof(motor_temperature));
                {
                    std::lock_guard<std::mutex> lock(mutex_);
                    info_[can_id].fet_temperature = fet_temperature;
                    info_[can_id].motor_temperature = motor_temperature;
                }
                break;
            case 0x017:
                // Get_Bus_Voltage_Current
                float bus_voltage, bus_current;
                std::memcpy(&bus_voltage, frame.data, sizeof(bus_voltage));
                std::memcpy(&bus_current, frame.data + 4, sizeof(bus_current));
                {
                    std::lock_guard<std::mutex> lock(mutex_);
                    info_[can_id].bus_voltage = bus_voltage;
                    info_[can_id].bus_current = bus_current;
                }
                break;
            case 0x009:
                // Get_Encoder_Estimates
                float pos_estimate, vel_estimate;
                std::memcpy(&pos_estimate, frame.data, sizeof(pos_estimate));
                std::memcpy(&vel_estimate, frame.data + 4, sizeof(vel_estimate));
                {
                    std::lock_guard<std::mutex> lock(mutex_);
                    info_[can_id].pos_estimate = pos_estimate;
                    info_[can_id].vel_estimate = vel_estimate;
                }
                break;
            case 0x01C:
                // Get_Torques
                float torque_estimate;
                std::memcpy(&torque_estimate, frame.data, sizeof(torque_estimate));
                {
                    std::lock_guard<std::mutex> lock(mutex_);
                    info_[can_id].torque_estimate = torque_estimate;
                }
                break;
            }
        }
    }

}