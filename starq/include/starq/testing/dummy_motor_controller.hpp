#ifndef STARQ_TESTING__DUMMY_MOTOR_CONTROLLER_HPP_
#define STARQ_TESTING__DUMMY_MOTOR_CONTROLLER_HPP_

#include "starq/motor_controller.hpp"

#include <iostream>

namespace starq::testing
{

    /// @brief A dummy motor controller that does nothing. For testing.
    class DummyMotorController : public starq::MotorController
    {
    public:
        DummyMotorController(const uint8_t motor_id)
        : motor_id_(motor_id)
        {}

        ~DummyMotorController() {}

        bool setGearRatio(const float gear_ratio) override
        {
            std::cout << "Setting gear ratio for motor " << (int)motor_id_ << " to " << gear_ratio << std::endl;
            return true;
        }

        bool setState(const uint32_t state) override
        {
            std::cout << "Setting state for motor " << (int)motor_id_ << " to " << state << std::endl;
            return true;
        }

        bool setControlMode(const uint32_t control_mode, const uint32_t input_mode = 0x1) override
        {
            std::cout << "Setting control mode for motor " << (int)motor_id_ << " to " << control_mode << " with input mode " << input_mode << std::endl;
            return true;
        }

        bool setPosition(const float pos, const float vel_ff = 0.F, const float torque_ff = 0.F) override
        {
            std::cout << "Setting position for motor " << (int)motor_id_ << " to " << pos
                      << " with velocity feedforward " << vel_ff
                      << " and torque feedforward " << torque_ff << std::endl;

            last_pos_ = pos;
            last_vel_ = vel_ff;
            last_torque_ = torque_ff;
            return true;
        }

        bool setVelocity(const float vel, const float torque_ff = 0.F) override
        {
            std::cout << "Setting velocity for motor " << (int)motor_id_ << " to " << vel
                      << " with torque feedforward " << torque_ff << std::endl;

            last_vel_ = vel;
            last_torque_ = torque_ff;
            return true;
        }

        bool setTorque(const float torque) override
        {
            std::cout << "Setting torque for motor " << (int)motor_id_ << " to " << torque << std::endl;

            last_torque_ = torque;
            return true;
        }

        float getPositionEstimate() override
        {
            return last_pos_;
        }

        float getVelocityEstimate() override
        {
            return last_vel_;
        }

        float getTorqueEstimate() override
        {
            return last_torque_;
        }

    private:
        uint8_t motor_id_;
        float last_pos_;
        float last_vel_;
        float last_torque_;
    };

}

#endif