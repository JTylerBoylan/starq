#ifndef STARQ__STARQ_ROBOT_HPP_
#define STARQ__STARQ_ROBOT_HPP_

#include "starq/robot.hpp"
#include "starq/odrive/odrive_controller.hpp"
#include "starq/starq/starq_fivebar2d_leg_dynamics.hpp"
#include "starq/starq/starq_robot_parameters.hpp"
#include "starq/osqp/osqp.hpp"

namespace starq
{

    class STARQRobot : public Robot
    {
    public:
        using Ptr = std::shared_ptr<STARQRobot>;

        /// @brief Create a STARQ robot
        STARQRobot();

        /// @brief Set the gains for all motors
        /// @param p_gain Position gain
        /// @param v_gain Velocity gain
        /// @param vi_gain Velocity integral gain
        /// @return True if successful
        bool setGains(const Float p_gain, const Float v_gain, const Float vi_gain);

        /// @brief Set the limits for all motors
        /// @param velocity_limit Velocity limit
        /// @param current_limit Current limit
        /// @return True if successful
        bool setLimits(const Float velocity_limit, const Float current_limit);

        /// @brief Get the first CAN socket
        /// @return The CAN socket
        can::CANSocket::Ptr getCanSocket0() const { return can_socket_0_; }

        /// @brief Get the second CAN socket
        /// @return The CAN socket
        can::CANSocket::Ptr getCanSocket1() const { return can_socket_1_; }

    private:
        /// @brief Setup the motor controllers
        void setupMotors() override;

        /// @brief Setup the leg controllers
        void setupLegs() override;

        /// @brief Setup the localization
        void setupLocalization() override;

        /// @brief Setup the robot dynamics
        void setupRobotDynamics() override;

        /// @brief Setup the model predictive control solver
        void setupMPCSolver() override;

        can::CANSocket::Ptr can_socket_0_;
        can::CANSocket::Ptr can_socket_1_;
    };
}

#endif