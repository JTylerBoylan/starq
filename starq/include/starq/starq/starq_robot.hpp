#ifndef STARQ__STARQ_ROBOT_HPP_
#define STARQ__STARQ_ROBOT_HPP_

#include "starq/robot.hpp"
#include "starq/odrive/odrive_controller.hpp"
#include "starq/starq/starq_fivebar2d_leg_dynamics.hpp"

namespace starq
{

    class STARQRobot : public Robot
    {
    public:
        using Ptr = std::shared_ptr<STARQRobot>;

        /// @brief Create a STARQ robot
        STARQRobot();

        /// @brief Set the state of the motors
        /// @param state The state
        /// @return True if successful
        bool setState(AxisState state);

        /// @brief Get the first CAN socket
        /// @return The CAN socket
        can::CANSocket::Ptr getCanSocket0() const { return can_socket_0_; }

        /// @brief Get the second CAN socket
        /// @return The CAN socket
        can::CANSocket::Ptr getCanSocket1() const { return can_socket_1_; }

        /// @brief Get the leg dynamics
        /// @return The leg dynamics
        STARQFiveBar2DLegDynamics::Ptr getLegDynamics() const { return leg_dynamics_; }

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

        STARQFiveBar2DLegDynamics::Ptr leg_dynamics_;
    };
}

#endif