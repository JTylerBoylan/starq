#ifndef STARQ__STARQ_ROBOT_HPP_
#define STARQ__STARQ_ROBOT_HPP_

#include "starq/robot.hpp"
#include "starq/odrive/odrive_controller.hpp"
#include "starq/starq/starq_fivebar2d_leg_dynamics.hpp"

#define STARQ_LINK_LENGTH_1 0.065
#define STARQ_LINK_LENGTH_2 0.2
#define STARQ_GEAR_RATIO 6.0

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

        /// @brief Get the leg dynamics of the front left leg
        /// @return The leg dynamics of the front left leg
        STARQFiveBar2DLegDynamics::Ptr getLegDynamicsFL() const { return leg_dynamics_FL_; }

        /// @brief Get the leg dynamics of the rear left leg
        /// @return The leg dynamics of the rear left leg
        STARQFiveBar2DLegDynamics::Ptr getLegDynamicsRL() const { return leg_dynamics_RL_; }

        /// @brief Get the leg dynamics of the rear right leg
        /// @return The leg dynamics of the rear right leg
        STARQFiveBar2DLegDynamics::Ptr getLegDynamicsRR() const { return leg_dynamics_RR_; }

        /// @brief Get the leg dynamics of the front right leg
        /// @return The leg dynamics of the front right leg
        STARQFiveBar2DLegDynamics::Ptr getLegDynamicsFR() const { return leg_dynamics_FR_; }

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

        STARQFiveBar2DLegDynamics::Ptr leg_dynamics_FL_;
        STARQFiveBar2DLegDynamics::Ptr leg_dynamics_RL_;
        STARQFiveBar2DLegDynamics::Ptr leg_dynamics_RR_;
        STARQFiveBar2DLegDynamics::Ptr leg_dynamics_FR_;
    };
}

#endif