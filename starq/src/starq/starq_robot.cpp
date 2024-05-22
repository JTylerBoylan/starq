#include "starq/starq/starq_robot.hpp"

#include <iostream>

namespace starq
{

    STARQRobot::STARQRobot()
    {
        setup();
    }

    bool STARQRobot::setGains(const Float p_gain, const Float v_gain, const Float vi_gain)
    {
        for (size_t i = 0; i < motors_.size(); i++)
        {
            auto odrive = std::dynamic_pointer_cast<odrive::ODriveController>(motors_[i]);

            if (!odrive->setPosGain(p_gain) || !odrive->setVelGains(v_gain, vi_gain))
            {
                std::cerr << "Failed to set gains for motor " << i << "." << std::endl;
                return false;
            }
        }
        return true;
    }

    bool STARQRobot::setLimits(const Float velocity_limit, const Float current_limit)
    {
        for (size_t i = 0; i < motors_.size(); i++)
        {
            auto odrive = std::dynamic_pointer_cast<odrive::ODriveController>(motors_[i]);

            if (!odrive->setLimits(velocity_limit, current_limit))
            {
                std::cerr << "Failed to set limits for motor " << i << "." << std::endl;
                return false;
            }
        }
        return true;
    }

    void STARQRobot::setupMotorControllers()
    {

        using namespace can;
        can_socket_0_ = std::make_shared<CANSocket>("can0");
        can_socket_1_ = std::make_shared<CANSocket>("can1");

        if (!can_socket_0_->connect())
        {
            std::cerr << "Failed to connect to CAN interface 0." << std::endl;
        }

        if (!can_socket_1_->connect())
        {
            std::cerr << "Failed to connect to CAN interface 1." << std::endl;
        }

        using namespace odrive;
        ODriveSocket::Ptr odrive_socket_0 = std::make_shared<ODriveSocket>(can_socket_0_);
        ODriveSocket::Ptr odrive_socket_1 = std::make_shared<ODriveSocket>(can_socket_1_);

        ODriveController::Ptr odrive_0 = std::make_shared<ODriveController>(odrive_socket_0, 0);
        ODriveController::Ptr odrive_1 = std::make_shared<ODriveController>(odrive_socket_0, 1);
        ODriveController::Ptr odrive_2 = std::make_shared<ODriveController>(odrive_socket_0, 2);
        ODriveController::Ptr odrive_3 = std::make_shared<ODriveController>(odrive_socket_0, 3);
        ODriveController::Ptr odrive_4 = std::make_shared<ODriveController>(odrive_socket_1, 4);
        ODriveController::Ptr odrive_5 = std::make_shared<ODriveController>(odrive_socket_1, 5);
        ODriveController::Ptr odrive_6 = std::make_shared<ODriveController>(odrive_socket_1, 6);
        ODriveController::Ptr odrive_7 = std::make_shared<ODriveController>(odrive_socket_1, 7);

        motors_ = {
            odrive_0,
            odrive_1,
            odrive_2,
            odrive_3,
            odrive_4,
            odrive_5,
            odrive_6,
            odrive_7};

        setStates(AxisState::IDLE);
        setGearRatios(STARQ_MOTOR_GEAR_RATIO);
        setGains(STARQ_MOTOR_P_GAIN, STARQ_MOTOR_V_GAIN, STARQ_MOTOR_VI_GAIN);
        setLimits(STARQ_MOTOR_MAX_VELOCITY, STARQ_MOTOR_MAX_CURRENT);
    }

    void STARQRobot::setupLegControllers()
    {
        auto leg_dynamics_L = std::make_shared<STARQFiveBar2DLegDynamics>(STARQ_LINK_LENGTH_1, STARQ_LINK_LENGTH_2);
        auto leg_dynamics_R = std::make_shared<STARQFiveBar2DLegDynamics>(STARQ_LINK_LENGTH_1, STARQ_LINK_LENGTH_2);

        leg_dynamics_L->flipY();

        LegController::Ptr leg_FL = std::make_shared<LegController>(leg_dynamics_L,
                                                                    MotorList{motors_[0], motors_[1]});

        LegController::Ptr leg_RL = std::make_shared<LegController>(leg_dynamics_L,
                                                                    MotorList{motors_[2], motors_[3]});

        LegController::Ptr leg_RR = std::make_shared<LegController>(leg_dynamics_R,
                                                                    MotorList{motors_[4], motors_[5]});

        LegController::Ptr leg_FR = std::make_shared<LegController>(leg_dynamics_R,
                                                                    MotorList{motors_[6], motors_[7]});

        legs_ = {leg_FL, leg_RL, leg_RR, leg_FR};
    }

    void STARQRobot::setupLocalization()
    {
        localization_ = std::make_shared<SystemLocalization>();
    }

    void STARQRobot::setupRobotParameters()
    {
        robot_parameters_ = std::make_shared<STARQRobotParameters>();
    }

    void STARQRobot::setupMPCSolver()
    {
        mpc_solver_ = std::make_shared<osqp::OSQP>();
    }

}