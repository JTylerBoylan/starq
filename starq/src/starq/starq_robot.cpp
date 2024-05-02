#include "starq/starq/starq_robot.hpp"

#include <iostream>

namespace starq
{

    STARQRobot::STARQRobot()
    {
        setup();
    }

    bool STARQRobot::setState(AxisState state)
    {
        for (size_t i = 0; i < motors_.size(); i++)
        {
            if (!motors_[i]->setState(state))
            {
                std::cout << "Failed to set axis state for motor " << i << "." << std::endl;
                return false;
            }
        }
        return true;
    }

    void STARQRobot::setupMotors()
    {

        using namespace can;
        can_socket_0_ = std::make_shared<CANSocket>("can0");
        can_socket_1_ = std::make_shared<CANSocket>("can1");

        if (!can_socket_0_->connect())
        {
            std::cout << "Failed to connect to CAN interface 0." << std::endl;
        }

        if (!can_socket_1_->connect())
        {
            std::cout << "Failed to connect to CAN interface 1." << std::endl;
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

        odrive_0->setGearRatio(STARQ_GEAR_RATIO);
        odrive_1->setGearRatio(STARQ_GEAR_RATIO);
        odrive_2->setGearRatio(STARQ_GEAR_RATIO);
        odrive_3->setGearRatio(STARQ_GEAR_RATIO);
        odrive_4->setGearRatio(STARQ_GEAR_RATIO);
        odrive_5->setGearRatio(STARQ_GEAR_RATIO);
        odrive_6->setGearRatio(STARQ_GEAR_RATIO);
        odrive_7->setGearRatio(STARQ_GEAR_RATIO);

        motors_ = {
            odrive_0,
            odrive_1,
            odrive_2,
            odrive_3,
            odrive_4,
            odrive_5,
            odrive_6,
            odrive_7};
    }

    void STARQRobot::setupLegs()
    {
        leg_dynamics_FL_ = std::make_shared<STARQFiveBar2DLegDynamics>(STARQ_LINK_LENGTH_1, STARQ_LINK_LENGTH_2);
        leg_dynamics_RL_ = std::make_shared<STARQFiveBar2DLegDynamics>(STARQ_LINK_LENGTH_1, STARQ_LINK_LENGTH_2);
        leg_dynamics_RR_ = std::make_shared<STARQFiveBar2DLegDynamics>(STARQ_LINK_LENGTH_1, STARQ_LINK_LENGTH_2);
        leg_dynamics_FR_ = std::make_shared<STARQFiveBar2DLegDynamics>(STARQ_LINK_LENGTH_1, STARQ_LINK_LENGTH_2);

        leg_dynamics_FL_->flipY();
        leg_dynamics_RL_->flipY();

        LegController::Ptr leg_FL = std::make_shared<LegController>(leg_dynamics_FL_,
                                                                    MotorList{motors_[0], motors_[1]});

        LegController::Ptr leg_RL = std::make_shared<LegController>(leg_dynamics_RL_,
                                                                    MotorList{motors_[2], motors_[3]});

        LegController::Ptr leg_RR = std::make_shared<LegController>(leg_dynamics_RR_,
                                                                    MotorList{motors_[4], motors_[5]});

        LegController::Ptr leg_FR = std::make_shared<LegController>(leg_dynamics_FR_,
                                                                    MotorList{motors_[6], motors_[7]});

        legs_ = {leg_FL, leg_RL, leg_RR, leg_FR};
    }

    void STARQRobot::setupLocalization()
    {
        localization_ = std::make_shared<SystemLocalization>();
    }

    void STARQRobot::setupRobotDynamics()
    {
        robot_parameters_ = nullptr;
    }

    void STARQRobot::setupMPCSolver()
    {
        mpc_solver_ = nullptr;
    }

}