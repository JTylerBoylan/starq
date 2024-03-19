#include "starq/robots/unitree_a1_mujoco.hpp"

#include <iostream>

namespace starq::robots
{

    UnitreeA1MuJoCoRobot::UnitreeA1MuJoCoRobot()
        : MuJoCoRobot()
    {
        scene_file_ = UNITREE_A1_MUJOCO_SCENE_FILE;

        setup();
    }

    void UnitreeA1MuJoCoRobot::setupMotors()
    {
        auto motor_FRA = std::make_shared<MuJoCoController>(mujoco_, 0);
        auto motor_FRB = std::make_shared<MuJoCoController>(mujoco_, 1);
        auto motor_FRC = std::make_shared<MuJoCoController>(mujoco_, 2);
        auto motor_FLA = std::make_shared<MuJoCoController>(mujoco_, 3);
        auto motor_FLB = std::make_shared<MuJoCoController>(mujoco_, 4);
        auto motor_FLC = std::make_shared<MuJoCoController>(mujoco_, 5);
        auto motor_RRA = std::make_shared<MuJoCoController>(mujoco_, 6);
        auto motor_RRB = std::make_shared<MuJoCoController>(mujoco_, 7);
        auto motor_RRC = std::make_shared<MuJoCoController>(mujoco_, 8);
        auto motor_RLA = std::make_shared<MuJoCoController>(mujoco_, 9);
        auto motor_RLB = std::make_shared<MuJoCoController>(mujoco_, 10);
        auto motor_RLC = std::make_shared<MuJoCoController>(mujoco_, 11);

        motors_ = {motor_FRA, motor_FRB, motor_FRC,
                   motor_FLA, motor_FLB, motor_FLC,
                   motor_RRA, motor_RRB, motor_RRC,
                   motor_RLA, motor_RLB, motor_RLC};
    }

    void UnitreeA1MuJoCoRobot::setupLegs()
    {
        unitree_RRR_L_ = std::make_shared<Unitree_RRR>(UNITREE_A1_LENGTH_D,
                                                       UNITREE_A1_LENGTH_LT,
                                                       UNITREE_A1_LENGTH_LC);
        unitree_RRR_R_ = std::make_shared<Unitree_RRR>(UNITREE_A1_LENGTH_D,
                                                       UNITREE_A1_LENGTH_LT,
                                                       UNITREE_A1_LENGTH_LC);
        unitree_RRR_R_->flipYAxis();

        auto leg_FR = std::make_shared<LegController>(unitree_RRR_R_,
                                                      MotorList{motors_[0], motors_[1], motors_[2]});
        auto leg_FL = std::make_shared<LegController>(unitree_RRR_L_,
                                                      MotorList{motors_[3], motors_[4], motors_[5]});
        auto leg_RR = std::make_shared<LegController>(unitree_RRR_R_,
                                                      MotorList{motors_[6], motors_[7], motors_[8]});
        auto leg_RL = std::make_shared<LegController>(unitree_RRR_L_,
                                                      MotorList{motors_[9], motors_[10], motors_[11]});

        legs_ = {leg_FL, leg_RL, leg_RR, leg_FR};
    }

    void UnitreeA1MuJoCoRobot::setupRobotDynamics()
    {
        robot_dynamics_ = std::make_shared<dynamics::UnitreeA1RobotDynamics>();
    }

    void UnitreeA1MuJoCoRobot::setupMPCSolver()
    {
        osqp_ = std::make_shared<osqp::OSQP>();
        mpc_solver_ = osqp_;
        
        osqp_->getSettings()->verbose = false;
        osqp_->getSettings()->max_iter = 2000;
        osqp_->getSettings()->polishing = true;
        osqp_->getSettings()->warm_starting = true;
    }

}