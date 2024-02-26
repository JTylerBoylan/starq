#include "starq/robots/unitree_a1_mujoco.hpp"

namespace starq::robots
{

    UnitreeA1MuJoCoRobot::UnitreeA1MuJoCoRobot()
        : mujoco_(MuJoCo::getInstance()),
          scene_file_("/home/nvidia/starq_ws/src/starq/models/unitree_a1/scene.xml")
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

        unitree_RRR_L_ = std::make_shared<Unitree_RRR>(UNITREE_A1_LENGTH_D,
                                                       UNITREE_A1_LENGTH_LT,
                                                       UNITREE_A1_LENGTH_LC);
        unitree_RRR_R_ = std::make_shared<Unitree_RRR>(UNITREE_A1_LENGTH_D,
                                                       UNITREE_A1_LENGTH_LT,
                                                       UNITREE_A1_LENGTH_LC);
        unitree_RRR_R_->flipYAxis();

        auto leg_FR = std::make_shared<LegController>(unitree_RRR_R_,
                                                      MotorList{motor_FRA, motor_FRB, motor_FRC});
        auto leg_FL = std::make_shared<LegController>(unitree_RRR_L_,
                                                      MotorList{motor_FLA, motor_FLB, motor_FLC});
        auto leg_RR = std::make_shared<LegController>(unitree_RRR_R_,
                                                      MotorList{motor_RRA, motor_RRB, motor_RRC});
        auto leg_RL = std::make_shared<LegController>(unitree_RRR_L_,
                                                      MotorList{motor_RLA, motor_RLB, motor_RLC});

        legs_ = {leg_FR, leg_FL, leg_RR, leg_RL};

        localization_ = std::make_shared<MuJoCoLocalization>(mujoco_);

        publisher_ = std::make_shared<LegCommandPublisher>(legs_);

        trajectory_file_reader_ = std::make_shared<TrajectoryFileReader>();

        trajectory_publisher_ = std::make_shared<TrajectoryPublisher>(publisher_);
    }

    void UnitreeA1MuJoCoRobot::startSimulation()
    {
        simulation_ = std::async(std::launch::async, [this]()
                                 { mujoco_->open(scene_file_); });
    }

    void UnitreeA1MuJoCoRobot::waitForSimulation()
    {
        simulation_.wait();
    }

    bool UnitreeA1MuJoCoRobot::isSimulationOpen()
    {
        return mujoco_->isOpen();
    }

}