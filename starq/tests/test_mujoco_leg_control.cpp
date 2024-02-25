#include <stdio.h>
#include <future>
#include <thread>

#include "starq/mujoco/mujoco_controller.hpp"
#include "starq/dynamics/unitree_rrr.hpp"
#include "starq/leg_controller.hpp"

using namespace starq;
using namespace starq::dynamics;
using namespace starq::mujoco;

#define UNITREE_A1_LENGTH_D 0.08505
#define UNITREE_A1_LENGTH_LT 0.2
#define UNITREE_A1_LENGTH_LC 0.2

int main()
{
    MuJoCo::Ptr mujoco = MuJoCo::getInstance();
    printf("MuJoCo created\n");

    MuJoCoController::Ptr motor_FRA = std::make_shared<MuJoCoController>(mujoco, 0);
    MuJoCoController::Ptr motor_FRB = std::make_shared<MuJoCoController>(mujoco, 1);
    MuJoCoController::Ptr motor_FRC = std::make_shared<MuJoCoController>(mujoco, 2);
    MuJoCoController::Ptr motor_FLA = std::make_shared<MuJoCoController>(mujoco, 3);
    MuJoCoController::Ptr motor_FLB = std::make_shared<MuJoCoController>(mujoco, 4);
    MuJoCoController::Ptr motor_FLC = std::make_shared<MuJoCoController>(mujoco, 5);
    MuJoCoController::Ptr motor_RRA = std::make_shared<MuJoCoController>(mujoco, 6);
    MuJoCoController::Ptr motor_RRB = std::make_shared<MuJoCoController>(mujoco, 7);
    MuJoCoController::Ptr motor_RRC = std::make_shared<MuJoCoController>(mujoco, 8);
    MuJoCoController::Ptr motor_RLA = std::make_shared<MuJoCoController>(mujoco, 9);
    MuJoCoController::Ptr motor_RLB = std::make_shared<MuJoCoController>(mujoco, 10);
    MuJoCoController::Ptr motor_RLC = std::make_shared<MuJoCoController>(mujoco, 11);
    printf("Controllers created\n");

    Unitree_RRR::Ptr unitree_RRR_L = std::make_shared<Unitree_RRR>(UNITREE_A1_LENGTH_D,
                                                                   UNITREE_A1_LENGTH_LT,
                                                                   UNITREE_A1_LENGTH_LC);

    Unitree_RRR::Ptr unitree_RRR_R = std::make_shared<Unitree_RRR>(UNITREE_A1_LENGTH_D,
                                                                   UNITREE_A1_LENGTH_LT,
                                                                   UNITREE_A1_LENGTH_LC);
    unitree_RRR_R->flipYAxis();

    LegController::Ptr leg_FR = std::make_shared<LegController>(unitree_RRR_R,
                                                                 MotorList{motor_FRA, motor_FRB, motor_FRC});
    LegController::Ptr leg_FL = std::make_shared<LegController>(unitree_RRR_L,
                                                                 MotorList{motor_FLA, motor_FLB, motor_FLC});
    LegController::Ptr leg_RR = std::make_shared<LegController>(unitree_RRR_R,
                                                                 MotorList{motor_RRA, motor_RRB, motor_RRC});
    LegController::Ptr leg_RL = std::make_shared<LegController>(unitree_RRR_L,
                                                                 MotorList{motor_RLA, motor_RLB, motor_RLC});

    std::future<void> sim = std::async(std::launch::async, [&]
                                       { mujoco->open("/home/nvidia/starq_ws/src/starq/models/unitree_a1/scene.xml"); });
    printf("Simulation started\n");

    leg_FR->setControlMode(ControlMode::POSITION);
    leg_FL->setControlMode(ControlMode::POSITION);
    leg_RR->setControlMode(ControlMode::POSITION);
    leg_RL->setControlMode(ControlMode::POSITION);

    leg_FR->setFootPosition(Eigen::Vector3f(0, UNITREE_A1_LENGTH_D, -0.2));
    leg_FL->setFootPosition(Eigen::Vector3f(0, UNITREE_A1_LENGTH_D, -0.2));
    leg_RR->setFootPosition(Eigen::Vector3f(0, UNITREE_A1_LENGTH_D, -0.2));
    leg_RL->setFootPosition(Eigen::Vector3f(0, UNITREE_A1_LENGTH_D, -0.2));

    std::this_thread::sleep_for(std::chrono::seconds(10));

    VectorXf foot_position;
    leg_FR->getFootPositionEstimate(foot_position);
    printf("Foot position: %f, %f, %f\n", foot_position(0), foot_position(1), foot_position(2));

    VectorXf foot_force;
    leg_FR->getFootForceEstimate(foot_force);
    printf("Foot force: %f, %f, %f\n", foot_force(0), foot_force(1), foot_force(2));

    sim.wait();
    printf("Done\n");

    return 0;
}