#include <stdio.h>
#include <future>
#include <thread>

#include "starq/mujoco/mujoco_controller.hpp"
#include "starq/dynamics/unitree_rrr.hpp"
#include "starq/leg_controller.hpp"
#include "starq/leg_command_publisher.hpp"

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
    printf("Dynamics created\n");

    LegController::Ptr leg_FR = std::make_shared<LegController>(unitree_RRR_R,
                                                                MotorList{motor_FRA, motor_FRB, motor_FRC});
    LegController::Ptr leg_FL = std::make_shared<LegController>(unitree_RRR_L,
                                                                MotorList{motor_FLA, motor_FLB, motor_FLC});
    LegController::Ptr leg_RR = std::make_shared<LegController>(unitree_RRR_R,
                                                                MotorList{motor_RRA, motor_RRB, motor_RRC});
    LegController::Ptr leg_RL = std::make_shared<LegController>(unitree_RRR_L,
                                                                MotorList{motor_RLA, motor_RLB, motor_RLC});
    printf("Legs created\n");

    LegCommandPublisher::Ptr leg_command_publisher = std::make_shared<LegCommandPublisher>(
        LegList{leg_FL, leg_RL, leg_RR, leg_FR});
    printf("Leg command publisher created\n");

    std::future<void> sim = std::async(std::launch::async, [&]
                                       { mujoco->open("/home/nvidia/starq_ws/src/starq/models/unitree_a1/scene.xml"); });

    while (!mujoco->isOpen())
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    printf("Simulation started\n");

    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    leg_FR->setControlMode(ControlMode::POSITION);
    leg_FL->setControlMode(ControlMode::POSITION);
    leg_RR->setControlMode(ControlMode::POSITION);
    leg_RL->setControlMode(ControlMode::POSITION);

    leg_FR->setFootPosition(Vector3(0, -UNITREE_A1_LENGTH_D, -0.2));
    leg_FL->setFootPosition(Vector3(0, UNITREE_A1_LENGTH_D, -0.2));
    leg_RR->setFootPosition(Vector3(0, -UNITREE_A1_LENGTH_D, -0.2));
    leg_RL->setFootPosition(Vector3(0, UNITREE_A1_LENGTH_D, -0.2));

    std::this_thread::sleep_for(std::chrono::seconds(3));

    LegCommand leg_command;
    leg_command.delay = std::chrono::milliseconds(0);
    leg_command.control_mode = ControlMode::TORQUE;
    leg_command.target_force = Vector3(0, 0, -100);

    for (uint32_t id = 0; id < 4; id++)
    {
        leg_command.leg_id = id;
        leg_command_publisher->sendCommand(std::make_shared<LegCommand>(leg_command));
    }

    while (mujoco->isOpen())
    {
        Vector3 foot_position;
        leg_FR->getFootPositionEstimate(foot_position);
        Vector3 foot_force;
        leg_FR->getFootForceEstimate(foot_force);
        printf("Foot position: %f, %f, %f\n", foot_position(0), foot_position(1), foot_position(2));
        printf("Foot force: %f, %f, %f\n", foot_force(0), foot_force(1), foot_force(2));

        Vector3 joint_angles = leg_FL->getCurrentJointAngles();
        Vector3 joint_torques = leg_FL->getCurrentJointTorques();
        printf("Joint angles: %f, %f, %f\n", joint_angles(0), joint_angles(1), joint_angles(2));
        printf("Joint torques: %f, %f, %f\n", joint_torques(0), joint_torques(1), joint_torques(2));

        printf("\n");

        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    }

    sim.wait();
    printf("Done\n");

    return 0;
}