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

    LegController::Ptr leg_FRA = std::make_shared<LegController>(unitree_RRR_R,
                                                                 MotorList{motor_FRA, motor_FRB, motor_FRC});
    LegController::Ptr leg_FLA = std::make_shared<LegController>(unitree_RRR_L,
                                                                 MotorList{motor_FLA, motor_FLB, motor_FLC});
    LegController::Ptr leg_RRA = std::make_shared<LegController>(unitree_RRR_R,
                                                                 MotorList{motor_RRA, motor_RRB, motor_RRC});
    LegController::Ptr leg_RLA = std::make_shared<LegController>(unitree_RRR_L,
                                                                 MotorList{motor_RLA, motor_RLB, motor_RLC});
    printf("Legs created\n");

    LegCommandPublisher::Ptr leg_command_publisher = std::make_shared<LegCommandPublisher>(
        LegList{leg_FRA, leg_FLA, leg_RRA, leg_RLA});
    printf("Leg command publisher created\n");

    std::future<void> sim = std::async(std::launch::async, [&]
                                       { mujoco->open("/home/nvidia/starq_ws/src/starq/models/unitree_a1/scene.xml"); });
    printf("Simulation started\n");

    leg_FRA->setControlMode(ControlMode::POSITION);
    leg_FLA->setControlMode(ControlMode::POSITION);
    leg_RRA->setControlMode(ControlMode::POSITION);
    leg_RLA->setControlMode(ControlMode::POSITION);

    leg_FRA->setFootPosition(Eigen::Vector3f(0, UNITREE_A1_LENGTH_D, -0.2));
    leg_FLA->setFootPosition(Eigen::Vector3f(0, UNITREE_A1_LENGTH_D, -0.2));
    leg_RRA->setFootPosition(Eigen::Vector3f(0, UNITREE_A1_LENGTH_D, -0.2));
    leg_RLA->setFootPosition(Eigen::Vector3f(0, UNITREE_A1_LENGTH_D, -0.2));

    std::this_thread::sleep_for(std::chrono::seconds(3));

    LegCommand leg_command;
    leg_command.delay = std::chrono::microseconds(0);
    leg_command.control_mode = ControlMode::TORQUE;
    leg_command.target_force = Eigen::Vector3f(0, 0, -100);

    for (uint32_t id = 0; id < 4; id++)
    {
        leg_command.leg_id = id;
        leg_command_publisher->sendCommand(std::make_shared<LegCommand>(leg_command));
    }

    sim.wait();
    printf("Done\n");

    return 0;
}