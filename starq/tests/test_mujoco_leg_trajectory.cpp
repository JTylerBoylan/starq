#include <stdio.h>
#include <future>

#include "starq/mujoco/mujoco_controller.hpp"
#include "starq/unitree/unitree_a1_leg_dynamics.hpp"
#include "starq/leg_controller.hpp"

#include "starq/mujoco/mujoco_localization.hpp"
#include "starq/trajectory_controller.hpp"

using namespace starq;
using namespace starq::mujoco;
using namespace starq::unitree;

// Unitree A1 leg link lengths in meters
#define UNITREE_A1_LENGTH_D 0.08505
#define UNITREE_A1_LENGTH_LT 0.2
#define UNITREE_A1_LENGTH_LC 0.2

int main()
{

    // Get MuJoCo singleton instance
    MuJoCo::Ptr mujoco = MuJoCo::getInstance();
    printf("MuJoCo created\n");

    // Create 12 motor controllers for the Unitree A1 robot
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

    // Create Unitree A1 leg dynamics for left and right legs
    UnitreeA1LegDynamics::Ptr unitree_RRR_L = std::make_shared<UnitreeA1LegDynamics>(UNITREE_A1_LENGTH_D,
                                                                                     UNITREE_A1_LENGTH_LT,
                                                                                     UNITREE_A1_LENGTH_LC);
    UnitreeA1LegDynamics::Ptr unitree_RRR_R = std::make_shared<UnitreeA1LegDynamics>(UNITREE_A1_LENGTH_D,
                                                                                     UNITREE_A1_LENGTH_LT,
                                                                                     UNITREE_A1_LENGTH_LC);
    
    // Flip the Y axis for the right leg
    unitree_RRR_R->flipYAxis();
    printf("Dynamics created\n");

    // Create 4 leg controllers for the Unitree A1 robot with the 12 motor controllers
    LegController::Ptr leg_FR = std::make_shared<LegController>(unitree_RRR_R,
                                                                MotorList{motor_FRA, motor_FRB, motor_FRC});
    LegController::Ptr leg_FL = std::make_shared<LegController>(unitree_RRR_L,
                                                                MotorList{motor_FLA, motor_FLB, motor_FLC});
    LegController::Ptr leg_RR = std::make_shared<LegController>(unitree_RRR_R,
                                                                MotorList{motor_RRA, motor_RRB, motor_RRC});
    LegController::Ptr leg_RL = std::make_shared<LegController>(unitree_RRR_L,
                                                                MotorList{motor_RLA, motor_RLB, motor_RLC});
    printf("Legs created\n");

    // Set leg control modes to position
    leg_FR->setControlMode(ControlMode::POSITION);
    leg_FL->setControlMode(ControlMode::POSITION);
    leg_RR->setControlMode(ControlMode::POSITION);
    leg_RL->setControlMode(ControlMode::POSITION);
    printf("Control mode set\n");

    // Create localization object
    MuJoCoLocalization::Ptr localization = std::make_shared<MuJoCoLocalization>(mujoco);

    // Create leg command publisher to send commands to the legs at a fixed rate
    // The localization is used for timing (sim vs. real). If no localization is provided, the system/real time is used.
    LegCommandPublisher::Ptr leg_command_publisher = std::make_shared<LegCommandPublisher>(
        LegList{leg_FL, leg_RL, leg_RR, leg_FR},
        localization);
    printf("Leg command publisher created\n");

    // Create trajectory publisher to send trajectory commands to the legs
    TrajectoryController::Ptr trajectory_controller = std::make_shared<TrajectoryController>(leg_command_publisher);
    printf("Trajectory publisher created\n");

    // Launch simulation in a separate thread
    std::future<void> sim = std::async(std::launch::async, [&]
                                       { mujoco->open("/home/nvidia/starq_ws/src/starq/models/unitree_a1/scene.xml"); });

    // Wait for simulation to start
    while (!mujoco->isOpen())
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    printf("Simulation started\n");
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    // Go to base positions
    leg_FR->setFootPosition(Vector3(0, -UNITREE_A1_LENGTH_D, -0.2));
    leg_FL->setFootPosition(Vector3(0, UNITREE_A1_LENGTH_D, -0.2));
    leg_RR->setFootPosition(Vector3(0, -UNITREE_A1_LENGTH_D, -0.2));
    leg_RL->setFootPosition(Vector3(0, UNITREE_A1_LENGTH_D, -0.2));

    // Wait for 3 seconds to let the simulation settle
    std::this_thread::sleep_for(std::chrono::seconds(3));

    while (mujoco->isOpen())
    {
        // Repeatedly run trajectory from file
        trajectory_controller->setTrajectory("/home/nvidia/starq_ws/src/starq/trajectories/walking.txt");
        trajectory_controller->start();

        // Wait for trajectory to finish before running it again
        while (trajectory_controller->isRunning())
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
    }

    // Wait for simulation to finish
    sim.wait();

    printf("Done\n");
    return 0;
}