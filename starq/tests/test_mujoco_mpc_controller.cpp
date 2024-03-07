#include <stdio.h>

#include "starq/robots/unitree_a1_mujoco.hpp"
#include "starq/osqp/osqp.hpp"
#include "starq/mpc/mpc_controller.hpp"

using namespace starq;
using namespace starq::mpc;
using namespace starq::osqp;

int main(void)
{

    auto robot = std::make_shared<robots::UnitreeA1MuJoCoRobot>();
    printf("UnitreeA1MuJoCoRobot created\n");

    Gait::Ptr walk_gait = std::make_shared<Gait>();
    walk_gait->load("/home/nvidia/starq_ws/src/starq/gaits/walk.txt");
    walk_gait->setControlMode(GaitControlMode::GAIT_POSITION_CONTROL);
    walk_gait->setPose(Vector3f(0, 0, UNITREE_A1_STAND_HEIGHT), Vector3f(0, 0, 0));
    // walk_gait->setVelocity(Vector3f(0, 0, 0), Vector3f(0, 0, 0));
    walk_gait->setFrequency(2.0);
    printf("Walk Gait loaded\n");

    Gait::Ptr stand_gait = std::make_shared<Gait>();
    stand_gait->load("/home/nvidia/starq_ws/src/starq/gaits/stand.txt");
    stand_gait->setPose(Vector3f(0, 0, UNITREE_A1_STAND_HEIGHT), Vector3f(0, 0, 0));
    stand_gait->setFrequency(10.0);
    printf("Stand Gait loaded\n");

    MPCConfiguration::Ptr mpc_config = std::make_shared<MPCConfiguration>(robot->getLegs(),
                                                                          robot->getRobotDynamics(),
                                                                          robot->getLocalization());
    mpc_config->setTimeStep(milliseconds(50));
    mpc_config->setWindowSize(21);
    printf("MPCConfiguration created\n");

    OSQP::Ptr osqp = std::make_shared<OSQP>();
    osqp->getSettings()->verbose = false;
    osqp->getSettings()->max_iter = 2000;
    osqp->getSettings()->polishing = true;
    osqp->getSettings()->warm_starting = true;
    printf("OSQP created\n");

    MPCController::Ptr mpc_controller = std::make_shared<MPCController>(mpc_config, osqp,
                                                                        robot->getLegCommandPublisher());

    robot->startSimulation();
    printf("Simulation started\n");

    for (uint8_t id = 0; id < UNITREE_A1_NUM_LEGS; id++)
    {
        robot->setFootPosition(id, robot->getRobotDynamics()->getDefaultFootLocations()[id]);
    }
    printf("Holding foot position for 5 seconds...\n");
    std::this_thread::sleep_for(std::chrono::seconds(5));

    mpc_controller->start();
    printf("MPCController started\n");

    printf("Standing for 5 seconds...\n");
    mpc_config->setNextGait(stand_gait);
    std::this_thread::sleep_for(std::chrono::seconds(5));

    printf("Walking...\n");
    mpc_config->setNextGait(walk_gait);

    robot->waitForSimulation();
    printf("Simulation closed\n");

    printf("Done\n");
    return 0;
}