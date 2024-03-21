#include <stdio.h>

#include "starq/unitree/unitree_a1_mujoco_robot.hpp"
#include "starq/osqp/osqp.hpp"
#include "starq/mpc/mpc_controller.hpp"

using namespace starq;
using namespace starq::mpc;
using namespace starq::osqp;
using namespace starq::unitree;

void printSolution(OSQP::Ptr osqp, MPCConfiguration::Ptr mpc_config);

int main(void)
{

    auto robot = std::make_shared<UnitreeA1MuJoCoRobot>();
    printf("UnitreeA1MuJoCoRobot created\n");

    Gait::Ptr walk_gait = std::make_shared<Gait>();
    walk_gait->load("/home/nvidia/starq_ws/src/starq/gaits/walk.txt");
    //walk_gait->setControlMode(GaitControlMode::GAIT_POSITION_CONTROL);
    //walk_gait->setPose(Vector3(0, 0, UNITREE_A1_STAND_HEIGHT), Vector3(0, 0, M_PI));
    walk_gait->setVelocity(Vector3(-1.0, 0.0, 0), Vector3(0, 0, 0));
    walk_gait->setFrequency(3.0);
    printf("Walk Gait loaded\n");

    Gait::Ptr stand_gait = std::make_shared<Gait>();
    stand_gait->load("/home/nvidia/starq_ws/src/starq/gaits/stand.txt");
    stand_gait->setPose(Vector3(0, 0, UNITREE_A1_STAND_HEIGHT), Vector3(0, 0, M_PI));
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

    while (robot->isSimulationOpen())
    {

        printSolution(osqp, mpc_config);

        const milliseconds time = robot->getLocalization()->getCurrentTime();
        const Vector3 position = robot->getLocalization()->getCurrentPosition();
        const Vector3 orientation = robot->getLocalization()->getCurrentOrientation();
        const Vector3 linear_velocity = robot->getLocalization()->getCurrentLinearVelocity();
        const Vector3 angular_velocity = robot->getLocalization()->getCurrentAngularVelocity();

        printf("----------------------------------------\n");
        printf("Time: %d\n", int(time.count()));
        printf("Position: %f %f %f\n", position.x(), position.y(), position.z());
        printf("Orientation: %f %f %f\n", orientation.x(), orientation.y(), orientation.z());
        printf("Linear velocity: %f %f %f\n", linear_velocity.x(), linear_velocity.y(), linear_velocity.z());
        printf("Angular velocity: %f %f %f\n", angular_velocity.x(), angular_velocity.y(), angular_velocity.z());
        printf("\n\n");

        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }

    robot->waitForSimulation();
    printf("Simulation closed\n");

    printf("Done\n");
    return 0;
}

void printSolution(OSQP::Ptr osqp, MPCConfiguration::Ptr mpc_config)
{
    auto solution = osqp->getSolution();

    for (size_t i = 0; i < mpc_config->getWindowSize(); i++)
    {
        printf("------\n");
        printf("Node[%lu]\n", i);

        const Vector3 position = solution->x_star[i].position;
        const Vector3 orientation = solution->x_star[i].orientation;
        const Vector3 linear_velocity = solution->x_star[i].linear_velocity;
        const Vector3 angular_velocity = solution->x_star[i].angular_velocity;

        printf("Position: %f %f %f\n", position.x(), position.y(), position.z());
        printf("Orientation: %f %f %f\n", orientation.x(), orientation.y(), orientation.z());
        printf("Linear velocity: %f %f %f\n", linear_velocity.x(), linear_velocity.y(), linear_velocity.z());
        printf("Angular velocity: %f %f %f\n", angular_velocity.x(), angular_velocity.y(), angular_velocity.z());

        if (i < mpc_config->getWindowSize() - 1)
        {
            const FootForceState force_state = solution->u_star[i];
            for (size_t j = 0; j < force_state.size(); j++)
            {
                if (force_state[j].first)
                {
                    Vector3 foot_force = force_state[j].second;
                    printf("Force[%lu]: %f %f %f\n", j, foot_force.x(), foot_force.y(), foot_force.z());
                }
            }
        }
    }
}