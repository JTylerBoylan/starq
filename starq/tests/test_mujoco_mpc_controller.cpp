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

    // Create Unitree A1 robot
    auto robot = std::make_shared<UnitreeA1MuJoCoRobot>();
    printf("UnitreeA1MuJoCoRobot created\n");

    // Load walk gait from file
    Gait::Ptr walk_gait = std::make_shared<Gait>();
    walk_gait->load("/home/nvidia/starq_ws/src/starq/gaits/walk.txt");
    printf("Walk Gait loaded\n");

    // Set gait to walk forward at constant velocity
    walk_gait->setVelocity(Vector3(0.5, 0.0, 0), Vector3(0, 0, 0));
    walk_gait->setFrequency(3.0);

    // Load stand gait from file
    Gait::Ptr stand_gait = std::make_shared<Gait>();
    stand_gait->load("/home/nvidia/starq_ws/src/starq/gaits/stand.txt");
    printf("Stand Gait loaded\n");

    // Stand still at default height
    stand_gait->setPose(Vector3(0, 0, UNITREE_A1_STAND_HEIGHT), Vector3(0, 0, 0));
    stand_gait->setFrequency(10.0);

    // Create MPC Configuration object based on the robot
    MPCConfiguration::Ptr mpc_config = std::make_shared<MPCConfiguration>(robot->getLegs(),
                                                                          robot->getRobotParameters(),
                                                                          robot->getLocalization());

    // Set MPC parameters
    mpc_config->setTimeStep(milliseconds(50));
    mpc_config->setWindowSize(21);
    printf("MPCConfiguration created\n");

    // Create OSQP solver object
    OSQP::Ptr osqp = std::make_shared<OSQP>();
    printf("OSQP created\n");

    // Set OSQP parameters
    osqp->getSettings()->verbose = false;
    osqp->getSettings()->max_iter = 2000;
    osqp->getSettings()->polishing = true;
    osqp->getSettings()->warm_starting = true;

    // Create MPC Controller object
    // MPC Controller is responsible for running the MPC solver and sending the leg commands to the robot
    MPCController::Ptr mpc_controller = std::make_shared<MPCController>(mpc_config, osqp,
                                                                        robot->getLegCommandPublisher());

    // Start simulation
    robot->startSimulation();
    printf("Simulation started\n");

    // Set foot positions to default locations
    for (uint8_t id = 0; id < UNITREE_A1_NUM_LEGS; id++)
    {
        robot->setFootPosition(id, robot->getRobotParameters()->getDefaultFootLocations()[id]);
    }
    printf("Holding foot position for 5 seconds...\n");

    // Wait for simulation to settle
    std::this_thread::sleep_for(std::chrono::seconds(5));

    // Start MPC Controller in a separate thread
    mpc_controller->start();
    printf("MPCController started\n");

    // Run standing gait for 5 seconds
    printf("Standing for 5 seconds...\n");
    mpc_config->setNextGait(stand_gait);
    std::this_thread::sleep_for(std::chrono::seconds(5));

    // Transition to walking gait
    printf("Walking...\n");
    mpc_config->setNextGait(walk_gait);

    while (robot->isSimulationOpen())
    {

        // Print MPC solution, for debugging purposes
        // printSolution(osqp, mpc_config);

        // Get localization information
        const milliseconds time = robot->getLocalization()->getCurrentTime();
        const Vector3 position = robot->getLocalization()->getCurrentPosition();
        const Vector3 orientation = robot->getLocalization()->getCurrentOrientation();
        const Vector3 linear_velocity = robot->getLocalization()->getCurrentLinearVelocity();
        const Vector3 angular_velocity = robot->getLocalization()->getCurrentAngularVelocity();

        // Print localization information
        printf("----------------------------------------\n");
        printf("Time: %d\n", int(time.count()));
        printf("Position: %f %f %f\n", position.x(), position.y(), position.z());
        printf("Orientation: %f %f %f\n", orientation.x(), orientation.y(), orientation.z());
        printf("Linear velocity: %f %f %f\n", linear_velocity.x(), linear_velocity.y(), linear_velocity.z());
        printf("Angular velocity: %f %f %f\n", angular_velocity.x(), angular_velocity.y(), angular_velocity.z());
        printf("\n\n");

        // Sleep for 50 milliseconds
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }

    // Wait for simulation to close
    robot->waitForSimulation();
    printf("Simulation closed\n");

    printf("Done\n");
    return 0;
}

// Function to print MPC solution at each node
void printSolution(OSQP::Ptr osqp, MPCConfiguration::Ptr mpc_config)
{
    // Get MPC solution
    auto solution = osqp->getSolution();

    // Print MPC solution at each node
    for (size_t i = 0; i < mpc_config->getWindowSize(); i++)
    {
        printf("------\n");
        printf("Node[%lu]\n", i);

        // Print reference state at each node
        const Vector3 position = solution->x_star[i].position;
        printf("Position: %f %f %f\n", position.x(), position.y(), position.z());

        const Vector3 orientation = solution->x_star[i].orientation;
        printf("Orientation: %f %f %f\n", orientation.x(), orientation.y(), orientation.z());

        const Vector3 linear_velocity = solution->x_star[i].linear_velocity;
        printf("Linear velocity: %f %f %f\n", linear_velocity.x(), linear_velocity.y(), linear_velocity.z());

        const Vector3 angular_velocity = solution->x_star[i].angular_velocity;
        printf("Angular velocity: %f %f %f\n", angular_velocity.x(), angular_velocity.y(), angular_velocity.z());

        // Print foot forces at each node
        if (i < mpc_config->getWindowSize() - 1)
        {
            // Get force from the solution
            const FootForceState force_state = solution->u_star[i];
            for (size_t j = 0; j < force_state.size(); j++)
            {
                if (force_state[j].first)
                {
                    // Print foot force if in stance
                    Vector3 foot_force = force_state[j].second;
                    printf("Force[%lu]: %f %f %f\n", j, foot_force.x(), foot_force.y(), foot_force.z());
                }
            }
        }
    }
}