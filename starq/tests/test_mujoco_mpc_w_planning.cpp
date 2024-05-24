#include <stdio.h>

#include "starq/unitree/unitree_a1_mujoco_robot.hpp"
#include "starq/osqp/osqp.hpp"
#include "starq/mpc/mpc_controller.hpp"
#include "starq/planning/planning_solver.hpp"
#include "starq/starq/starq_planning_model.hpp"

using namespace starq;
using namespace starq::mpc;
using namespace starq::osqp;
using namespace starq::unitree;
using namespace starq::planning;

void printSolution(OSQP::Ptr osqp, MPCConfiguration::Ptr mpc_config);

int main(void)
{
    // Set simulation frame rate
    mujoco::MuJoCo::getInstance()->setFrameRate(30);

    // Create Unitree A1 robot
    auto robot = std::make_shared<UnitreeA1MuJoCoRobot>();
    printf("UnitreeA1MuJoCoRobot created\n");

    // Load walk gait from file
    Gait::Ptr walk_gait = std::make_shared<Gait>();
    walk_gait->load("/home/nvidia/starq_ws/src/starq/gaits/walk.txt");
    printf("Walk Gait loaded\n");

    // Set gait to walk forward at constant velocity
    walk_gait->setVelocity(Vector3(0, 0, 0), Vector3(0, 0, 0));
    walk_gait->setFrequency(3.0);

    // Load stand gait from file
    Gait::Ptr stand_gait = std::make_shared<Gait>();
    stand_gait->load("/home/nvidia/starq_ws/src/starq/gaits/stand.txt");
    printf("Stand Gait loaded\n");

    // Stand still at default height
    stand_gait->setPose(Vector3(0, 0, UNITREE_A1_STAND_HEIGHT), Vector3(0, 0, 0));
    stand_gait->setFrequency(10.0);

    // MPC settings
    robot->getMPCConfiguration()->setTimeStep(milliseconds(50));
    robot->getMPCConfiguration()->setWindowSize(21);

    // OSQP settings
    robot->getOSQPSolver()->getSettings()->verbose = false;
    robot->getOSQPSolver()->getSettings()->max_iter = 2000;
    robot->getOSQPSolver()->getSettings()->polishing = true;
    robot->getOSQPSolver()->getSettings()->warm_starting = true;

    // Goal states
    int goal_index = 0;
    const std::vector<Vector3> goal_states = {Vector3(1.0, 1.0, 0.0),
                                              Vector3(1.0, -1.0, 0.0),
                                              Vector3(-1.0, -1.0, 0.0),
                                              Vector3(-1.0, 1.0, 0.0)};

    // Create STARQ planning solver
    PlanSolver::Ptr solver = std::make_shared<PlanSolver>();
    printf("Solver created.\n");

    // Create plan configuration
    PlanConfiguration::Ptr config = std::make_shared<PlanConfiguration>();
    config->dx = Vector3(0.05, 0.05, M_PI / 64.0);
    config->dt = 0.15;
    config->time_limit = milliseconds(5000);
    config->max_iterations = 500000;
    config->max_generations = 250;
    printf("Configuration created.\n");

    // Create starq planning model
    STARQPlanningModel::Ptr model = std::make_shared<STARQPlanningModel>(robot->getLocalization());
    model->setGoalState(goal_states[goal_index]);
    model->setGoalThreshold(0.10);
    model->setGridResolution(config->dx);
    model->setTimeStep(config->dt);
    printf("Model created.\n");

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
    robot->startMPC();
    printf("MPCController started\n");

    // Run standing gait for 5 seconds
    printf("Standing for 5 seconds...\n");
    robot->setNextGait(stand_gait);
    std::this_thread::sleep_for(std::chrono::seconds(5));

    // Transition to walking gait
    printf("Walking...\n");
    robot->setNextGait(walk_gait);

    printf("Starting Planner\n");
    while (robot->isSimulationOpen())
    {

        // Print position
        const Vector3 position = robot->getLocalization()->getCurrentPosition();
        const Vector3 orientation = robot->getLocalization()->getCurrentOrientation();
        printf("Position: x: %f y: %f th: %f ", position.x(), position.y(), orientation.z());

        // Solve
        PlanResults::Ptr results = solver->solve(config, model);
        printf("  (%d)  ", results->exit_code);

        switch (results->exit_code)
        {
        case ExitCode::SUCCESS:
            // Send command
            if (results->node_path.size() > 1)
            {
                VectorX u = results->node_path[1]->u;
                printf("  ->  Send: vx: %f vy: %f w: %f", u(0), u(1), u(2));
                walk_gait->setVelocity(Vector3(u(0), u(1), 0), Vector3(0, 0, u(2)));
            }
            break;
        case ExitCode::START_IS_FINAL:
            // Cycle through goal states
            goal_index = (goal_index + 1) % goal_states.size();
            model->setGoalState(goal_states[goal_index]);
            printf(" -> New goal: x: %f y: %f th: %f", goal_states[goal_index].x(), goal_states[goal_index].y(), goal_states[goal_index].z());
            walk_gait->setVelocity(Vector3(0, 0, 0), Vector3(0, 0, 0));
            break;
        default:
            // Print error
            printf(" -> Planner failed with exit code: %d", results->exit_code);
            walk_gait->setVelocity(Vector3(0, 0, 0), Vector3(0, 0, 0));
            break;
        }
        printf("\n");

        // Sleep for 10 milliseconds
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    // Wait for simulation to close
    robot->waitForSimulation();
    printf("Simulation closed\n");

    printf("Done\n");
    return 0;
}