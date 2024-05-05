#include <stdio.h>

#include "starq/system_localization.hpp"
#include "starq/planning/planning_solver.hpp"
#include "starq/starq/starq_planning_model.hpp"

using namespace starq;
using namespace starq::planning;

int main()
{
    // Create localization
    SystemLocalization::Ptr localization = std::make_shared<SystemLocalization>();

    // Create starq planning model
    STARQPlanningModel::Ptr model = std::make_shared<STARQPlanningModel>(localization);
    model->setGoalState(Vector3(-10.0, 10.0, 0.0));
    model->setGoalThreshold(0.1);
    printf("Model created.\n");

    // Create STARQ planning solver
    PlanSolver::Ptr solver = std::make_shared<PlanSolver>();
    printf("Solver created.\n");

    // Create plan configuration
    PlanConfiguration::Ptr config = std::make_shared<PlanConfiguration>();
    config->dx = Vector3(0.1, 0.1, M_PI / 16.0);
    config->dt = 0.25;
    config->time_limit = milliseconds(10000);
    printf("Configuration created.\n");

    // Solve
    PlanResults::Ptr results = solver->solve(config, model);
    printf("Solved.\n");

    // Print results
    printf("Exit code: %d\n", results->exit_code);
    printf("Time elapsed: %lu ms\n", results->time_elapsed.count());
    printf("Iterations: %d\n", results->iterations);
    printf("Cost: %f\n", results->cost);
    for (auto node : results->node_path)
    {
        printf("Node: x: %f y: %f th: %f g: %f h: %f f: %f\n", node->x(0), node->x(1), node->x(2), node->g, node->h, node->f);
    }

    // Save nodes
    solver->saveNodes("/home/nvidia/starq_ws/src/logging/nodes.txt");

    printf("Done.\n");
    return 0;
}