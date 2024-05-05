#include <stdio.h>

#include "starq/planning/planning_solver.hpp"
#include "starq/testing/dummy_planning_model.hpp"

using namespace starq;
using namespace starq::testing;
using namespace starq::planning;

int main()
{

    // Create dummy planning model
    DummyPlanningModel::Ptr model = std::make_shared<DummyPlanningModel>();
    printf("Model created.\n");

    // Create STARQ planning solver
    PlanSolver::Ptr solver = std::make_shared<PlanSolver>();
    printf("Solver created.\n");

    // Create plan configuration
    PlanConfiguration::Ptr config = std::make_shared<PlanConfiguration>();
    config->dx = VectorX::Ones(2) * 0.1;
    config->dt = 0.1;
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
        printf("Node: %f %f\n", node->x(0), node->x(1));
    }

    printf("Done.\n");
    return 0;
}