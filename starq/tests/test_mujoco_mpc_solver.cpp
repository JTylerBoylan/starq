#include <stdio.h>

#include "starq/robots/unitree_a1_mujoco.hpp"
#include "starq/mpc/mpc_planner.hpp"
#include "starq/osqp/osqp_solver.hpp"

using namespace starq;
using namespace starq::mpc;
using namespace starq::osqp;

int main()
{

    auto robot = std::make_shared<robots::UnitreeA1MuJoCoRobot>();
    printf("UnitreeA1MuJoCoRobot created\n");

    Gait::Ptr gait = std::make_shared<Gait>();
    gait->load("/home/nvidia/starq_ws/src/starq/gaits/stand.txt");
    printf("Gait loaded\n");

    gait->setVelocity(Vector3f(0, 0, 0), Vector3f(0, 0, 0));

    auto duration = gait->getDuration();
    auto stance_duration = gait->getStanceDuration();
    auto swing_duration = gait->getSwingDuration();
    printf("Gait duration: %d\n", int(duration.count()));
    printf("Stance duration: %d\n", int(stance_duration.count()));
    printf("Swing duration: %d\n", int(swing_duration.count()));

    MPCPlanner mpc_planner(robot);
    printf("MPCPlanner created\n");

    mpc_planner.setTimeStep(milliseconds(50));
    mpc_planner.setWindowSize(11);

    mpc_planner.setStateWeights(Vector3f(0.0, 0.0, 50.0),
                                Vector3f(0.0, 0.0, 1.0),
                                Vector3f(1.0, 0.0, 0.0),
                                Vector3f(0.0, 0.0, 1.0));

    mpc_planner.setControlWeights(Vector3f(0.0, 0.0, 0.0));

    mpc_planner.setControlBounds(Vector3f(-1000, -1000, -1000),
                                 Vector3f(1000, 1000, 1000));

    mpc_planner.setNextGait(gait);
    printf("Gait set\n");

    std::this_thread::sleep_for(std::chrono::milliseconds(1000));

    robot->startSimulation();
    printf("Simulation started\n");

    const auto foot_position = Eigen::Vector3f(0, UNITREE_A1_LENGTH_D, -0.27);
    for (uint8_t id = 0; id < UNITREE_A1_NUM_LEGS; id++)
    {
        robot->setFootPosition(id, foot_position);
    }
    std::this_thread::sleep_for(std::chrono::seconds(5));

    // while (robot->isSimulationOpen())
    //{

    MPCConfiguration config;
    mpc_planner.getConfiguration(config);

    OSQP_MPCSolver mpc_solver(config);
    mpc_solver.getSettings()->verbose = true;
    mpc_solver.getSettings()->max_iter = 100000;

    mpc_solver.setup();
    printf("OSQP_MPCSolver setup\n");

    mpc_solver.solve();
    printf("OSQP_MPCSolver solved\n");

    auto global_time = robot->getLocalization()->getCurrentTime();
    printf("Global time: %d\n", int(global_time.count()));

    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    //}

    robot->waitForSimulation();
    printf("Simulation closed\n");

    printf("Done\n");
    return 0;
}