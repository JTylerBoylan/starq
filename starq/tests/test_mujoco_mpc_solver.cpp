#include <stdio.h>

#include "starq/robots/unitree_a1_mujoco.hpp"
#include "starq/mpc/mpc_planner.hpp"
#include "starq/mpc/mpc_solver.hpp"

using namespace starq;
using namespace starq::mpc;

int main()
{

    auto robot = std::make_shared<robots::UnitreeA1MuJoCoRobot>();
    printf("UnitreeA1MuJoCoRobot created\n");

    Gait::Ptr gait = std::make_shared<Gait>();
    gait->load("/home/nvidia/starq_ws/src/starq/gaits/walk.txt");
    printf("Gait loaded\n");

    gait->setVelocity(Vector3f(1.0, 0, 0), Vector3f(0, 0, 0));

    auto duration = gait->getDuration();
    auto stance_duration = gait->getStanceDuration();
    auto swing_duration = gait->getSwingDuration();
    printf("Gait duration: %d\n", int(duration.count()));
    printf("Stance duration: %d\n", int(stance_duration.count()));
    printf("Swing duration: %d\n", int(swing_duration.count()));

    MPCPlanner mpc_planner(robot);
    printf("MPCPlanner created\n");

    mpc_planner.setTimeStep(milliseconds(50));
    mpc_planner.setWindowSize(21);

    mpc_planner.setNextGait(gait);
    printf("Gait set\n");

    std::this_thread::sleep_for(std::chrono::milliseconds(1000));

    robot->startSimulation();
    printf("Simulation started\n");

    while (robot->isSimulationOpen())
    {

        MPCConfiguration config;
        mpc_planner.getConfiguration(config);

        MPCSolver mpc_solver(config);

        auto global_time = robot->getLocalization()->getCurrentTime();
        printf("Global time: %d\n", int(global_time.count()));

        mpc_solver.print();

        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    robot->waitForSimulation();
    printf("Simulation closed\n");

    printf("Done\n");
    return 0;
}