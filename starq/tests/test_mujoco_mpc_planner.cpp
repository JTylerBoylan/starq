#include <stdio.h>

#include "starq/robots/unitree_a1_mujoco.hpp"
#include "starq/mpc/mpc_planner.hpp"

using namespace starq;
using namespace starq::mpc;

int main()
{

    robots::UnitreeA1MuJoCoRobot robot;
    printf("UnitreeA1MuJoCoRobot created\n");

    MPCPlanner mpc_planner(robot.getLocalization());

    Gait::Ptr gait = std::make_shared<Gait>();
    gait->load("/home/nvidia/starq_ws/src/starq/gaits/walk.txt");

    robot.startSimulation();
    printf("Simulation started\n");

    while (robot.isSimulationOpen())
    {
        mpc_planner.setNextGait(gait);
        MPCConfiguration config;
        mpc_planner.getPlan(config);

        auto global_time = robot.getLocalization()->getCurrentTime();
        printf("Global time: %d\n", int(global_time.count()));

        printf("t \t stance \t com pos \t\t com ori \n");
        for (size_t i = 0; i < config.window_size; i++)
        {
            auto time = config.time_step * i;
            printf("%d \t ", int(time.count()));

            StanceState stance_state = config.stance_trajectory[i];
            for (size_t j = 0; j < stance_state.size(); j++)
            {
                printf("%d ", int(stance_state[j]));
            }
            printf("\t");

            CenterOfMassState com_state = config.com_trajectory[i];
            printf("%.3f %.3f %.3f \t %.3f %.3f %.3f \n",
                   com_state.position.x(),
                   com_state.position.y(),
                   com_state.position.z(),
                   com_state.orientation.x(),
                   com_state.orientation.y(),
                   com_state.orientation.z());
        }
        printf("------\n");

        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    robot.waitForSimulation();
    printf("Simulation closed\n");

    printf("Done\n");
    return 0;
}