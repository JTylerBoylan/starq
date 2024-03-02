#include <stdio.h>

#include "starq/robots/unitree_a1_mujoco.hpp"
#include "starq/mpc/mpc_planner.hpp"

using namespace starq;
using namespace starq::mpc;

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
    mpc_planner.setWindowSize(21);

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

    while (robot->isSimulationOpen())
    {

        MPCConfiguration config;
        mpc_planner.getConfiguration(config);

        auto global_time = robot->getLocalization()->getCurrentTime();
        printf("Global time: %d\n", int(global_time.count()));

        printf("t \t stance \t com pos \t\t com ori \t\t com vel \t\t com rot \n");
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
            printf("%.3f %.3f %.3f \t %.3f %.3f %.3f \t %.3f %.3f %.3f \t %.3f %.3f %.3f \n",
                   com_state.position.x(),
                   com_state.position.y(),
                   com_state.position.z(),
                   com_state.orientation.x(),
                   com_state.orientation.y(),
                   com_state.orientation.z(),
                   com_state.linear_velocity.x(),
                   com_state.linear_velocity.y(),
                   com_state.linear_velocity.z(),
                   com_state.angular_velocity.x(),
                   com_state.angular_velocity.y(),
                   com_state.angular_velocity.z());
        }
        printf("------\n");
        printf("t \t FL \t\t\t RL \t\t\t RR \t\t\t FR \n");
        for (size_t i = 0; i < config.window_size; i++)
        {
            auto time = config.time_step * i;
            printf("%d \t ", int(time.count()));

            for (size_t j = 0; j < config.foothold_trajectory[i].size(); j++)
            {
                printf("%.3f %.3f %.3f \t",
                       config.foothold_trajectory[i][j].x(),
                       config.foothold_trajectory[i][j].y(),
                       config.foothold_trajectory[i][j].z());
            }
            printf("\n");
        }
        printf("------\n");
        printf("------\n");

        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    robot->waitForSimulation();
    printf("Simulation closed\n");

    printf("Done\n");
    return 0;
}