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
    mpc_planner.setWindowSize(21);

    mpc_planner.setStateWeights(Vector3f(10.0, 10.0, 50.0),
                                Vector3f(1.0, 1.0, 1.0),
                                Vector3f(1.0, 1.0, 1.0),
                                Vector3f(1.0, 1.0, 1.0));

    mpc_planner.setControlWeights(Vector3f(1E-6, 1E-6, 1E-6));

    mpc_planner.setControlBounds(10, 250);

    mpc_planner.setNextGait(gait);
    printf("Gait set\n");

    std::this_thread::sleep_for(std::chrono::milliseconds(1000));

    robot->startSimulation();
    printf("Simulation started\n");

    const auto foot_position = Eigen::Vector3f(0, UNITREE_A1_LENGTH_D, -UNITREE_A1_HEIGHT);
    for (uint8_t id = 0; id < UNITREE_A1_NUM_LEGS; id++)
    {
        robot->setFootPosition(id, foot_position);
    }
    std::this_thread::sleep_for(std::chrono::seconds(5));

    while (robot->isSimulationOpen())
    {
        MPCConfiguration config;
        mpc_planner.getConfiguration(config);

        OSQP_MPCSolver mpc_solver(config);
        mpc_solver.getSettings()->verbose = true;
        mpc_solver.getSettings()->max_iter = 100000;
        mpc_solver.getSettings()->polishing = true;
        mpc_solver.getSettings()->warm_starting = true;

        mpc_solver.setup();
        printf("OSQP_MPCSolver setup\n");

        auto solution = mpc_solver.solve();
        printf("OSQP_MPCSolver solved\n");

        const Vector3f current_com_orientation = robot->getLocalization()->getCurrentOrientation();
        Matrix3f current_com_rotation;
        current_com_rotation = AngleAxisf(current_com_orientation.x(), Vector3f::UnitX()) *
                               AngleAxisf(current_com_orientation.y(), Vector3f::UnitY()) *
                               AngleAxisf(current_com_orientation.z(), Vector3f::UnitZ());

        // for (size_t i = 0; i < config.window_size; i++)
        // {
        //     printf("------\n");
        //     printf("Node[%lu]\n", i);

        //     const Vector3f position = solution.x_star[i].position;
        //     const Vector3f orientation = solution.x_star[i].orientation;
        //     const Vector3f linear_velocity = solution.x_star[i].linear_velocity;
        //     const Vector3f angular_velocity = solution.x_star[i].angular_velocity;

        //     printf("Position: %f %f %f\n", position.x(), position.y(), position.z());
        //     printf("Orientation: %f %f %f\n", orientation.x(), orientation.y(), orientation.z());
        //     printf("Linear velocity: %f %f %f\n", linear_velocity.x(), linear_velocity.y(), linear_velocity.z());
        //     printf("Angular velocity: %f %f %f\n", angular_velocity.x(), angular_velocity.y(), angular_velocity.z());

        //     for (auto force_state : solution.u_star)
        //     {
        //         for (size_t j = 0; j < force_state.size(); j++)
        //         {
        //             if (force_state[j].first)
        //             {
        //                 Vector3f foot_force = force_state[j].second;
        //                 printf("Force[%lu]: %f %f %f\n", j, foot_force.x(), foot_force.y(), foot_force.z());
        //             }
        //         }
        //     }
        // }

        printf("------\n");
        auto global_time = robot->getLocalization()->getCurrentTime();
        printf("Global time: %d\n\n", int(global_time.count()));
        auto position = robot->getLocalization()->getCurrentPosition();
        auto orientation = robot->getLocalization()->getCurrentOrientation();
        auto linear_velocity = robot->getLocalization()->getCurrentLinearVelocity();
        auto angular_velocity = robot->getLocalization()->getCurrentAngularVelocity();
        printf("Current Position: %f %f %f\n", position.x(), position.y(), position.z());
        printf("Current Orientation: %f %f %f\n", orientation.x(), orientation.y(), orientation.z());
        printf("Current Linear velocity: %f %f %f\n", linear_velocity.x(), linear_velocity.y(), linear_velocity.z());
        printf("Current Angular velocity: %f %f %f\n", angular_velocity.x(), angular_velocity.y(), angular_velocity.z());
        printf("\n");

        for (size_t j = 0; j < solution.u_star[0].size(); j++)
        {
            if (solution.u_star[0][j].first)
            {
                std::string leg_name;
                switch (j)
                {
                case 0:
                    leg_name = "FL";
                    break;
                case 1:
                    leg_name = "RL";
                    break;
                case 2:
                    leg_name = "RR";
                    break;
                case 3:
                    leg_name = "FR";
                    break;
                }

                Vector3f foot_force = -solution.u_star[0][j].second;
                robot->setFootForce(j, foot_force);
                printf("%s Force: %f %f %f\n", leg_name.c_str(), foot_force.x(), foot_force.y(), foot_force.z());
            }
        }

        printf("------\n");
        printf("\n\n");

        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    robot->waitForSimulation();
    printf("Simulation closed\n");

    printf("Done\n");
    return 0;
}