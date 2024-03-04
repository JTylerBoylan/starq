#include <stdio.h>

#include "starq/robots/unitree_a1_mujoco.hpp"
#include "starq/osqp/osqp.hpp"

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

    MPCConfiguration::Ptr mpc_config = std::make_shared<MPCConfiguration>(robot->getLegs(),
                                                                          robot->getRobotDynamics(),
                                                                          robot->getLocalization());
    mpc_config->setTimeStep(milliseconds(50));
    mpc_config->setWindowSize(21);
    mpc_config->setNextGait(gait);
    printf("MPCConfiguration created\n");

    OSQP::Ptr osqp = std::make_shared<OSQP>();
    osqp->getSettings()->verbose = false;
    osqp->getSettings()->max_iter = 2000;
    osqp->getSettings()->polishing = true;
    osqp->getSettings()->warm_starting = true;
    printf("OSQP created\n");

    robot->startSimulation();
    printf("Simulation started\n");

    const auto foot_position = Eigen::Vector3f(0, UNITREE_A1_LENGTH_D, -UNITREE_A1_HEIGHT);
    for (uint8_t id = 0; id < UNITREE_A1_NUM_LEGS; id++)
    {
        robot->setFootPosition(id, foot_position);
    }
    printf("Holding foot position for 5 seconds...\n");
    std::this_thread::sleep_for(std::chrono::seconds(5));

    size_t c = 0;
    auto cstart = std::chrono::high_resolution_clock::now();
    printf("Starting MPC loop\n");
    while (robot->isSimulationOpen())
    {
        if (!osqp->update(mpc_config))
            break;

        if (!osqp->solve())
            break;

        auto solution = osqp->getSolution();

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

        const Vector3f current_orientation = robot->getLocalization()->getCurrentOrientation();
        const Matrix3f current_rotation = robot->getLocalization()->toRotationMatrix(current_orientation);

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

                Vector3f foot_force_world = -solution.u_star[0][j].second;
                Vector3f foot_force_body = current_rotation.transpose() * foot_force_world;
                robot->setFootForce(j, foot_force_body);
                printf("%s Force: %f %f %f\n", leg_name.c_str(), foot_force_body.x(), foot_force_body.y(), foot_force_body.z());
            }
        }
        printf("\n");

        c++;
        auto cend = std::chrono::high_resolution_clock::now();
        auto ctime = std::chrono::duration_cast<std::chrono::milliseconds>(cend - cstart);
        printf("Frequency: %lu Hz\n", (1000 * c) / ctime.count());

        printf("------\n");
        printf("\n\n");

        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }

    robot->waitForSimulation();
    printf("Simulation closed\n");

    printf("Done\n");
    return 0;
}