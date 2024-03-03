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
                                Vector3f(1.0, 1.0, 1.0),
                                Vector3f(1.0, 0.0, 0.0),
                                Vector3f(1.0, 1.0, 1.0));

    mpc_planner.setControlWeights(Vector3f(1E-6, 1E-6, 1E-6));

    mpc_planner.setControlBounds(Vector3f(-1000, -1000, -1000),
                                 Vector3f(1000, 1000, 1000));

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

        mpc_solver.setup();
        printf("OSQP_MPCSolver setup\n");

        mpc_solver.solve();
        printf("OSQP_MPCSolver solved\n");

        auto x = mpc_solver.getSolver()->solution->x;

        const Vector3f current_com_orientation = robot->getLocalization()->getCurrentOrientation();
        Matrix3f current_com_rotation;
        current_com_rotation = AngleAxisf(current_com_orientation.x(), Vector3f::UnitX()) *
                               AngleAxisf(current_com_orientation.y(), Vector3f::UnitY()) *
                               AngleAxisf(current_com_orientation.z(), Vector3f::UnitZ());

        int offset = mpc_solver.sizeX();
        for (size_t i = 0; i < config.window_size; i++)
        {
            printf("------\n");
            printf("Node[%lu]\n", i);

            Vector3f orientation(x[13 * i], x[13 * i + 1], x[13 * i + 2]);
            Vector3f position(x[13 * i + 3], x[13 * i + 4], x[13 * i + 5]);
            Vector3f angular_velocity(x[13 * i + 6], x[13 * i + 7], x[13 * i + 8]);
            Vector3f linear_velocity(x[13 * i + 9], x[13 * i + 10], x[13 * i + 11]);

            printf("Position: %f %f %f\n", position.x(), position.y(), position.z());
            printf("Orientation: %f %f %f\n", orientation.x(), orientation.y(), orientation.z());
            printf("Linear velocity: %f %f %f\n", linear_velocity.x(), linear_velocity.y(), linear_velocity.z());
            printf("Angular velocity: %f %f %f\n", angular_velocity.x(), angular_velocity.y(), angular_velocity.z());

            for (size_t j = 0; j < config.stance_trajectory[i].size(); j++)
            {
                if (config.stance_trajectory[i][j])
                {
                    Vector3f foot_force(x[offset], x[offset + 1], x[offset + 2]);
                    printf("Force[%lu]: %f %f %f\n", j, foot_force.x(), foot_force.y(), foot_force.z());
                    offset += 3;
                }
            }
        }

        offset = mpc_solver.sizeX();
        for (size_t j = 0; j < config.stance_trajectory[0].size(); j++)
        {
            if (config.stance_trajectory[0][j])
            {
                Vector3f foot_force(x[offset], x[offset + 1], x[offset + 2]);
                robot->setFootForce(j, -foot_force);
                printf("Applied Force[%lu]: %f %f %f\n", j, foot_force.x(), foot_force.y(), foot_force.z());
                offset += 3;
            }
        }

        auto global_time = robot->getLocalization()->getCurrentTime();
        printf("Global time: %d\n", int(global_time.count()));

        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    robot->waitForSimulation();
    printf("Simulation closed\n");

    printf("Done\n");
    return 0;
}