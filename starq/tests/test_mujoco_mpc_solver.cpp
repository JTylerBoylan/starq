#include <stdio.h>
#include <iostream>

#include "starq/unitree/unitree_a1_mujoco_robot.hpp"
#include "starq/osqp/osqp.hpp"

using namespace starq;
using namespace starq::mpc;
using namespace starq::osqp;
using namespace starq::unitree;

int main()
{

    auto robot = std::make_shared<UnitreeA1MuJoCoRobot>();
    printf("UnitreeA1MuJoCoRobot created\n");

    Gait::Ptr gait = std::make_shared<Gait>();
    gait->load("/home/nvidia/starq_ws/src/starq/gaits/stand.txt");
    printf("Gait loaded\n");

    // gait ->setControlMode(GAIT_VELOCITY_CONTROL);
    // gait->setVelocity(Vector3f(0, 0, 0), Vector3f(0, 0, 0));

    gait->setPose(Vector3(0, 0, UNITREE_A1_STAND_HEIGHT), Vector3(0, 0, 0));
    auto duration = gait->getDuration();
    auto stance_duration = gait->getStanceDuration();
    auto swing_duration = gait->getSwingDuration();
    printf("Gait duration: %d\n", int(duration.count()));
    printf("Stance duration: %d\n", int(stance_duration.count()));
    printf("Swing duration: %d\n", int(swing_duration.count()));

    MPCConfiguration::Ptr mpc_config = std::make_shared<MPCConfiguration>(robot->getLegs(),
                                                                          robot->getRobotParameters(),
                                                                          robot->getLocalization());
    mpc_config->setTimeStep(milliseconds(20));
    mpc_config->setWindowSize(21);
    mpc_config->setNextGait(gait);
    printf("MPCConfiguration created\n");

    OSQP::Ptr osqp = std::make_shared<OSQP>();
    osqp->getSettings()->verbose = false;
    osqp->getSettings()->max_iter = 2000;
    osqp->getSettings()->polishing = true;
    osqp->getSettings()->warm_starting = true;
    printf("OSQP created\n");

    MuJoCo::getInstance()->setFrameRate(60.0);

    robot->startSimulation();
    printf("Simulation started\n");

    for (uint8_t id = 0; id < UNITREE_A1_NUM_LEGS; id++)
    {
        robot->setFootPosition(id, robot->getRobotParameters()->getDefaultFootLocations()[id]);
    }
    printf("Holding foot position for 5 seconds...\n");
    std::this_thread::sleep_for(std::chrono::seconds(5));

    size_t c = 0;
    auto cstart = std::chrono::high_resolution_clock::now();
    printf("Starting MPC loop\n");
    while (robot->isSimulationOpen())
    {
        auto global_time = robot->getLocalization()->getCurrentTime();

        float offset_x = 0; // 0.025 * std::cos(2 * 1E-3 * global_time.count());
        float offset_y = 0; // 0.025 * std::sin(2 * 1E-3 * global_time.count());
        float offset_z = 0; // 0.05 * std::sin(2 * 1E-3 * global_time.count());

        float offset_roll = 0.0;     // 0.1 * std::sin(2 * 1E-3 * global_time.count());
        float offset_pitch = 0.0;    // 0.2 * std::cos(2 * 1E-3 * global_time.count());
        float offset_yaw = 0.0; // 0.1 * std::sin(2 * 1E-3 * global_time.count());

        gait->setPose(Vector3(offset_x, offset_y, UNITREE_A1_STAND_HEIGHT + offset_z),
                      Vector3(offset_roll, offset_pitch, offset_yaw));

        if (!osqp->update(mpc_config))
            break;

        if (!osqp->solve())
            break;

        auto solution = osqp->getSolution();

        for (size_t i = 0; i < mpc_config->getWindowSize(); i++)
        {
            printf("------\n");
            printf("Node[%lu]\n", i);

            const Vector3 position = solution->x_star[i].position;
            const Vector3 orientation = solution->x_star[i].orientation;
            const Vector3 linear_velocity = solution->x_star[i].linear_velocity;
            const Vector3 angular_velocity = solution->x_star[i].angular_velocity;

            printf("Position: %f %f %f\n", position.x(), position.y(), position.z());
            printf("Orientation: %f %f %f\n", orientation.x(), orientation.y(), orientation.z());
            printf("Linear velocity: %f %f %f\n", linear_velocity.x(), linear_velocity.y(), linear_velocity.z());
            printf("Angular velocity: %f %f %f\n", angular_velocity.x(), angular_velocity.y(), angular_velocity.z());

            if (i < mpc_config->getWindowSize() - 1)
            {
                const FootForceState force_state = solution->u_star[i];
                for (size_t j = 0; j < force_state.size(); j++)
                {
                    if (force_state[j].first)
                    {
                        Vector3 foot_force = force_state[j].second;
                        printf("Force[%lu]: %f %f %f\n", j, foot_force.x(), foot_force.y(), foot_force.z());
                    }
                }
            }
        }

        printf("------\n");
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

        const Vector3 current_orientation = robot->getLocalization()->getCurrentOrientation();
        const Matrix3 current_rotation = robot->getLocalization()->toRotationMatrix(current_orientation);

        for (size_t j = 0; j < solution->u_star[0].size(); j++)
        {
            if (solution->u_star[0][j].first)
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

                Vector3 foot_force_world = -solution->u_star[0][j].second;
                Vector3 foot_force_body = current_rotation.transpose() * foot_force_world;
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

        std::this_thread::sleep_for(std::chrono::milliseconds(0));
    }

    robot->waitForSimulation();
    printf("Simulation closed\n");

    printf("Done\n");
    return 0;
}