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

    // Create Unitree A1 robot
    auto robot = std::make_shared<UnitreeA1MuJoCoRobot>();
    printf("UnitreeA1MuJoCoRobot created\n");

    // Load stand gait from file
    Gait::Ptr gait = std::make_shared<Gait>();
    gait->load("/home/nvidia/starq_ws/src/starq/gaits/stand.txt");
    printf("Gait loaded\n");

    // Set gait to stand still at default height using velocity control
    // gait ->setControlMode(GAIT_VELOCITY_CONTROL);
    // gait->setVelocity(Vector3f(0, 0, 0), Vector3f(0, 0, 0));

    // Set gait to stand still at default height using pose control
    gait->setPose(Vector3(0, 0, UNITREE_A1_STAND_HEIGHT), Vector3(0, 0, 0));

    // Print gait durations
    auto duration = gait->getDuration();
    printf("Gait duration: %d\n", int(duration.count()));

    auto stance_duration = gait->getStanceDuration();
    printf("Stance duration: %d\n", int(stance_duration.count()));

    auto swing_duration = gait->getSwingDuration();
    printf("Swing duration: %d\n", int(swing_duration.count()));

    // Create MPC Configuration object based on the robot
    MPCConfiguration::Ptr mpc_config = std::make_shared<MPCConfiguration>(robot->getLegs(),
                                                                          robot->getRobotParameters(),
                                                                          robot->getLocalization());
    printf("MPCConfiguration created\n");

    // Set MPC parameters
    mpc_config->setTimeStep(milliseconds(20));
    mpc_config->setWindowSize(21);

    // Set the next gait
    mpc_config->setNextGait(gait);

    // Create OSQP solver object
    OSQP::Ptr osqp = std::make_shared<OSQP>();
    printf("OSQP created\n");

    // Set OSQP parameters
    osqp->getSettings()->verbose = false;
    osqp->getSettings()->max_iter = 2000;
    osqp->getSettings()->polishing = true;
    osqp->getSettings()->warm_starting = true;

    // Set simulation frame rate
    // Lower frame rates can be used to speed up simulation
    MuJoCo::getInstance()->setFrameRate(60.0);

    // Start simulation
    robot->startSimulation();
    printf("Simulation started\n");

    // Send feet to default positions
    for (uint8_t id = 0; id < UNITREE_A1_NUM_LEGS; id++)
    {
        robot->setFootPosition(id, robot->getRobotParameters()->getDefaultFootLocations()[id]);
    }
    printf("Holding foot position for 5 seconds...\n");

    // Wait for simulation to settle
    std::this_thread::sleep_for(std::chrono::seconds(5));

    // Keep track of average solve frequency
    size_t c = 0;
    auto cstart = std::chrono::high_resolution_clock::now();

    // Start MPC loop
    printf("Starting MPC loop\n");
    while (robot->isSimulationOpen())
    {
        // Get global time
        auto global_time = robot->getLocalization()->getCurrentTime();

        // Use these variables to test various standing motions
        float offset_x = 0;       // 0.025 * std::cos(2 * 1E-3 * global_time.count());
        float offset_y = 0;       // 0.025 * std::sin(2 * 1E-3 * global_time.count());
        float offset_z = 0;       // 0.05 * std::sin(2 * 1E-3 * global_time.count());
        float offset_roll = 0.0;  // 0.1 * std::sin(2 * 1E-3 * global_time.count());
        float offset_pitch = 0.0; // 0.2 * std::cos(2 * 1E-3 * global_time.count());
        float offset_yaw = 0.0;   // 0.1 * std::sin(2 * 1E-3 * global_time.count());

        // Set gait pose
        gait->setPose(Vector3(offset_x, offset_y, UNITREE_A1_STAND_HEIGHT + offset_z),
                      Vector3(offset_roll, offset_pitch, offset_yaw));

        // Update MPC configuration
        if (!osqp->update(mpc_config))
            break;

        // Solve MPC
        if (!osqp->solve())
            break;

        // Get MPC solution
        auto solution = osqp->getSolution();

        // Print MPC solution at each node
        for (size_t i = 0; i < mpc_config->getWindowSize(); i++)
        {
            printf("------\n");
            printf("Node[%lu]\n", i);

            // Print reference state at each node
            const Vector3 position = solution->x_star[i].position;
            printf("Position: %f %f %f\n", position.x(), position.y(), position.z());

            const Vector3 orientation = solution->x_star[i].orientation;
            printf("Orientation: %f %f %f\n", orientation.x(), orientation.y(), orientation.z());

            const Vector3 linear_velocity = solution->x_star[i].linear_velocity;
            printf("Linear velocity: %f %f %f\n", linear_velocity.x(), linear_velocity.y(), linear_velocity.z());

            const Vector3 angular_velocity = solution->x_star[i].angular_velocity;
            printf("Angular velocity: %f %f %f\n", angular_velocity.x(), angular_velocity.y(), angular_velocity.z());

            // Print foot forces at each node
            if (i < mpc_config->getWindowSize() - 1)
            {
                // Get force from the solution
                const FootForceState force_state = solution->u_star[i];
                for (size_t j = 0; j < force_state.size(); j++)
                {
                    if (force_state[j].first)
                    {
                        // Print foot force if in stance
                        Vector3 foot_force = force_state[j].second;
                        printf("Force[%lu]: %f %f %f\n", j, foot_force.x(), foot_force.y(), foot_force.z());
                    }
                }
            }
        }

        // Print localization information
        printf("------\n");

        // Print global time
        printf("Global time: %d\n\n", int(global_time.count()));

        // Print current position
        Vector3 position = robot->getLocalization()->getCurrentPosition();
        printf("Position: %f %f %f\n", position.x(), position.y(), position.z());

        // Print current orientation
        Vector3 orientation = robot->getLocalization()->getCurrentOrientation();
        printf("Orientation: %f %f %f\n", orientation.x(), orientation.y(), orientation.z());

        // Print current linear velocity
        Vector3 linear_velocity = robot->getLocalization()->getCurrentLinearVelocity();
        printf("Linear velocity: %f %f %f\n", linear_velocity.x(), linear_velocity.y(), linear_velocity.z());

        // Print current angular velocity
        Vector3 angular_velocity = robot->getLocalization()->getCurrentAngularVelocity();
        printf("Angular velocity: %f %f %f\n", angular_velocity.x(), angular_velocity.y(), angular_velocity.z());
        printf("\n");

        // Get current rotation matrix
        const Matrix3 current_rotation = robot->getLocalization()->toRotationMatrix(orientation);

        // Print leg commands
        for (size_t j = 0; j < solution->u_star[0].size(); j++)
        {
            // Only apply foot forces if in stance
            if (solution->u_star[0][j].first)
            {
                // Get leg name
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

                // Get foot force from the solution
                // Apply the force in the opposite direction, since the MPC solves for ground reaction forces
                Vector3 foot_force = -solution->u_star[0][j].second;

                // Send foot force to the robot
                robot->setFootForce(j, foot_force);
                printf("%s Force: %f %f %f\n", leg_name.c_str(), foot_force.x(), foot_force.y(), foot_force.z());
            }
        }
        printf("\n");

        // Calculate average solve frequency
        auto cend = std::chrono::high_resolution_clock::now();
        auto ctime = std::chrono::duration_cast<std::chrono::milliseconds>(cend - cstart);
        printf("Frequency: %lu Hz\n", (1000 * c++) / ctime.count());

        printf("------\n");
        printf("\n\n");

        // Use to limit MPC frequency, or 0 for max performance
        std::this_thread::sleep_for(std::chrono::milliseconds(0));
    }

    // Wait for simulation to close
    robot->waitForSimulation();
    printf("Simulation closed\n");

    printf("Done\n");
    return 0;
}