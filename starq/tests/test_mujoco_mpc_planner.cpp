#include <stdio.h>

#include "starq/unitree/unitree_a1_mujoco_robot.hpp"
#include "starq/unitree/unitree_a1_robot_parameters.hpp"
#include "starq/mpc/mpc_configuration.hpp"

using namespace starq;
using namespace starq::mpc;
using namespace starq::unitree;

int main()
{

    // Create Unitree A1 robot
    // Contains the MuJoCo simulation and the Unitree A1 robot parameters
    // See test_unitree_a1_mujoco_robot.cpp for more information
    auto robot = std::make_shared<UnitreeA1MuJoCoRobot>();
    printf("UnitreeA1MuJoCoRobot created\n");

    // Create Gait object
    Gait::Ptr gait = std::make_shared<Gait>();

    // Load walk gait from file
    gait->load("/home/nvidia/starq_ws/src/starq/gaits/walk.txt");
    printf("Gait loaded\n");

    // Set gait velocity
    gait->setVelocity(Vector3(0, 0, 0), Vector3(0, 0, 0));

    // Print gait durations
    auto duration = gait->getDuration();
    printf("Gait duration: %d\n", int(duration.count()));

    auto stance_duration = gait->getStanceDuration();
    printf("Stance duration: %d\n", int(stance_duration.count()));

    auto swing_duration = gait->getSwingDuration();
    printf("Swing duration: %d\n", int(swing_duration.count()));

    // Create MPC Configuration object based on the robot
    MPCConfiguration mpc_configuration(robot->getLegs(), robot->getRobotParameters(), robot->getLocalization());
    printf("MPCPlanner created\n");

    // Set MPC parameters
    mpc_configuration.setTimeStep(milliseconds(50));
    mpc_configuration.setWindowSize(21);

    // Set the next gait
    mpc_configuration.setNextGait(gait);
    printf("Gait set\n");

    // Wait a second
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));

    // Start simulation
    robot->startSimulation();
    printf("Simulation started\n");

    // Send feet to default positions
    for (uint8_t id = 0; id < UNITREE_A1_NUM_LEGS; id++)
    {
        robot->setFootPosition(id, robot->getRobotParameters()->getDefaultFootLocations()[id]);
    }
    // Wait for the robot to settle
    std::this_thread::sleep_for(std::chrono::seconds(5));

    while (robot->isSimulationOpen())
    {

        // Update MPC configuration
        mpc_configuration.update();

        // Print global time
        auto global_time = robot->getLocalization()->getCurrentTime();
        printf("Global time: %d\n", int(global_time.count()));

        // Print table of gait sequence
        printf("t \t stance \t com pos \t\t com ori \t\t com vel \t\t com rot \n");
        for (size_t i = 0; i < mpc_configuration.getWindowSize(); i++)
        {
            // Print time
            auto time = mpc_configuration.getTimeStep() * i;
            printf("%.2f \t ", time);

            // Print stance
            StanceState stance_state = mpc_configuration.getStanceState(i);
            for (size_t j = 0; j < stance_state.size(); j++)
            {
                printf("%d ", int(stance_state[j]));
            }
            printf("\t");

            // Print reference state
            ReferenceState com_state = mpc_configuration.getReferenceState(i);
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

        // Print table of foothold sequence
        printf("t \t FL \t\t\t RL \t\t\t RR \t\t\t FR \n");
        for (size_t i = 0; i < mpc_configuration.getWindowSize(); i++)
        {
            // Print time
            auto time = mpc_configuration.getTimeStep() * i;
            printf("%.2f \t ", time);

            // Print foothold state for each leg
            for (size_t j = 0; j < mpc_configuration.getFootholdState(i).size(); j++)
            {
                printf("%.3f %.3f %.3f \t",
                       mpc_configuration.getFootholdState(i)[j].x(),
                       mpc_configuration.getFootholdState(i)[j].y(),
                       mpc_configuration.getFootholdState(i)[j].z());
            }
            printf("\n");
        }
        printf("------\n");
        printf("------\n");

        // Wait for 100 ms
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    // Wait for simulation to finish
    robot->waitForSimulation();
    printf("Simulation closed\n");

    printf("Done\n");
    return 0;
}