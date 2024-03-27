#ifndef STARQ_UNITREE__UNITREE_A1_MUJOCO_ROBOT_HPP_
#define STARQ_UNITREE__UNITREE_A1_MUJOCO_ROBOT_HPP_

#include "starq/mujoco/mujoco_robot.hpp"
#include "starq/unitree/unitree_a1_leg_dynamics.hpp"
#include "starq/unitree/unitree_a1_robot_parameters.hpp"
#include "starq/osqp/osqp.hpp"

#define UNITREE_A1_MUJOCO_SCENE_FILE "/home/nvidia/starq_ws/src/starq/models/unitree_a1/scene.xml"

#define UNITREE_A1_NUM_MOTORS 12
#define UNITREE_A1_NUM_LEGS 4

namespace starq::unitree
{
    using namespace starq::mujoco;
    using namespace starq::osqp;

    /// @brief Unitree A1 MuJoCo robot class
    class UnitreeA1MuJoCoRobot : public MuJoCoRobot
    {
    public:
        enum LegId
        {
            FL = 0,
            RL = 1,
            RR = 2,
            FR = 3
        };

        using Ptr = std::shared_ptr<UnitreeA1MuJoCoRobot>;

        /// @brief Constructor
        UnitreeA1MuJoCoRobot();

        /// @brief Get the left leg dynamics
        /// @return The left leg dynamics
        UnitreeA1LegDynamics::Ptr getLeftLegDynamics() const { return unitree_RRR_L_; }

        /// @brief Get the right leg dynamics
        /// @return The right leg dynamics
        UnitreeA1LegDynamics::Ptr getRightLegDynamics() const { return unitree_RRR_R_; }

        /// @brief Get the OSQP solver
        /// @return The OSQP solver
        OSQP::Ptr getOSQPSolver() const { return osqp_; }

    private:

        /// @brief Setup the motors
        void setupMotors() override;

        /// @brief Setup the legs
        void setupLegs() override;

        /// @brief Setup the robot dynamics
        void setupRobotDynamics() override;

        /// @brief Setup the model predictive control solver
        void setupMPCSolver() override;

        UnitreeA1LegDynamics::Ptr unitree_RRR_L_;
        UnitreeA1LegDynamics::Ptr unitree_RRR_R_;
        OSQP::Ptr osqp_;
    };

}

#endif