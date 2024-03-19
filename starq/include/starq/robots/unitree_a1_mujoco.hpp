#ifndef STARQ_ROBOTS__UNITREE_A1_MUJOCO_HPP_
#define STARQ_ROBOTS__UNITREE_A1_MUJOCO_HPP_

#include "starq/mujoco/mujoco_robot.hpp"
#include "starq/dynamics/unitree_rrr.hpp"
#include "starq/dynamics/unitree_a1_robot.hpp"
#include "starq/osqp/osqp.hpp"

#define UNITREE_A1_MUJOCO_SCENE_FILE "/home/nvidia/starq_ws/src/starq/models/unitree_a1/scene.xml"

#define UNITREE_A1_NUM_MOTORS 12
#define UNITREE_A1_NUM_LEGS 4

namespace starq::robots
{
    using namespace starq::mujoco;
    using namespace starq::dynamics;
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
        Unitree_RRR::Ptr getLeftLegDynamics() const { return unitree_RRR_L_; }

        /// @brief Get the right leg dynamics
        /// @return The right leg dynamics
        Unitree_RRR::Ptr getRightLegDynamics() const { return unitree_RRR_R_; }

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

        Unitree_RRR::Ptr unitree_RRR_L_;
        Unitree_RRR::Ptr unitree_RRR_R_;
        OSQP::Ptr osqp_;
    };

}

#endif