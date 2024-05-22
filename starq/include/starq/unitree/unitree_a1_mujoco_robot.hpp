#ifndef STARQ_UNITREE__UNITREE_A1_MUJOCO_ROBOT_HPP_
#define STARQ_UNITREE__UNITREE_A1_MUJOCO_ROBOT_HPP_

#include "starq/mujoco/mujoco_robot.hpp"
#include "starq/mujoco/mujoco_camera.hpp"
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

        /// @brief Get the front camera
        /// @return The front camera
        MuJoCoCamera::Ptr getFrontCamera() const { return front_camera_; }

        /// @brief Get the OSQP solver
        /// @return The OSQP solver
        OSQP::Ptr getOSQPSolver() const { return osqp_; }

        /// @brief Open the camera
        /// @return The future object
        std::future<void> &openCamera();

    private:

        /// @brief Setup the motors
        void setupMotorControllers() override;

        /// @brief Setup the legs
        void setupLegControllers() override;

        /// @brief Setup the robot dynamics
        void setupRobotParameters() override;

        /// @brief Setup the model predictive control solver
        void setupMPCSolver() override;

        /// @brief Setup the camera
        void setupCamera();

        UnitreeA1LegDynamics::Ptr unitree_RRR_L_;
        UnitreeA1LegDynamics::Ptr unitree_RRR_R_;
        MuJoCoCamera::Ptr front_camera_;
        OSQP::Ptr osqp_;

        std::future<void> camera_future_;
    };

}

#endif