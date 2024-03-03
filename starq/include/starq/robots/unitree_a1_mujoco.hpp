#ifndef STARQ_ROBOTS__UNITREE_A1_MUJOCO_HPP_
#define STARQ_ROBOTS__UNITREE_A1_MUJOCO_HPP_

#include "starq/mujoco/mujoco_robot.hpp"
#include "starq/dynamics/unitree_rrr.hpp"

#define UNITREE_A1_MUJOCO_SCENE_FILE "/home/nvidia/starq_ws/src/starq/models/unitree_a1/scene.xml"

#define UNITREE_A1_NUM_MOTORS 12
#define UNITREE_A1_NUM_LEGS 4

#define UNITREE_A1_LENGTH_D 0.08505
#define UNITREE_A1_LENGTH_LT 0.2
#define UNITREE_A1_LENGTH_LC 0.2

#define UNITREE_A1_MASS 12.5
#define UNITREE_A1_INERTIA 0.016, 0, 0, 0, 0.038, 0, 0, 0, 0.046
#define UNITREE_A1_GRAVITY 0.0, 0.0, -9.81
#define UNITREE_A1_HEIGHT 0.27
#define UNITREE_A1_FOOT_FRICTION 0.8

#define UNITREE_A1_HIP_OFFSET_X 0.183
#define UNITREE_A1_HIP_OFFSET_Y 0.047

namespace starq::robots
{
    using namespace starq::mujoco;
    using namespace starq::dynamics;

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

    private:

        /// @brief Setup the parameters
        void setupParams() override;

        /// @brief Setup the motors
        void setupMotors() override;

        /// @brief Setup the legs
        void setupLegs() override;

        Unitree_RRR::Ptr unitree_RRR_L_;
        Unitree_RRR::Ptr unitree_RRR_R_;
    };

}

#endif