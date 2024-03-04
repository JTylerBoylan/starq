#ifndef STARQ_DYNAMICS__UNITREE_A1_ROBOT_HPP_
#define STARQ_DYNAMICS__UNITREE_A1_ROBOT_HPP_

#include "starq/robot_dynamics.hpp"

#define UNITREE_A1_MASS 12.5
#define UNITREE_A1_INERTIA 0.016, 0, 0, 0, 0.038, 0, 0, 0, 0.046
#define UNITREE_A1_GRAVITY 0.0, 0.0, -9.81
#define UNITREE_A1_HEIGHT 0.27
#define UNITREE_A1_FOOT_FRICTION 0.8

#define UNITREE_A1_HIP_OFFSET_X 0.183
#define UNITREE_A1_HIP_OFFSET_Y 0.047

namespace starq::dynamics
{

    class UnitreeA1RobotDynamics : public RobotDynamics
    {
    public:
        using Ptr = std::shared_ptr<UnitreeA1RobotDynamics>;

        UnitreeA1RobotDynamics();

        ~UnitreeA1RobotDynamics();

        float getBodyMass() const override;

        Matrix3f getBodyInertia() const override;

        Vector3f getGravity() const override;

        float getBodyHeight() const override;

        float getFootFriction() const override;

        std::vector<Vector3f> getHipLocations() const override;

        float getForceZMin() const override;

        float getForceZMax() const override;

    private:
        float body_mass_;
        Matrix3f body_inertia_;
        Vector3f gravity_;
        float body_height_;
        float foot_friction_;
        std::vector<Vector3f> hip_locations_;
        float force_z_min_;
        float force_z_max_;
    };

}

#endif